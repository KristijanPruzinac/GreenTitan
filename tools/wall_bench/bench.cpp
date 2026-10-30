// Wall bench firmware side: the real ToF task and wall assist (HIL build, frames through
// hil_tof_push) on a simulated clock, driven over stdin/stdout by bench.py.
//
// Lockstep: every FreeRTOS task is a thread, but only one of them or the bench runs at a time. A
// task runs until it blocks (xTaskNotifyWait, vTaskDelay, an empty queue), then the bench goes on.
// Time only moves when the bench advances it, so a run is the same every time.
//
// Also here, minimal copies of what main.cpp does around the assist: publish_manual_move (the
// 'v' command), the MANUAL_WATCHDOG_MS stop, the stop key, 'm' (manual mode, normally via the
// controller), 'c'/'o'/'e' (capture: wall points are printed only while capturing).
//
// stdin, one command per line:        stdout:
//   P x y yaw vx omega   /fused_pose     motor <t_us> <instruction> <v> <w>   (every /motor message)
//   I roll pitch         /imu            log <t_ms> <text>                    (SerialDebug lines)
//   F <hex>              hil_tof_push    map <kind> <x0> <y0> <x1> <y1> ...   (after M, if built in)
//   V lin ang            'v' command     ok <t_us>                            (after T and M)
//   K <key>              m f i c o e s (s = space, the stop key)
//   T <t_us>             run to this time
//   M                    print the ToF map
//   Q                    quit

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include "Arduino.h"
#include "freertos/stream_buffer.h"
#include "definitions.h"
#include "esp_dds.h"
#include "debug/debug_serial.h"
#include "hil/hil.h"
#include "motion/wall_assist.h"
#include "sensor/tof_task.h"

void bench_map_print() __attribute__((weak));   // map_hook.cpp, when the firmware has a ToF map

// ---------------------------------------------------------------- CLOCK + SCHEDULER

static uint64_t now_us = 5000000;   // boot took a while: millis() never starts at 0 on the ESP
static const uint64_t NEVER = UINT64_MAX;

struct sim_task {
  std::string name;
  TaskFunction_t fn = nullptr;
  void* param = nullptr;
  bool virt = false;          // the bench's own subscriber: never runs, only collects notifications
  bool started = false, blocked = false, finished = false;
  uint64_t wake_us = NEVER;
  std::function<bool()> ready;
  uint32_t notify_value = 0;
  bool notified = false;
};

struct sim_timer {
  esp_timer_cb_t cb;
  void* arg;
  uint64_t period = 0, next = NEVER;
};

static std::mutex lock_m;
static std::condition_variable lock_cv;
static sim_task* running = nullptr;
static thread_local sim_task* current = nullptr;
static std::vector<sim_task*> tasks;
static std::vector<sim_timer*> timers;

static void task_entry(sim_task* t) {
  current = t;
  {
    std::unique_lock<std::mutex> lk(lock_m);
    lock_cv.wait(lk, [t] { return running == t; });
  }
  t->fn(t->param);
  std::unique_lock<std::mutex> lk(lock_m);
  t->finished = true;
  running = nullptr;
  lock_cv.notify_all();
}

// The calling task blocks until ready() or the wake time; the bench goes on meanwhile
static void task_block(uint64_t wake, std::function<bool()> ready) {
  sim_task* t = current;
  if (t == nullptr) return;   // the bench itself never blocks
  std::unique_lock<std::mutex> lk(lock_m);
  t->blocked = true;
  t->wake_us = wake;
  t->ready = ready;
  running = nullptr;
  lock_cv.notify_all();
  lock_cv.wait(lk, [t] { return running == t; });
  t->blocked = false;
  t->ready = nullptr;
}

static void task_resume(sim_task* t) {
  std::unique_lock<std::mutex> lk(lock_m);
  running = t;
  lock_cv.notify_all();
  lock_cv.wait(lk, [] { return running == nullptr; });
}

static sim_task* task_spawn(const char* name, TaskFunction_t fn, void* param) {
  sim_task* t = new sim_task();
  t->name = name; t->fn = fn; t->param = param;
  tasks.push_back(t);
  std::thread(task_entry, t).detach();
  return t;
}

// Runs every task that can run now, until all are blocked again
static void run_ready() {
  for (int guard = 0; guard < 100000; guard++) {
    sim_task* pick = nullptr;
    for (sim_task* t : tasks) {
      if (t->virt || t->finished) continue;
      if (!t->started || (t->blocked && (now_us >= t->wake_us || (t->ready && t->ready())))) { pick = t; break; }
    }
    if (pick == nullptr) return;
    pick->started = true;
    task_resume(pick);
  }
  fprintf(stderr, "bench: a task never blocks\n");
  _exit(2);
}

// ---------------------------------------------------------------- FREERTOS

BaseType_t xTaskNotify(TaskHandle_t t, uint32_t value, eNotifyAction action) {
  if (t == nullptr) return pdFAIL;
  if (action == eSetBits) t->notify_value |= value;
  else if (action == eIncrement) t->notify_value++;
  else if (action == eSetValueWithOverwrite || action == eSetValueWithoutOverwrite) t->notify_value = value;
  t->notified = true;
  return pdPASS;
}

BaseType_t xTaskNotifyWait(uint32_t clear_on_entry, uint32_t clear_on_exit, uint32_t* value, TickType_t ticks) {
  sim_task* t = current;
  if (t == nullptr) return pdFALSE;
  if (!t->notified) {
    t->notify_value &= ~clear_on_entry;
    if (ticks == 0) return pdFALSE;
    task_block(ticks == portMAX_DELAY ? NEVER : now_us + (uint64_t)ticks * 1000, [t] { return t->notified; });
    if (!t->notified) {
      if (value) *value = t->notify_value;
      return pdFALSE;
    }
  }
  if (value) *value = t->notify_value;
  t->notify_value &= ~clear_on_exit;
  t->notified = false;
  return pdTRUE;
}

TaskHandle_t xTaskGetCurrentTaskHandle() { return current; }
void vTaskDelay(TickType_t ticks) { task_block(now_us + (uint64_t)ticks * 1000, nullptr); }
TickType_t xTaskGetTickCount() { return (TickType_t)(now_us / 1000); }

BaseType_t xTaskCreate(TaskFunction_t fn, const char* name, uint32_t, void* param, UBaseType_t, TaskHandle_t* out) {
  sim_task* t = task_spawn(name, fn, param);
  if (out) *out = t;
  return pdPASS;
}
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name, uint32_t stack, void* param, UBaseType_t prio,
                                   TaskHandle_t* out, BaseType_t) {
  return xTaskCreate(fn, name, stack, param, prio, out);
}

struct sim_queue {
  size_t item, length;
  std::deque<std::vector<uint8_t>> items;
  bool mutex = false;
  int count = 0;   // semaphores
};

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size) {
  sim_queue* q = new sim_queue();
  q->item = item_size; q->length = length;
  return q;
}
BaseType_t xQueueSend(QueueHandle_t q, const void* item, TickType_t) {
  if (q->items.size() >= q->length) return pdFALSE;   // a full queue drops (callers use timeout 0)
  const uint8_t* p = (const uint8_t*)item;
  q->items.emplace_back(p, p + q->item);
  return pdTRUE;
}
BaseType_t xQueueOverwrite(QueueHandle_t q, const void* item) {
  if (!q->items.empty() && q->items.size() >= q->length) q->items.pop_back();
  return xQueueSend(q, item, 0);
}
static BaseType_t queue_get(QueueHandle_t q, void* item, TickType_t ticks, bool remove) {
  if (q->items.empty() && ticks > 0)
    task_block(ticks == portMAX_DELAY ? NEVER : now_us + (uint64_t)ticks * 1000, [q] { return !q->items.empty(); });
  if (q->items.empty()) return pdFALSE;
  memcpy(item, q->items.front().data(), q->item);
  if (remove) q->items.pop_front();
  return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q, void* item, TickType_t ticks) { return queue_get(q, item, ticks, true); }
BaseType_t xQueuePeek(QueueHandle_t q, void* item, TickType_t ticks) { return queue_get(q, item, ticks, false); }
BaseType_t xQueueReset(QueueHandle_t q) { q->items.clear(); return pdPASS; }
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q) { return (UBaseType_t)q->items.size(); }
void vQueueDelete(QueueHandle_t q) { delete q; }

SemaphoreHandle_t xSemaphoreCreateMutex() { sim_queue* s = new sim_queue(); s->mutex = true; s->count = 1; return s; }
SemaphoreHandle_t xSemaphoreCreateBinary() { sim_queue* s = new sim_queue(); s->count = 0; return s; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks) {
  if (s->count <= 0 && ticks > 0)
    task_block(ticks == portMAX_DELAY ? NEVER : now_us + (uint64_t)ticks * 1000, [s] { return s->count > 0; });
  if (s->count <= 0) return pdFALSE;
  s->count--;
  return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t s) {
  if (s->mutex && s->count >= 1) return pdFALSE;
  s->count++;
  return pdTRUE;
}

struct sim_stream { size_t size; std::deque<uint8_t> bytes; };
StreamBufferHandle_t xStreamBufferCreate(size_t size, size_t) { sim_stream* s = new sim_stream(); s->size = size; return s; }
size_t xStreamBufferSend(StreamBufferHandle_t s, const void* data, size_t len, TickType_t) {
  size_t n = std::min(len, s->size - s->bytes.size());
  for (size_t i = 0; i < n; i++) s->bytes.push_back(((const uint8_t*)data)[i]);
  return n;
}
size_t xStreamBufferReceive(StreamBufferHandle_t s, void* data, size_t len, TickType_t) {
  size_t n = std::min(len, s->bytes.size());
  for (size_t i = 0; i < n; i++) { ((uint8_t*)data)[i] = s->bytes.front(); s->bytes.pop_front(); }
  return n;
}
size_t xStreamBufferBytesAvailable(StreamBufferHandle_t s) { return s->bytes.size(); }

// ---------------------------------------------------------------- ESP_TIMER, ARDUINO

esp_err_t esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* out) {
  sim_timer* t = new sim_timer{ args->callback, args->arg };
  timers.push_back(t);
  *out = t;
  return ESP_OK;
}
esp_err_t esp_timer_start_periodic(esp_timer_handle_t t, uint64_t period_us) { t->period = period_us; t->next = now_us + period_us; return ESP_OK; }
esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t timeout_us) { t->period = 0; t->next = now_us + timeout_us; return ESP_OK; }
esp_err_t esp_timer_stop(esp_timer_handle_t t) { t->next = NEVER; return ESP_OK; }
esp_err_t esp_timer_delete(esp_timer_handle_t t) { t->next = NEVER; return ESP_OK; }
int64_t esp_timer_get_time() { return (int64_t)now_us; }

uint32_t millis() { return (uint32_t)(now_us / 1000); }
uint32_t micros() { return (uint32_t)now_us; }
void delay(uint32_t ms) { vTaskDelay(ms); }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return 0; }
static uint32_t random_state = 0x12345678u;
uint32_t esp_random() { random_state ^= random_state << 13; random_state ^= random_state >> 17; random_state ^= random_state << 5; return random_state; }

// Serial output -> "log <t_ms> <line>"
static std::string line_buf;
static void log_char(uint8_t c) {
  if (c == '\r') return;
  if (c == '\n') { printf("log %u %s\n", millis(), line_buf.c_str()); line_buf.clear(); return; }
  line_buf.push_back((char)c);
}
size_t HardwareSerial::write(uint8_t c) { log_char(c); return 1; }
size_t HardwareSerial::write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) log_char(b[i]); return n; }
size_t DebugSerial::write(uint8_t c) { log_char(c); return 1; }
size_t DebugSerial::write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) log_char(b[i]); return n; }
int DebugSerial::available() { return 0; }
int DebugSerial::read() { return -1; }
int DebugSerial::peek() { return -1; }
size_t DebugSerial::drain_output(char*, size_t) { return 0; }
void DebugSerial::inject_input(const char*, size_t) {}

HardwareSerial Serial(0);
DebugSerial SerialDebug(1);

// ---------------------------------------------------------------- MAIN.CPP, MINIMAL

volatile bool TASKS_STARTED = false;
volatile bool MANUAL_MODE_ACTIVE = false;
volatile uint32_t LAST_MANUAL_MOVE_MS = 0;
volatile bool MANUAL_MOTORS_STOPPED = true;
static bool capture_active = false;

static void publish_manual_move(float lin, float ang) {   // main.cpp
  if (!MANUAL_MODE_ACTIVE) {
    SerialDebug.println("[DEBUG] Not in manual mode (press 'm' first)");
    return;
  }
  if (!wall_assist_manual(lin, ang)) {
    motor_data_t cmd = { MOTOR_MOVE, lin, ang };
    DDS_PUBLISH("/motor", cmd);
  }
  LAST_MANUAL_MOVE_MS = millis();
  MANUAL_MOTORS_STOPPED = false;
}

static void key(char c) {
  if (c == 'm') {   // controller enter/exit_manual_mode, without the IDLE check
    motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
    DDS_PUBLISH("/motor", stop);
    MANUAL_MODE_ACTIVE = !MANUAL_MODE_ACTIVE;
    MANUAL_MOTORS_STOPPED = true;
    SerialDebug.println(MANUAL_MODE_ACTIVE ? "[CTRL] Manual mode ON" : "[CTRL] Manual mode OFF");
  } else if (c == 's') {
    wall_assist_release();
    if (MANUAL_MODE_ACTIVE) {
      motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
      DDS_PUBLISH("/motor", stop);
      MANUAL_MOTORS_STOPPED = true;
      SerialDebug.println("[DEBUG] Manual stop");
    }
  } else if (c == 'c') { capture_active = true; SerialDebug.println("[DEBUG] Path capture started"); }
  else if (c == 'o') SerialDebug.println("[DEBUG] New outline started");
  else if (c == 'e') { capture_active = false; SerialDebug.println("[DEBUG] Recording ended"); }
  else if (c == 'f') wall_assist_toggle();
  else if (c == 'i') wall_assist_print_status();
}

static void main_tick() {   // main.cpp loop: the manual watchdog and the wall assist's outline points
  if (MANUAL_MODE_ACTIVE && !MANUAL_MOTORS_STOPPED && (int32_t)(millis() - LAST_MANUAL_MOVE_MS) > MANUAL_WATCHDOG_MS) {
    motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
    DDS_PUBLISH("/motor", stop);
    MANUAL_MOTORS_STOPPED = true;
  }
  long long wx, wy;
  bool wcorner;
  while (wall_assist_take_point(&wx, &wy, &wcorner)) {
    if (!capture_active) continue;
    SerialDebug.printf("[WALL] Point added (%lld, %lld) cm%s\r\n", wx, wy, wcorner ? " (corner)" : "");
  }
}

// /motor as the motor task gets it
static dds_thread_context_t bench_context;
static void motor_callback(dds_callback_context_t* c) {
  motor_data_t* d = (motor_data_t*)c->message_data.data;
  printf("motor %lld %d %.5f %.5f\n", (long long)c->message_data.timestamp, (int)d->instruction, d->linear_vel, d->angular_vel);
}
static void drain_motor() { DDS_PROCESS_THREAD_MESSAGES(&bench_context); }

#define MAIN_TICK_US 10000
static uint64_t next_main = 0;

static void advance(uint64_t until) {
  for (;;) {
    run_ready();
    drain_motor();
    uint64_t next = next_main;
    for (sim_timer* t : timers) next = std::min(next, t->next);
    for (sim_task* t : tasks) if (t->blocked) next = std::min(next, t->wake_us);
    if (next > until) break;
    if (next > now_us) now_us = next;
    for (sim_timer* t : timers) {
      if (t->next > now_us) continue;
      t->next = t->period ? t->next + t->period : NEVER;
      t->cb(t->arg);
    }
    if (now_us >= next_main) { main_tick(); next_main += MAIN_TICK_US; }
  }
  now_us = std::max(now_us, until);
  run_ready();
  drain_motor();
}

static int hexval(char c) { return c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10; }

int main() {
  setvbuf(stdout, nullptr, _IOFBF, 1 << 16);
  DDS_INIT();
  hil_init();
  sim_task* bench = new sim_task();
  bench->name = "bench"; bench->virt = true;
  bench_context.task = bench;
  bench_context.queue = xQueueCreate(64, sizeof(dds_callback_context_t));
  bench_context.sync_mutex = xSemaphoreCreateMutex();
  DDS_SUBSCRIBE("/motor", motor_callback, &bench_context);
  TASKS_STARTED = true;
  xTaskCreatePinnedToCore(tof_task, "tof_task", 8192, NULL, 1, NULL, 1);
  next_main = now_us;
  run_ready();

  char line[4096];
  std::vector<uint8_t> bytes;
  while (fgets(line, sizeof(line), stdin)) {
    char cmd = line[0];
    if (cmd == 'P') {
      fused_pose_data_t p = {};
      double x, y, yaw, vx, w;
      if (sscanf(line + 1, "%lf %lf %lf %lf %lf", &x, &y, &yaw, &vx, &w) == 5) {
        p.x = x; p.y = y; p.yaw = (float)yaw; p.vx = (float)vx; p.omega = (float)w;
        DDS_PUBLISH("/fused_pose", p);
      }
    } else if (cmd == 'I') {
      IMU_data_t d = {};
      double r, p;
      if (sscanf(line + 1, "%lf %lf", &r, &p) == 2) {
        d.acc_z = 9.81f; d.roll = (float)r; d.pitch = (float)p;
        d.timestamp_sec = (uint32_t)(now_us / 1000000); d.timestamp_nsec = (uint32_t)(now_us % 1000000) * 1000;
        DDS_PUBLISH("/imu", d);
      }
    } else if (cmd == 'F') {
      bytes.clear();
      const char* h = line + 2;
      while (h[0] && h[1] && h[0] != '\n') { bytes.push_back((uint8_t)(hexval(h[0]) * 16 + hexval(h[1]))); h += 2; }
      hil_tof_push(bytes.data(), bytes.size());
    } else if (cmd == 'V') {
      float lin, ang;
      if (sscanf(line + 1, "%f %f", &lin, &ang) == 2) publish_manual_move(lin, ang);
    } else if (cmd == 'K') {
      key(line[2]);
    } else if (cmd == 'T') {
      unsigned long long t;
      if (sscanf(line + 1, "%llu", &t) == 1) advance(t);
      printf("ok %llu\n", (unsigned long long)now_us);
      fflush(stdout);
      continue;
    } else if (cmd == 'M') {
      if (bench_map_print) bench_map_print();
      printf("ok %llu\n", (unsigned long long)now_us);
      fflush(stdout);
      continue;
    } else if (cmd == 'Q') {
      break;
    }
    run_ready();   // a published message wakes the task at once, as on the ESP
    drain_motor();
  }
  fflush(stdout);
  _exit(0);
}
