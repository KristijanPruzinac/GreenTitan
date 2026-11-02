#include "tof_task.h"
#include "../hil/hil.h"
#include "../motion/wall_assist.h"
#include "tof_map.h"
#include <new>

#if !HIL_ENABLED
#include <Wire.h>
#include "vl53l8cx_api.h"   // ST ULD (lib stm32duino/STM32duino VL53L8CX); only its C API is used
#endif

#define TOF_TICK_MS 20   // frames are polled this often (15 Hz each)

static TofPose pose = {};
static float tilt_roll = 0.0f, tilt_pitch = 0.0f;   // rad, from /imu

static TofPoint* memory = nullptr;                  // heap: TOF_MEMORY_POINTS
static TofZone* zones = nullptr;                    // heap: one frame's zones
static bool map_ok = false;
static TofRay rays[2][TOF_ZONES];
static tof_frame_t latest[2];
static bool latest_valid[2] = { false, false };
static bool sensor_present[2] = { false, false };
static int sensors_found = 0;

const TofPoint* tof_memory() { return memory; }
const TofRay* tof_rays(int sensor) { return rays[sensor]; }
int tof_side_sensor(int side) { return TOF_MOUNT_YAW < 0.0f ? 1 - side : side; }
int tof_sensor_count() { return sensors_found; }

bool tof_latest_frame(int sensor, const tof_frame_t** frame) {
  *frame = &latest[sensor];
  return latest_valid[sensor] && millis() - latest[sensor].t_ms < TOF_FRAME_TIMEOUT_MS;
}

void tof_forget_point(int index) {
  if (memory && index >= 0 && index < TOF_MEMORY_POINTS) memory[index].t_ms = 0;
}

static void memory_clear() {
  for (int i = 0; memory && i < TOF_MEMORY_POINTS; i++) memory[i].t_ms = 0;
}

// Zone centre directions in the robot frame. tof_frame_t zone k = row * 8 + col, row 0 = top,
// col 0 = left seen from the sensor looking out along its +x: azimuth (3.5 - col) * FOV / 8
// (+ = left), elevation (3.5 - row) * FOV / 8 (+ = up). The ranges are radial along these.
static void build_rays() {
  for (int s = 0; s < 2; s++) {
    float side = s == 0 ? 1.0f : -1.0f;
    float c = cosf(side * TOF_MOUNT_YAW), sn = sinf(side * TOF_MOUNT_YAW);
    for (int k = 0; k < TOF_ZONES; k++) {
      float az = (3.5f - (k % 8)) * TOF_FOV / 8.0f, el = (3.5f - (k / 8)) * TOF_FOV / 8.0f;
      float sx = cosf(el) * cosf(az), sy = cosf(el) * sinf(az);
      rays[s][k] = { TOF_MOUNT_X, side * TOF_MOUNT_Y, TOF_MOUNT_Z, c * sx - sn * sy, sn * sx + c * sy, sinf(el) };
    }
  }
}

// ---------------------------------------------------------------- REAL SENSORS (I2C)
#if !HIL_ENABLED
// ULD platform over Wire1 (the MPU6050 keeps Wire alone, so imu_task timing is untouched). Every
// call returns an error instead of retrying: the library's own Arduino class loops forever on a
// NACK, which would hang the task with a sensor unplugged.
static VL53L8CX_Configuration* dev[2] = { nullptr, nullptr };
static VL53L8CX_ResultsData* results = nullptr;

#define TOF_I2C_CHUNK 120   // payload per Wire transaction (128 B buffer, 2 B register address)

static uint8_t i2c_write(void* handle, uint16_t reg, uint8_t* values, uint32_t size) {
  uint8_t addr = ((VL53L8CX_Platform*)handle)->address >> 1;
  for (uint32_t i = 0; i < size; i += TOF_I2C_CHUNK) {
    uint32_t n = size - i < TOF_I2C_CHUNK ? size - i : TOF_I2C_CHUNK;
    Wire1.beginTransmission(addr);
    Wire1.write((uint8_t)((reg + i) >> 8));
    Wire1.write((uint8_t)((reg + i) & 0xFF));
    Wire1.write(values + i, n);
    if (Wire1.endTransmission(true) != 0) return 1;
  }
  return 0;
}

// Each chunk re-sends its register address, so it does not rely on the read pointer running on
static uint8_t i2c_read(void* handle, uint16_t reg, uint8_t* values, uint32_t size) {
  uint8_t addr = ((VL53L8CX_Platform*)handle)->address >> 1;
  for (uint32_t i = 0; i < size; i += TOF_I2C_CHUNK) {
    uint32_t n = size - i < TOF_I2C_CHUNK ? size - i : TOF_I2C_CHUNK;
    Wire1.beginTransmission(addr);
    Wire1.write((uint8_t)((reg + i) >> 8));
    Wire1.write((uint8_t)((reg + i) & 0xFF));
    if (Wire1.endTransmission(false) != 0) return 1;
    if (Wire1.requestFrom(addr, (size_t)n) != n) return 1;
    for (uint32_t k = 0; k < n; k++) values[i + k] = Wire1.read();
  }
  return 0;
}

static uint8_t i2c_wait(void* handle, uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
  return 0;
}

static bool i2c_probe(uint8_t addr8) {
  Wire1.beginTransmission(addr8 >> 1);
  return Wire1.endTransmission(true) == 0;
}

// The sensor on its LPn pin is the only one at the default address: raise it, move it to `address`
static bool start_sensor(int s, int lpn_pin, uint16_t address) {
  const char* name = s == 0 ? "left" : "right";
  digitalWrite(lpn_pin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  if (!i2c_probe(VL53L8CX_DEFAULT_I2C_ADDRESS)) {
    SerialDebug.printf("[TOF] %s sensor not found (LPn GPIO%d)\r\n", name, lpn_pin);
    return false;
  }
  VL53L8CX_Configuration* d = new (std::nothrow) VL53L8CX_Configuration();
  if (d == nullptr) { SerialDebug.println("[TOF] Out of memory"); return false; }
  d->platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
  d->platform.Write = i2c_write;
  d->platform.Read = i2c_read;
  d->platform.Wait = i2c_wait;
  d->platform.handle = &d->platform;
  uint8_t alive = 0, status = 0;
  if (address != VL53L8CX_DEFAULT_I2C_ADDRESS) status |= vl53l8cx_set_i2c_address(d, address);
  status |= vl53l8cx_is_alive(d, &alive);
  if (status || !alive) {
    SerialDebug.printf("[TOF] %s sensor does not answer at 0x%02X\r\n", name, d->platform.address);
    digitalWrite(lpn_pin, LOW);   // off the bus, so it can't clash with the other one
    delete d;
    return false;
  }
  uint32_t t0 = millis();
  status |= vl53l8cx_init(d);   // uploads the sensor firmware (~84 kB)
  status |= vl53l8cx_set_resolution(d, VL53L8CX_RESOLUTION_8X8);
  status |= vl53l8cx_set_ranging_frequency_hz(d, TOF_RANGING_FREQUENCY);
  status |= vl53l8cx_set_target_order(d, VL53L8CX_TARGET_ORDER_CLOSEST);
  status |= vl53l8cx_set_ranging_mode(d, VL53L8CX_RANGING_MODE_CONTINUOUS);
  status |= vl53l8cx_start_ranging(d);
  if (status) {
    SerialDebug.printf("[TOF] %s sensor setup failed (status %d)\r\n", name, status);
    digitalWrite(lpn_pin, LOW);
    delete d;
    return false;
  }
  SerialDebug.printf("[TOF] %s sensor ranging at 0x%02X (init %lu ms)\r\n", name, d->platform.address,
                     (unsigned long)(millis() - t0));
  dev[s] = d;
  return true;
}

static void init_sensors() {
  pinMode(TOF_LPN_LEFT_PIN, OUTPUT);
  pinMode(TOF_LPN_RIGHT_PIN, OUTPUT);
  digitalWrite(TOF_LPN_LEFT_PIN, LOW);
  digitalWrite(TOF_LPN_RIGHT_PIN, LOW);
  Wire1.begin(TOF_SDA_PIN, TOF_SCL_PIN, TOF_I2C_FREQUENCY);
  Wire1.setTimeOut(20);
  vTaskDelay(pdMS_TO_TICKS(10));
  sensor_present[0] = start_sensor(0, TOF_LPN_LEFT_PIN, TOF_I2C_ADDRESS_LEFT);
  sensor_present[1] = start_sensor(1, TOF_LPN_RIGHT_PIN, VL53L8CX_DEFAULT_I2C_ADDRESS);
  if (sensor_present[0] || sensor_present[1]) results = new (std::nothrow) VL53L8CX_ResultsData();
  if (results == nullptr) sensor_present[0] = sensor_present[1] = false;
}

// ULD zone u = r * 8 + c -> tof_frame_t zone; TOF_ULD_* say how the ULD image is turned (to verify)
static bool read_sensor(int s, tof_frame_t* f) {
  if (!sensor_present[s]) return false;
  uint8_t ready = 0;
  if (vl53l8cx_check_data_ready(dev[s], &ready) || !ready) return false;
  if (vl53l8cx_get_ranging_data(dev[s], results)) return false;
  for (int u = 0; u < TOF_ZONES; u++) {
    int r = u / 8, c = u % 8;
    int k = (TOF_ULD_FLIP_ROWS ? 7 - r : r) * 8 + (TOF_ULD_MIRROR_COLS ? 7 - c : c);
    uint8_t st = results->target_status[u * VL53L8CX_NB_TARGET_PER_ZONE];
    int16_t mm = results->distance_mm[u * VL53L8CX_NB_TARGET_PER_ZONE];
    f->range_mm[k] = mm > 0 ? mm : 0;
    f->status[k] = results->nb_target_detected[u] == 0 ? TOF_STATUS_NO_TARGET :
                   (st == 5 || st == 9) ? TOF_STATUS_VALID : st;   // ST: 5 and 9 are valid ranges
  }
  f->t_ms = millis();
  return true;
}
#endif

static bool next_frame(int s, tof_frame_t* f) {
#if HIL_ENABLED
  return hil_tof_get(s, f);
#else
  return read_sensor(s, f);
#endif
}

// ---------------------------------------------------------------- POINTS

static uint8_t to_cm(float m) { return (uint8_t)fminf(fmaxf(m * 100.0f + 0.5f, 0.0f), 255.0f); }

static void memory_add(const TofZone& z, uint32_t now) {
  float x = z.x, y = z.y;
  int16_t cx = (int16_t)floorf(x / TOF_GRID), cy = (int16_t)floorf(y / TOF_GRID);
  uint8_t top = (z.flags & TOF_ZONE_TOP) ? to_cm(0.5f * (z.top_lo + z.top_hi)) : 0;
  int free_slot = -1, oldest = 0;
  for (int i = 0; i < TOF_MEMORY_POINTS; i++) {
    TofPoint& p = memory[i];
    if (p.t_ms == 0) { if (free_slot < 0) free_slot = i; continue; }
    if (p.cx == cx && p.cy == cy) { p.x = x; p.y = y; p.t_ms = now; p.flags = z.flags; p.z_cm = to_cm(z.z); p.top_cm = top; return; }
    if (p.t_ms < memory[oldest].t_ms || memory[oldest].t_ms == 0) oldest = i;
  }
  int i = free_slot >= 0 ? free_slot : oldest;
  memory[i] = { x, y, cx, cy, now, z.flags, to_cm(z.z), top };
}

// Old points, far points and points under the body (driven over: not solid) are forgotten
static void memory_prune(uint32_t now) {
  float c = cosf(pose.yaw), s = sinf(pose.yaw);
  for (int i = 0; i < TOF_MEMORY_POINTS; i++) {
    TofPoint& p = memory[i];
    if (p.t_ms == 0) continue;
    float dx = p.x - (float)pose.x, dy = p.y - (float)pose.y;
    float rx = c * dx + s * dy, ry = -s * dx + c * dy;
    bool under = rx > -MOWER_REAR_OVERHANG && rx < MOWER_FRONT_OVERHANG && fabsf(ry) < MOWER_HALF_WIDTH;
    if (now - p.t_ms > TOF_MEMORY_AGE_MS || dx * dx + dy * dy > TOF_MEMORY_RADIUS * TOF_MEMORY_RADIUS || under)
      p.t_ms = 0;
  }
}

// Valid zones -> world points (tof_frame_analyse). The pose is carried from its arrival to the
// frame's measure time with its own speed and turn rate; roll and pitch tilt the body. The memory
// keeps the zones above the plain ground threshold and the face points (low edges too); the map
// takes the face points.
static void add_frame(int s, const tof_frame_t& f) {
  if (!pose.valid || millis() - pose.ms > MOTION_POSE_TIMEOUT_MS) return;   // can't place the points
  float dt = ((int32_t)(f.t_ms - pose.ms) - TOF_FRAME_DELAY * 1000.0f) * 1e-3f;
  TofFramePose fp;
  fp.x = pose.x + pose.vx * cosf(pose.yaw) * dt;
  fp.y = pose.y + pose.vx * sinf(pose.yaw) * dt;
  fp.yaw = pose.yaw + pose.omega * dt;
  fp.roll = tilt_roll; fp.pitch = tilt_pitch; fp.omega = pose.omega;
  tof_frame_analyse(rays[s], f, fp, zones);
  for (int k = 0; k < TOF_ZONES; k++)
    if (zones[k].flags & (TOF_ZONE_HIGH | TOF_ZONE_FACE)) memory_add(zones[k], f.t_ms);
  // Not with the boot heading: standing at boot the guess wanders by degrees, and every frame stamped a narrow
  // object (a wall end) at another bearing until the smear became a confirmed line across the view (HIL 2026-11-01)
  if (map_ok && HEADING_CONFIRMED) tof_map_frame(rays[s], f, fp, zones, f.t_ms);
}

// ---------------------------------------------------------------- DDS

static void pose_callback(dds_callback_context_t* context) {
  fused_pose_data_t* d = (fused_pose_data_t*)context->message_data.data;
  uint32_t now = millis();
  double x = d->x + GNSS_ANTENNA_BEHIND_AXLE * cos(d->yaw), y = d->y + GNSS_ANTENNA_BEHIND_AXLE * sin(d->yaw);
  // A jump (RTK fix change, EKF reset) would smear the memory: start it over
  if (pose.valid && now - pose.ms < MOTION_POSE_TIMEOUT_MS) {
    float dt = (now - pose.ms) * 1e-3f;
    double jump = hypot(x - (pose.x + pose.vx * cosf(pose.yaw) * dt), y - (pose.y + pose.vx * sinf(pose.yaw) * dt));
    if (jump > TOF_POSE_JUMP) {
      memory_clear();
      tof_map_pose_jump();
      SerialDebug.printf("[TOF] Pose jumped %.0f cm, point memory cleared, map kept apart\r\n", jump * 100.0);
    }
  }
  pose = { x, y, d->yaw, d->vx, d->omega, now, true };
}

// Each tick: the EKF pose projected to now (fused_pose_now: gyro yaw and EKF velocity since it arrived, 2026-11-02),
// not the 10 Hz /fused_pose as it was up to 100 ms ago. Stale or none: the last one stays (and times out)
static void pose_refresh() {
  fused_pose_data_t d;
  if (!fused_pose_now(&d)) return;
  double x = d.x + GNSS_ANTENNA_BEHIND_AXLE * cos(d.yaw), y = d.y + GNSS_ANTENNA_BEHIND_AXLE * sin(d.yaw);
  pose = { x, y, d.yaw, d.vx, d.omega, millis(), true };
}

static void imu_callback(dds_callback_context_t* context) {
  IMU_data_t* d = (IMU_data_t*)context->message_data.data;
  tilt_roll = d->roll;
  tilt_pitch = d->pitch;
}

// ---------------------------------------------------------------- TASK

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void tof_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();

    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));

    // ------- THREAD SETUP CODE START -------

    build_rays();
    memory = new (std::nothrow) TofPoint[TOF_MEMORY_POINTS]();
    zones = new (std::nothrow) TofZone[TOF_ZONES]();
    map_ok = zones && tof_map_init();
#if HIL_ENABLED
    sensor_present[0] = sensor_present[1] = true;   // frames come from hil/tof
#else
    init_sensors();   // here, not in setup(): the firmware upload takes seconds and needs no one else
#endif
    sensors_found = (sensor_present[0] ? 1 : 0) + (sensor_present[1] ? 1 : 0);
    if (memory == nullptr || zones == nullptr) sensors_found = 0;
    else if (!map_ok) SerialDebug.println("[TOF] Out of memory for the map");
    if (sensors_found == 0) SerialDebug.println("[TOF] No sensors, ToF and wall assist off");
    wall_assist_init(sensors_found > 0);

    dds_result_t result = DDS_SUBSCRIBE("/fused_pose", pose_callback, &thread_context);
    if (result != DDS_SUCCESS) SerialDebug.printf("[TOF] Fused pose subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    result = DDS_SUBSCRIBE("/imu", imu_callback, &thread_context);
    if (result != DDS_SUCCESS) SerialDebug.printf("[TOF] IMU subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));

    esp_timer_start_periodic(thread_context.timer, TOF_TICK_MS * 1000);

    // ------- THREAD SETUP CODE END -------

    vTaskDelay(500);

    while(1) {
        uint32_t notification_value;
        xTaskNotifyWait(0x00, 0xFF, &notification_value, portMAX_DELAY);

        if (notification_value & DDS_NOTIFY_BIT) {
            DDS_TAKE_MUTEX(&thread_context);
            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
        if (notification_value & THREAD_NOTIFY_BIT) {
            DDS_TAKE_MUTEX(&thread_context);

            // ------- THREAD LOOP CODE START -------

            if (sensors_found > 0) {
              // Heading just confirmed: the points placed with the guess are dropped (the guard rebuilds its
              // view within a frame); after a relock the old map is kept apart
              static uint32_t heading_epoch = 0;
              if (HEADING_EPOCH != heading_epoch) {
                heading_epoch = HEADING_EPOCH;
                memory_clear();
                tof_map_pose_jump();
                SerialDebug.printf("[TOF] Heading confirmed: point memory cleared\r\n");
              }
              pose_refresh();
              bool new_frame = false;
              tof_frame_t f;
              for (int s = 0; s < 2; s++) {
                if (!next_frame(s, &f)) continue;
                latest[s] = f;
                latest_valid[s] = true;
                add_frame(s, f);
                new_frame = true;
              }
              if (new_frame && pose.valid) memory_prune(millis());
              wall_assist_update(pose, new_frame);
              static uint32_t map_ms = 0;
              if (map_ok && pose.valid && millis() - map_ms >= TOF_MAP_UPDATE_MS) {
                map_ms = millis();
                TofFramePose fp = { pose.x, pose.y, pose.yaw, tilt_roll, tilt_pitch, pose.omega };
                tof_map_update(fp, millis());
              }
            }

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}
