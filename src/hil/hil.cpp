#include "hil.h"

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "esp_timer.h"
#include <math.h>

static const double HIL_RTF_WINDOW = 2.0;   // s of wall time per sim-speed measurement

static StreamBufferHandle_t gnss_stream = NULL;

static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static hil_imu_sample_t imu_sample;
static bool imu_valid = false;

void hil_init() {
  if (gnss_stream == NULL) gnss_stream = xStreamBufferCreate(1024, 1);
}

void hil_gnss_push(const uint8_t* data, size_t len) {
  if (gnss_stream == NULL) return;
  xStreamBufferSend(gnss_stream, data, len, 0);  // drop on overflow, like a UART FIFO
}

int hil_gnss_available() {
  return gnss_stream == NULL ? 0 : (int)xStreamBufferBytesAvailable(gnss_stream);
}

int hil_gnss_read() {
  uint8_t b;
  if (gnss_stream == NULL || xStreamBufferReceive(gnss_stream, &b, 1, 0) != 1) return -1;
  return b;
}

static portMUX_TYPE tof_mux = portMUX_INITIALIZER_UNLOCKED;
static tof_frame_t tof_frames[2];
static bool tof_new[2] = { false, false };

void hil_tof_push(const uint8_t* data, size_t len) {
  for (size_t at = 0; at + HIL_TOF_BLOCK <= len; at += HIL_TOF_BLOCK) {
    int sensor = data[at];
    if (sensor < 0 || sensor > 1) continue;
    tof_frame_t f;
    const uint8_t* z = data + at + 1;
    for (int k = 0; k < TOF_ZONES; k++, z += 3) {
      f.range_mm[k] = (uint16_t)(z[0] | (z[1] << 8));
      f.status[k] = z[2];
    }
    f.t_ms = millis();
    portENTER_CRITICAL(&tof_mux);
    tof_frames[sensor] = f;
    tof_new[sensor] = true;
    portEXIT_CRITICAL(&tof_mux);
  }
}

bool hil_tof_get(int sensor, tof_frame_t* frame) {
  portENTER_CRITICAL(&tof_mux);
  bool fresh = tof_new[sensor];
  if (fresh) *frame = tof_frames[sensor];
  tof_new[sensor] = false;
  portEXIT_CRITICAL(&tof_mux);
  return fresh;
}

void hil_imu_set(const hil_imu_sample_t& sample) {
  portENTER_CRITICAL(&imu_mux);
  imu_sample = sample;
  imu_valid = true;
  portEXIT_CRITICAL(&imu_mux);
}

bool hil_imu_get(hil_imu_sample_t* sample) {
  // The rate is the angle change over the simulator's stamps: the mean rate between two sim
  // samples, as a real gyro would read it. Over the ESP's arrival times instead (until
  // 2026-10-28) samples that came in a burst gave rates many times too high, and the trapezoid
  // in imu_task carried them over the next gap: every spin was over-counted by 20-30 %.
  // imu_task integrates over ESP (wall) time, so the sim rate is scaled by the sim's speed
  // (sim s per wall s, measured over HIL_RTF_WINDOW where bursts average out): at 0.4x every
  // turn was counted 2.5x and the wall map came out rotated (manual drive 2026-11-01).
  static double last_angle = 0.0, last_stamp = 0.0;
  static double rtf = 1.0, win_stamp = 0.0, win_wall = 0.0;
  portENTER_CRITICAL(&imu_mux);
  bool valid = imu_valid;
  *sample = imu_sample;
  portEXIT_CRITICAL(&imu_mux);
  if (!valid || sample->stamp <= 0.0) return valid;
  if (sample->stamp == last_stamp) return false;
  bool restarted = sample->stamp < last_stamp;   // simulator restarted: its angle starts over
  double wall = esp_timer_get_time() * 1e-6;
  if (restarted || win_wall == 0.0) {
    win_stamp = sample->stamp; win_wall = wall;
  } else if (wall - win_wall >= HIL_RTF_WINDOW) {
    rtf = fmin(1.2, fmax(0.1, (sample->stamp - win_stamp) / (wall - win_wall)));
    win_stamp = sample->stamp; win_wall = wall;
  }
  if (last_stamp > 0.0 && !restarted)
    sample->gyro_z = (float)(rtf * remainder(sample->gyro_z_angle - last_angle, 2.0 * M_PI) / (sample->stamp - last_stamp));
  last_angle = sample->gyro_z_angle;
  last_stamp = sample->stamp;
  return true;
}

static portMUX_TYPE power_mux = portMUX_INITIALIZER_UNLOCKED;
static float power_dock_v = 0.0f, power_battery_v = 0.0f;
static uint32_t power_ms = 0;

void hil_power_set(float dock_v, float battery_v) {
  portENTER_CRITICAL(&power_mux);
  power_dock_v = dock_v;
  power_battery_v = battery_v;
  power_ms = millis();
  portEXIT_CRITICAL(&power_mux);
}

bool hil_power_get(float* dock_v, float* battery_v) {
  portENTER_CRITICAL(&power_mux);
  uint32_t t = power_ms;
  *dock_v = power_dock_v;
  *battery_v = power_battery_v;
  portEXIT_CRITICAL(&power_mux);
  return t != 0 && millis() - t < 1000;
}
