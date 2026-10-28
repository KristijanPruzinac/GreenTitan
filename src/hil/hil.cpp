#include "hil.h"

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"

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
  // in imu_task carried them over the next gap: every spin was over-counted by 20-30 %. The
  // price is a heading error in proportion when the simulation runs off real time.
  static double last_angle = 0.0, last_stamp = 0.0;
  portENTER_CRITICAL(&imu_mux);
  bool valid = imu_valid;
  *sample = imu_sample;
  portEXIT_CRITICAL(&imu_mux);
  if (!valid || sample->stamp <= 0.0) return valid;
  if (sample->stamp == last_stamp) return false;
  bool restarted = sample->stamp < last_stamp;   // simulator restarted: its angle starts over
  if (last_stamp > 0.0 && !restarted)
    sample->gyro_z = (float)(remainder(sample->gyro_z_angle - last_angle, 2.0 * M_PI) / (sample->stamp - last_stamp));
  last_angle = sample->gyro_z_angle;
  last_stamp = sample->stamp;
  return true;
}
