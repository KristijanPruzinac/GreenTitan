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
  portENTER_CRITICAL(&imu_mux);
  bool valid = imu_valid;
  *sample = imu_sample;
  portEXIT_CRITICAL(&imu_mux);
  return valid;
}
