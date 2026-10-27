#include "debug_serial.h"

#include "freertos/FreeRTOS.h"
#include "../definitions.h"

namespace {

template <size_t N>
struct Ring {
  uint8_t buf[N];
  size_t head = 0, tail = 0;
  bool push(uint8_t c) {
    size_t next = (head + 1) % N;
    if (next == tail) return false;
    buf[head] = c;
    head = next;
    return true;
  }
  int pop() {
    if (head == tail) return -1;
    uint8_t c = buf[tail];
    tail = (tail + 1) % N;
    return c;
  }
  int front() const { return head == tail ? -1 : buf[tail]; }
  size_t size() const { return (head + N - tail) % N; }
};

Ring<2048> out_ring;
Ring<128> in_ring;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

}  // namespace

size_t DebugSerial::write(uint8_t c) { return write(&c, 1); }

size_t DebugSerial::write(const uint8_t* buffer, size_t size) {
  if (DEBUG_OVER_ROS) {
    portENTER_CRITICAL(&mux);
    for (size_t i = 0; i < size && out_ring.push(buffer[i]); i++) {}  // drop when full
    portEXIT_CRITICAL(&mux);
  }
  return HardwareSerial::write(buffer, size);
}

int DebugSerial::available() {
  portENTER_CRITICAL(&mux);
  int injected = (int)in_ring.size();
  portEXIT_CRITICAL(&mux);
  return injected + HardwareSerial::available();
}

int DebugSerial::read() {
  portENTER_CRITICAL(&mux);
  int c = in_ring.pop();
  portEXIT_CRITICAL(&mux);
  return c >= 0 ? c : HardwareSerial::read();
}

int DebugSerial::peek() {
  portENTER_CRITICAL(&mux);
  int c = in_ring.front();
  portEXIT_CRITICAL(&mux);
  return c >= 0 ? c : HardwareSerial::peek();
}

size_t DebugSerial::drain_output(char* out, size_t max) {
  size_t n = 0;
  portENTER_CRITICAL(&mux);
  int c;
  while (n < max && (c = out_ring.pop()) >= 0) out[n++] = (char)c;
  portEXIT_CRITICAL(&mux);
  return n;
}

void DebugSerial::inject_input(const char* data, size_t len) {
  portENTER_CRITICAL(&mux);
  for (size_t i = 0; i < len && in_ring.push((uint8_t)data[i]); i++) {}
  portEXIT_CRITICAL(&mux);
}
