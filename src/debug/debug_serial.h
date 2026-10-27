// Debug console on a hardware UART. With DEBUG_OVER_ROS the output is also mirrored into a
// buffer that main.cpp publishes on esp/log, and input can be injected from esp/console, so a
// board without a second USB-UART adapter can still be driven from the PC.

#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <HardwareSerial.h>

class DebugSerial : public HardwareSerial {
public:
  explicit DebugSerial(int uart_nr) : HardwareSerial(uart_nr) {}

  using HardwareSerial::write;
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buffer, size_t size) override;
  int available() override;
  int read() override;
  int peek() override;

  size_t drain_output(char* out, size_t max);  // mirrored output since the last call
  void inject_input(const char* data, size_t len);
};

extern DebugSerial SerialDebug;

#endif
