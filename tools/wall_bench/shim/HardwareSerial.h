// Wall bench: Print / HardwareSerial. Everything written ends up in the bench log (bench.cpp).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <string>

#define SERIAL_8N1 0

class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* buffer, size_t size) {
    for (size_t i = 0; i < size; i++) write(buffer[i]);
    return size;
  }
  size_t write(const char* s) { return write((const uint8_t*)s, strlen(s)); }
  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return 0;
    return write((const uint8_t*)buf, (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1);
  }
  size_t print(const char* s) { return write(s); }
  size_t print(const std::string& s) { return write(s.c_str()); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(long v) { return printf("%ld", v); }
  size_t print(int v) { return printf("%d", v); }
  size_t print(unsigned long v) { return printf("%lu", v); }
  size_t print(unsigned v) { return printf("%u", v); }
  size_t print(double v, int digits = 2) { return printf("%.*f", digits, v); }
  size_t println() { return write("\r\n"); }
  template <typename T> size_t println(const T& v) { size_t n = print(v); return n + println(); }
  size_t println(double v, int digits) { size_t n = print(v, digits); return n + println(); }
};

class Stream : public Print {
public:
  virtual int available() { return 0; }
  virtual int read() { return -1; }
  virtual int peek() { return -1; }
  void setTimeout(unsigned long) {}
};

class HardwareSerial : public Stream {
public:
  explicit HardwareSerial(int uart_nr) : uart_nr_(uart_nr) {}
  void begin(unsigned long, uint32_t = SERIAL_8N1, int8_t = -1, int8_t = -1, bool = false, unsigned long = 20000UL) {}
  void end() {}
  void flush() {}
  void setRxBufferSize(size_t) {}
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buffer, size_t size) override;
  using Print::write;
  operator bool() const { return true; }
private:
  int uart_nr_;
};
