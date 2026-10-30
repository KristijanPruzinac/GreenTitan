// Wall bench: the few Arduino-ESP32 pieces the ToF task and the wall assist use, on the bench's
// simulated clock (bench.cpp). Not a general Arduino emulation.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "HardwareSerial.h"

#define PI 3.1415926535897932384626433832795
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

using std::min;
using std::max;

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
uint32_t esp_random();

class String : public std::string {
public:
  String() {}
  String(const char* s) : std::string(s ? s : "") {}
  String(const std::string& s) : std::string(s) {}
  String(int v) : std::string(std::to_string(v)) {}
  String(long v) : std::string(std::to_string(v)) {}
  String(unsigned v) : std::string(std::to_string(v)) {}
  String(unsigned long v) : std::string(std::to_string(v)) {}
  String(double v, int digits = 2) { char b[48]; snprintf(b, sizeof(b), "%.*f", digits, v); assign(b); }
  int toInt() const { return atoi(c_str()); }
  float toFloat() const { return (float)atof(c_str()); }
};

extern HardwareSerial Serial;
