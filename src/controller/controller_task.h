#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"

#include <vector>

extern bool CONFIG_DATUM;
extern bool CONFIG_PATH;

extern HardwareSerial SerialDebug;

void controller_task(void* parameter);

#endif