#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"

extern HardwareSerial SerialDebug;

void motion_task(void* parameter);

#endif