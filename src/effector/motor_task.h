#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"

#include <ContinuousStepper.h>

extern HardwareSerial SerialDebug;

extern bool ENABLE_MOTORS;

bool init_motors();

void motor_task(void* parameter);

#endif