#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"

#include <ContinuousStepper.h>

#include "../debug/debug_serial.h"

extern bool ENABLE_MOTORS;

bool init_motors();
void motor_get_step_rates(float* left, float* right);  // current steps/s (HIL output)

void motor_task(void* parameter);

#endif