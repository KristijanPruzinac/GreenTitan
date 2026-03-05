#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "Arduino.h"
#include "../Definitions.h"

#include "esp_dds.h"
#include "esp_timer.h"

#include <ContinuousStepper.h>

bool init_motors();

void motor_task(void* parameter);

#endif