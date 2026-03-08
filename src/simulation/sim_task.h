#ifndef SIM_TASK_H
#define SIM_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"

void sim_task(void* parameter);

#endif