#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"

#include <vector>

#include "../algorithm/algorithm.h"
#include "../configuration/configuration.h"

extern bool CONFIG_DATUM;
extern bool CONFIG_PATH;
extern int GPS_ACC_THRESHOLD;

#include "../debug/debug_serial.h"
extern volatile bool TASKS_STARTED;

void controller_task(void* parameter);
// False while a mission runs: the planner's path vectors must not change under it.
bool controller_path_edit_allowed();
// Console B: power, dock contact and station pose
void controller_print_dock_status();

#endif