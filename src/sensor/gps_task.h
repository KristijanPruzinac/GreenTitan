//FUNCTIONAL

#ifndef GPS_TASK_H
#define GPS_TASK_H

#include "Arduino.h"
#include "../Definitions.h"
#include "../Functions.h"

#include <math.h>
#include <HardwareSerial.h>

#include "esp_dds.h"
#include "esp_timer.h"

bool init_gps();
void gps_task(void* parameter);

#endif