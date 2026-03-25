//FUNCTIONAL

#ifndef GPS_TASK_H
#define GPS_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include <math.h>
#include <HardwareSerial.h>

#include "esp_dds.h"
#include "esp_timer.h"

extern bool ENABLE_GPS;

extern float odom_x;
extern float odom_y;

bool init_gps();
void gps_task(void* parameter);

#endif