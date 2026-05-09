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

extern HardwareSerial SerialDebug;

extern bool ENABLE_GPS;

extern double odom_x;
extern double odom_y;

bool init_gps();
void gps_task(void* parameter);

#endif