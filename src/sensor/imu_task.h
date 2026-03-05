#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <Arduino.h>
#include "../Definitions.h"
#include "../Functions.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#include "esp_dds.h"
#include "esp_timer.h"

extern bool ENABLE_IMU;

extern bool IMU_INVERT;

bool init_imu();
void imu_calibrate();

void imu_task(void* parameter);

#endif