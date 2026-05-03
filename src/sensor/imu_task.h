#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <Arduino.h>
#include "../definitions.h"
#include "../functions.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#include "esp_dds.h"
#include "esp_timer.h"

extern HardwareSerial SerialDebug;

extern bool IMU_INVERT;

extern double odom_linear_velocity;
extern double odom_angular_velocity;

bool init_imu();
void imu_calibrate();

void imu_task(void* parameter);

#endif