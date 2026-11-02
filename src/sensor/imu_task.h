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

#include "../debug/debug_serial.h"
extern volatile bool TASKS_STARTED;

extern bool IMU_INVERT;

extern double odom_linear_velocity;
extern double odom_angular_velocity;

bool init_imu();
void imu_calibrate();
bool imu_ready();   // gyro bias calibrated, /imu is being published
bool imu_yaw_now(double* yaw);            // integrated gyro yaw now (rad, unwrapped, relative); false before calibration
void imu_set_ground_speed(float speed);   // GPS ground speed (m/s), for the standstill gyro bias

void imu_task(void* parameter);

#endif