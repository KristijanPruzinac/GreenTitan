//TODO: Add yaw

#ifndef GYRO_H
#define GYRO_H

#include "Arduino.h"
#include "Defines.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

sensors_event_t g_acc, g_gyro, g_temp;

bool InitGyro();
void GyroRead();

#endif