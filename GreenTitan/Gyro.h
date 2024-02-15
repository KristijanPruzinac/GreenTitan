//TODO: Add yaw

#ifndef GYRO_H
#define GYRO_H

#include "Arduino.h"
#include "Defines.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

bool InitGyro();
void GyroRead();

#endif