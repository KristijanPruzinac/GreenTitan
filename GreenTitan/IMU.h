#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

bool InitIMU();
void IMURead();

float IMUGetAzimuth();

float IMUGetAccX();
float IMUGetAccY();
float IMUGetAccZ();

float IMUGetGyroX();
float IMUGetGyroY();
float IMUGetGyroZ();

float IMUGetTemp();

#endif