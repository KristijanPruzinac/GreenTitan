#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

extern SemaphoreHandle_t AzimuthMutex;

extern float MagOffsetAngle;
extern float MagDeclinationAngle;
extern bool InvertCompassAzimuth;

bool InitIMU();
void IMURead();

float IMUGetAzimuth();

/*
float IMUGetAccX();
float IMUGetAccY();
float IMUGetAccZ();

float IMUGetGyroX();
float IMUGetGyroY();
float IMUGetGyroZ();

float IMUGetTemp();
*/

#endif