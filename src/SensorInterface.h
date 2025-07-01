#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <Arduino.h>
#include "Defines.h"
#include "Functions.h"

extern SemaphoreHandle_t IMUMutex;
extern SemaphoreHandle_t GPSMutex;
extern SemaphoreHandle_t MotionMutex;

extern float IMUGetGyroZ();
extern float IMURotSpeed;
extern float IMURotAcc;
extern float IMUHeadingChange;
float IMUGetRotationSpeedRaw();
float IMUGetRotationSpeed();
float IMUGetRotationAcceleration();

float GetHeading();

//DEBUG
void PlotIMUData();

#endif