#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"

#include <vector>
#include <math.h>

extern int MAX_DEVIATION;

extern SemaphoreHandle_t AzimuthMutex;

//Main
extern void MainStop();

//Gps
extern int GpsGetLon();
extern int GpsGetLat();

//Motor
extern void MotorDriveAngle(float angle, bool forward, float speedFactor);
extern void MotorRotate(bool direction, float speedFactor);
extern void MotorStop();

//IMU
extern float IMUCurrentAzimuth;

void MotionUpdateAzimuth();
void MotionSetTarget(int tLon, int tLat);
void MotionMoveToTarget();
void MotionRotateToTarget();
void MotionTask(void* pvParameters);

//TODO: Remove
extern void BluetoothWrite(String message);

#endif