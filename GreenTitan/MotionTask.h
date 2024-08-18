#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"

#include <vector>
#include <math.h>

extern int MAX_DEVIATION;

extern SemaphoreHandle_t IMUMutex;
extern SemaphoreHandle_t GPSMutex;

//Main
extern void MainStop();

//Gps
extern int GpsGetLon();
extern int GpsGetLat();
extern float GPS_Heading;
extern float GPS_PrevHeading;

//Motor
extern float MowerAngularDegreesToMotorSteps(float unit);
extern void MotorSetAcceleration(float acceleration);
extern void MotorResetAcceleration();
extern void MotorDriveAngle(float angle, bool forward, float speedFactor);
extern void MotorRotate(bool direction, float speedFactor, float degrees);
extern void MotorStop();

extern void MotorRotateAcceleration(float acceleration);

//IMU
void MotionSetMode(int mode);
void MotionSetTarget(int tLon, int tLat);
void MotionMoveToTarget();
void MotionRotateToTarget();
void MotionTask(void* pvParameters);

extern float IMURotSpeed;
extern float IMURotAcc;
extern float IMUHeading;
extern float MOTION_ACC_FACTOR;

//TODO: Remove
extern void BluetoothWrite(String message);

#endif