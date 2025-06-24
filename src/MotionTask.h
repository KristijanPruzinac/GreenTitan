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
extern SemaphoreHandle_t MotionMutex;

//Gps
extern long GpsGetLon();
extern long GpsGetLat();
extern float GPS_Heading;
extern float GPS_PrevHeading;

//Motor
extern float MowerAngularDegreesToMotorSteps(float unit);
extern void MotorSetAcceleration(float acceleration);
extern void MotorResetAcceleration();
extern void MotorDriveAngle(float angle, bool forward, float speedFactor);
extern void MotorRotate(float speedFactor);
extern void MotorStop();

extern void MotorRotateAcceleration(float acceleration);


void MotionSetMode(int mode);
void MotionSetTarget(long tLon, long tLat);
void MotionSetTargetRotation(float azimuthDegrees);
void MotionMoveToTarget();
void MotionRotateToTarget();
void MotionTask(void* pvParameters);

bool MowerIsInMotion();

extern bool ENABLE_GPS;

extern float IMURotSpeed;
extern float IMURotAcc;
extern float IMUHeading;
extern float MOTION_ACC_FACTOR;

//TODO: Remove
extern void BluetoothWrite(String message);

#endif