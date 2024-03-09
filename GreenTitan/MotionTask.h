#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"

#include <PID_v1.h>
#include <vector>
#include <math.h>

extern int MAX_DEVIATION;

extern double PID_Kp;
extern double PID_Ki;
extern double PID_Kd;

extern SemaphoreHandle_t AzimuthMutex;
extern SemaphoreHandle_t PID_Mutex;

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

void InitializeMotionPID();
void MotionUpdatePIDParameters(double Kp, double Ki, double Kd);
void MotionSetMode(int mode);
void MotionUpdateAzimuth();
void MotionSetTarget(int tLon, int tLat);
void MotionMoveToTarget();
void MotionRotateToTarget();
void MotionTask(void* pvParameters);

//TODO: Remove
extern void BluetoothWrite(String message);

#endif