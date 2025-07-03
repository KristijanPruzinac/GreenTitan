#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"
#include "SensorInterfaceTask.h"

#include <vector>
#include <math.h>

extern int MAX_DEVIATION;

extern SemaphoreHandle_t MotionMutex;

//Motor
extern float MowerAngularDegreesToMotorSteps(float unit);
extern void MotorSetAcceleration(float acceleration);
extern void MotorResetAcceleration();
extern void MotorDriveAngle(float angle, bool forward, float speedFactor);
extern void MotorRotate(float speedFactor);
extern void MotorStop();

extern void MotorRotateAcceleration(float acceleration);


void MotionSetMode(int mode, int MoveAfterRotation = 0);
void MotionSetTargetPoint(long tLon, long tLat);
void MotionSetTargetPointRotation(float azimuthDegrees);
void MotionMoveToTarget();
void MotionRotateToTarget();
void MotionTask(void* pvParameters);

bool MowerIsInMotion();

extern float MOTION_ACC_FACTOR;

//TODO: Remove
extern void BluetoothWrite(String message);

#endif