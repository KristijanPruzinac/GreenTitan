#ifndef MOTION_H
#define MOTION_H

#include "Arduino.h"
#include "Defines.h"

void MotorStop();
void MotorMainOn();
void MotorMainOff();
void MotorGradualLeft(float amount);
void MotorTurnLeft();
void MotorGradualRight(float amount);
void MotorTurnRight();
void MotionMoveToTarget();
void MotionRotateToTarget();

#endif