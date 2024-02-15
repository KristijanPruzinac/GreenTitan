//FUNCTIONAL

#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Defines.h"

#include "Battery.h"

extern bool MOTOR_SIDE_INVERT;
extern bool MOTOR_LEFT_INVERT;
extern bool MOTOR_RIGHT_INVERT;
extern float MOTOR_OPTIMAL_VOLTAGE;

void MotorDriveAngle(float angle, bool forward, float speedFactor);
void MotorRotate(bool direction, float speedFactor);
void InitMotors();
void MotorStop();
void MotorMainOn();
void MotorMainOff();

#endif