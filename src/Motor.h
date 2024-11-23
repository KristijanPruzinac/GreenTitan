//FUNCTIONAL

#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Defines.h"

//#include <ESP_FlexyStepper.h>
#include <ContinuousStepper.h>

extern bool MOTOR_SIDE_INVERT;
extern bool MOTOR_LEFT_INVERT;
extern bool MOTOR_RIGHT_INVERT;
extern float MOTOR_OPTIMAL_VOLTAGE;

float MowerAngularDegreesToMotorSteps(float unit);
void MotorDriveAngle(float angle, bool forward, float speedFactor);
void MotorRotate(float speedFactor);
void MotorRotateAcceleration(float acceleration);
bool InitMotors();
void MotorStop();
void MotorMainOn();
void MotorMainOff();
void MotorTask(void* pvParameters);

#endif