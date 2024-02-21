#include "Motor.h"

void MotorDriveAngle(float angle, bool forward, float speedFactor = 1){
  //SpeedFactor
  if (speedFactor < 0) speedFactor = 0;
  if (speedFactor > 1) speedFactor = 1;

  //Angle measured from North clockwise
  if (angle < -90) angle = -90;
  if (angle > 90) angle = 90;

  float motorPercentLeft = ((angle - (-90.0)) / 180.0) * 2.0; if (motorPercentLeft > 1.0) motorPercentLeft = 1.0;
  float motorPercentRight = (1.0 - (angle - (-90.0)) / 180.0) * 2.0; if (motorPercentRight > 1.0) motorPercentRight = 1.0;

  float fullSpeedVal = (MOTOR_OPTIMAL_VOLTAGE / BatteryCurrentVoltage()); if (fullSpeedVal > 1) fullSpeedVal = 1; fullSpeedVal *= 4095;
  
  fullSpeedVal *= speedFactor; //Slow down if asked

  if (forward){
    if (!MOTOR_SIDE_INVERT){
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_LEFT_A, motorPercentLeft * fullSpeedVal);
        digitalWrite(MOTOR_LEFT_B, 0);
      }
      else {
        digitalWrite(MOTOR_LEFT_A, 0);
        analogWrite(MOTOR_LEFT_B, motorPercentLeft * fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_RIGHT_A, motorPercentRight * fullSpeedVal);
        digitalWrite(MOTOR_RIGHT_B, 0);
      }
      else {
        digitalWrite(MOTOR_RIGHT_A, 0);
        analogWrite(MOTOR_RIGHT_B, motorPercentRight * fullSpeedVal);
      }
    }
    else {
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_LEFT_A, motorPercentRight * fullSpeedVal);
        digitalWrite(MOTOR_LEFT_B, 0);
      }
      else {
        digitalWrite(MOTOR_LEFT_A, 0);
        analogWrite(MOTOR_LEFT_B, motorPercentRight * fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_RIGHT_A, motorPercentLeft * fullSpeedVal);
        digitalWrite(MOTOR_RIGHT_B, 0);
      }
      else {
        digitalWrite(MOTOR_RIGHT_A, 0);
        analogWrite(MOTOR_RIGHT_B, motorPercentLeft * fullSpeedVal);
      }
    }
  }
  else {
    if (!MOTOR_SIDE_INVERT){
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_LEFT_B, motorPercentLeft * fullSpeedVal);
        digitalWrite(MOTOR_LEFT_A, 0);
      }
      else {
        digitalWrite(MOTOR_LEFT_B, 0);
        analogWrite(MOTOR_LEFT_A, motorPercentLeft * fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_RIGHT_B, motorPercentRight * fullSpeedVal);
        digitalWrite(MOTOR_RIGHT_A, 0);
      }
      else {
        digitalWrite(MOTOR_RIGHT_B, 0);
        analogWrite(MOTOR_RIGHT_A, motorPercentRight * fullSpeedVal);
      }
    }
    else {
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_LEFT_B, motorPercentRight * fullSpeedVal);
        digitalWrite(MOTOR_LEFT_A, 0);
      }
      else {
        digitalWrite(MOTOR_LEFT_B, 0);
        analogWrite(MOTOR_LEFT_A, motorPercentRight * fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_RIGHT_B, motorPercentLeft * fullSpeedVal);
        digitalWrite(MOTOR_RIGHT_A, 0);
      }
      else {
        digitalWrite(MOTOR_RIGHT_B, 0);
        analogWrite(MOTOR_RIGHT_A, motorPercentLeft * fullSpeedVal);
      }
    }
  }
}

void MotorRotate(bool direction, float speedFactor = 1){
  //SpeedFactor
  if (speedFactor < 0) speedFactor = 0;
  if (speedFactor > 1) speedFactor = 1;

  float fullSpeedVal = (MOTOR_OPTIMAL_VOLTAGE / BatteryCurrentVoltage()); if (fullSpeedVal > 1) fullSpeedVal = 1; fullSpeedVal *= 4095;
  
  fullSpeedVal *= speedFactor; //Slow down if asked

  if (direction == LEFT){
    if (!MOTOR_SIDE_INVERT){
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_LEFT_B, fullSpeedVal);
        digitalWrite(MOTOR_LEFT_A, 0);
      }
      else {
        digitalWrite(MOTOR_LEFT_B, 0);
        analogWrite(MOTOR_LEFT_A, fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_RIGHT_A, fullSpeedVal);
        digitalWrite(MOTOR_RIGHT_B, 0);
      }
      else {
        digitalWrite(MOTOR_RIGHT_A, 0);
        analogWrite(MOTOR_RIGHT_B, fullSpeedVal);
      }
    }
    else {
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_LEFT_A, fullSpeedVal);
        digitalWrite(MOTOR_LEFT_B, 0);
      }
      else {
        digitalWrite(MOTOR_LEFT_A, 0);
        analogWrite(MOTOR_LEFT_B, fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_RIGHT_B, fullSpeedVal);
        digitalWrite(MOTOR_RIGHT_A, 0);
      }
      else {
        digitalWrite(MOTOR_RIGHT_B, 0);
        analogWrite(MOTOR_RIGHT_A, fullSpeedVal);
      }
    }
  }
  else {
    if (!MOTOR_SIDE_INVERT){
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_LEFT_A, fullSpeedVal);
        digitalWrite(MOTOR_LEFT_B, 0);
      }
      else {
        digitalWrite(MOTOR_LEFT_A, 0);
        analogWrite(MOTOR_LEFT_B, fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_RIGHT_B, fullSpeedVal);
        digitalWrite(MOTOR_RIGHT_A, 0);
      }
      else {
        digitalWrite(MOTOR_RIGHT_B, 0);
        analogWrite(MOTOR_RIGHT_A, fullSpeedVal);
      }
    }
    else {
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_LEFT_B, fullSpeedVal);
        digitalWrite(MOTOR_LEFT_A, 0);
      }
      else {
        digitalWrite(MOTOR_LEFT_B, 0);
        analogWrite(MOTOR_LEFT_A, fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_RIGHT_A, fullSpeedVal);
        digitalWrite(MOTOR_RIGHT_B, 0);
      }
      else {
        digitalWrite(MOTOR_RIGHT_A, 0);
        analogWrite(MOTOR_RIGHT_B, fullSpeedVal);
      }
    }
  }
}

void InitMotors(){
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  pinMode(MOTOR_MAIN, OUTPUT);

  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_MAIN, LOW);
}

void MotorStop(){
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void MotorMainOn(){
  digitalWrite(MOTOR_MAIN, HIGH);
}
void MotorMainOff(){
  digitalWrite(MOTOR_MAIN, LOW);
}