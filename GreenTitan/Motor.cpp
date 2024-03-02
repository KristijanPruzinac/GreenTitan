#include "Motor.h"

void MotorDriveAngle(float angle, bool forward, float speedFactor = 1){
  // SpeedFactor
  speedFactor = constrain(speedFactor, 0, 1);
  if (speedFactor < MOTOR_MIN_SPEED)
    speedFactor = MOTOR_MIN_SPEED;

  // Angle measured from North clockwise
  angle = constrain(angle, MOTOR_ANGLE_MIN, MOTOR_ANGLE_MAX);

  //Motor activation percentages
  float motorPercentLeft = ((angle - (-90.0)) / 180.0) * 2.0;
  motorPercentLeft = constrain(motorPercentLeft, 0, 1);

  float motorPercentRight = (1.0 - (angle - (-90.0)) / 180.0) * 2.0;
  motorPercentRight = constrain(motorPercentRight, 0, 1);

  //Calculate optimal voltage based on battery
  float fullSpeedVal = (MOTOR_OPTIMAL_VOLTAGE / BatteryCurrentVoltage());
  fullSpeedVal = constrain(fullSpeedVal, 0, 1);

  //Adjust desired speed
  fullSpeedVal *= speedFactor;

  //Scale to DAC
  fullSpeedVal *= DAC_MAX_VALUE; fullSpeedVal = constrain(fullSpeedVal, 200, DAC_MAX_VALUE);

  int leftVal = motorPercentLeft * fullSpeedVal;
  int rightVal = motorPercentRight * fullSpeedVal;

  bool leftInvert = MOTOR_LEFT_INVERT;
  bool rightInvert = MOTOR_RIGHT_INVERT;

  //Switch sides
  if (MOTOR_SIDE_INVERT){
    std::swap(leftVal, rightVal);
  }

  //Switch direction
  if (!forward){
    leftInvert = !leftInvert;
    rightInvert = !rightInvert;
  }

  // Left motor
  analogWrite((!leftInvert) ? MOTOR_LEFT_A : MOTOR_LEFT_B, leftVal);
  analogWrite((!leftInvert) ? MOTOR_LEFT_B : MOTOR_LEFT_A, 0);

  // Right motor
  analogWrite((!rightInvert) ? MOTOR_RIGHT_A : MOTOR_RIGHT_B, rightVal);
  analogWrite((!rightInvert) ? MOTOR_RIGHT_B : MOTOR_RIGHT_A, 0);
}

void MotorRotate(bool direction, float speedFactor = 1){
  //SpeedFactor
  speedFactor = constrain(speedFactor, 0, 1);
  if (speedFactor < MOTOR_MIN_SPEED)
    speedFactor = MOTOR_MIN_SPEED;

  //Calculate optimal voltage based on battery
  float fullSpeedVal = (MOTOR_OPTIMAL_VOLTAGE / BatteryCurrentVoltage());
  fullSpeedVal = constrain(fullSpeedVal, 0, 1);

  //Adjust desired speed
  fullSpeedVal *= speedFactor;

  //Scale to DAC
  fullSpeedVal *= DAC_MAX_VALUE; fullSpeedVal = constrain(fullSpeedVal, 200, DAC_MAX_VALUE);

  bool leftInvert = MOTOR_LEFT_INVERT;
  bool rightInvert = MOTOR_RIGHT_INVERT;

  //Set motors to drive in opposite directions to spin on spot
  if (direction == RIGHT){
    leftInvert = !leftInvert;
    rightInvert = !rightInvert;
  }

  //Switch sides
  if (MOTOR_SIDE_INVERT){
    direction = !direction;
  }

  //TODO: FIX SWITCHING WIRING / PRETTY SURE IT DOESNT WORK FOR GENERAL CASES

  //TODO: Remove
  //Serial.print(leftInvert); Serial.print(" "); Serial.print(rightInvert); Serial.print(" "); Serial.print(fullSpeedVal);
  //Serial.println();


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

  analogWriteResolution(DAC_MAX_BITS);
}

void MotorStop(){
  analogWrite(MOTOR_LEFT_A, LOW);
  analogWrite(MOTOR_LEFT_B, LOW);

  analogWrite(MOTOR_RIGHT_A, LOW);
  analogWrite(MOTOR_RIGHT_B, LOW);
}

void MotorMainOn(){
  digitalWrite(MOTOR_MAIN, HIGH);
}
void MotorMainOff(){
  digitalWrite(MOTOR_MAIN, LOW);
}