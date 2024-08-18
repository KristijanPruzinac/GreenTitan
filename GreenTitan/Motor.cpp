#include "Motor.h"

// create the stepper motor objects
ESP_FlexyStepper motorA;
ESP_FlexyStepper motorB;

float MowerAngularDegreesToMotorSteps(float unit){
  //     PATH AROUND CIRCUMFERENCE TRAVELLED (CM)                     STEPS PER CM                2 MOTORS
  return ((unit / (360.0)) * (MOTOR_DIAMETER * PI)) * (MOTOR_STEPS_PER_REV / (WHEEL_DIAMETER * PI)) / 2;
}

void MotorDriveAngle(float angle, bool forward, float speedFactor = 1) {
  // SpeedFactor
  speedFactor = constrain(speedFactor, 0.0, 1.0);

  // Angle measured from North clockwise
  angle = constrain(angle, MOTOR_ANGLE_MIN, MOTOR_ANGLE_MAX);

  // Motor activation percentages
  float motorPercentLeft = (angle + 90.0) / 180.0; motorPercentLeft = constrain(motorPercentLeft, 0, 1);
  float motorPercentRight = 1 - (motorPercentLeft); motorPercentRight = constrain(motorPercentRight, 0, 1);

  // Switch sides
  if (MOTOR_SIDE_INVERT) {
    std::swap(motorPercentLeft, motorPercentRight);
  }

  // Calculate speeds for motors
  float speedA = MOTOR_MAX_SPEED * speedFactor * motorPercentLeft;
  float speedB = MOTOR_MAX_SPEED * speedFactor * motorPercentRight;

  motorA.setSpeedInStepsPerSecond(speedA);
  motorB.setSpeedInStepsPerSecond(speedB);

  if (!MOTOR_LEFT_INVERT) {
    motorA.setTargetPositionInSteps(forward ? 1e9 : -1e9);
  } else {
    motorA.setTargetPositionInSteps(forward ? -1e9 : 1e9);
  }

  if (!MOTOR_RIGHT_INVERT) {
    motorB.setTargetPositionInSteps(forward ? 1e9 : -1e9);
  } else {
    motorB.setTargetPositionInSteps(forward ? -1e9 : 1e9);
  }
}

void MotorRotate(bool direction, float speedFactor = 1) {
  // SpeedFactor
  speedFactor = constrain(speedFactor, 0.0, 1.0);

  int invertA = 1;
  int invertB = 1;

  // Rotate
  if (MOTOR_SIDE_INVERT) {
    direction = !direction;
  }

  if (MOTOR_LEFT_INVERT) {
    invertA = -invertA;
  }
  if (MOTOR_RIGHT_INVERT) {
    invertB = -invertB;
  }

  float speed = MOTOR_MAX_SPEED * 0.5 * speedFactor;

  motorA.setSpeedInStepsPerSecond(speed);
  motorB.setSpeedInStepsPerSecond(speed);

  if (direction == LEFT) {
    motorA.setTargetPositionInSteps(-invertA * 1e9);
    motorB.setTargetPositionInSteps(invertB * 1e9);
  } else {
    motorA.setTargetPositionInSteps(invertA * 1e9);
    motorB.setTargetPositionInSteps(-invertB * 1e9);
  }
}

float motorASpeed = 0;
void MotorRotateAcceleration(float acceleration){
  int invertA = 1;
  int invertB = 1;

  // Rotate
  /*
  if (MOTOR_SIDE_INVERT) {
    direction = !direction;
  }
  */

  if (MOTOR_LEFT_INVERT) {
    invertA = -invertA;
  }
  if (MOTOR_RIGHT_INVERT) {
    invertB = -invertB;
  }

  float oldMotorASpeed = motorASpeed;

  motorASpeed += MowerAngularDegreesToMotorSteps(acceleration);
  if (motorASpeed > MOTOR_MAX_SPEED / 2) motorASpeed = MOTOR_MAX_SPEED / 2;
  if (motorASpeed < -MOTOR_MAX_SPEED / 2) motorASpeed = -MOTOR_MAX_SPEED / 2;

  if (fabs(motorASpeed - oldMotorASpeed) > 0.1){
    motorA.setSpeedInStepsPerSecond(motorASpeed);
    motorB.setSpeedInStepsPerSecond(motorASpeed);

    if (motorASpeed > 0){
      motorA.startJogging(invertA);
      motorB.startJogging(-invertB);
    }
    else if (motorASpeed < 0) {
      motorA.startJogging(-invertA);
      motorB.startJogging(invertB);
    }
  }

  Serial.println(motorA.getCurrentVelocityInStepsPerSecond());

  //Serial.println(motorASpeed);
  //Serial.println(motorA.getCurrentVelocityInStepsPerSecond());
}

void InitMotors() {
  pinMode(MOTOR_MAIN, OUTPUT);

  // Initialize motors
  motorA.connectToPins(MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
  motorB.connectToPins(MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN);

  // Set acceleration
  motorA.setAccelerationInStepsPerSecondPerSecond(MOTOR_ACCELERATION);
  motorB.setAccelerationInStepsPerSecondPerSecond(MOTOR_ACCELERATION);

  motorA.setDecelerationInStepsPerSecondPerSecond(MOTOR_ACCELERATION);
  motorB.setDecelerationInStepsPerSecondPerSecond(MOTOR_ACCELERATION);

  motorA.setStepsPerMillimeter(10);
  motorB.setStepsPerMillimeter(10);

  // Initial speed
  motorA.setTargetPositionToStop();
  motorB.setTargetPositionToStop();

  motorA.startAsService(1);
  motorB.startAsService(1);
}

void MotorStop() {
  motorA.setCurrentPositionInSteps(0);
  motorB.setCurrentPositionInSteps(0);

  motorA.setTargetPositionInSteps(0);
  motorB.setTargetPositionInSteps(0);

  motorA.setTargetPositionToStop();
  motorB.setTargetPositionToStop();
}

void MotorMainOn() {
  digitalWrite(MOTOR_MAIN, HIGH);
}

void MotorMainOff() {
  digitalWrite(MOTOR_MAIN, LOW);
}