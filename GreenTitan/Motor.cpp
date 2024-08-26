#include "Motor.h"

// create the stepper motor objects
ContinuousStepper<StepperDriver> motorA;
ContinuousStepper<StepperDriver> motorB;

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

  if (!MOTOR_LEFT_INVERT) {
    motorA.spin(forward ? speedA : -speedA);
  } else {
    motorA.spin(forward ? -speedA : speedA);
  }

  if (!MOTOR_RIGHT_INVERT) {
    motorB.spin(forward ? speedB : -speedB);
  } else {
    motorB.spin(forward ? -speedB : speedB);
  }
}

void MotorRotate(float speedFactor) {
  // SpeedFactor
  speedFactor = constrain(speedFactor, -1.0, 1.0);

  int invertA = 1;
  int invertB = 1;

  // Rotate
  if (MOTOR_SIDE_INVERT) {
    speedFactor = -speedFactor;
  }

  if (MOTOR_LEFT_INVERT) {
    invertA = -invertA;
  }
  if (MOTOR_RIGHT_INVERT) {
    invertB = -invertB;
  }
  
  motorA.spin(-invertA * MOTOR_MAX_SPEED * speedFactor);
  motorB.spin(invertB * MOTOR_MAX_SPEED * speedFactor);
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

  motorASpeed += MowerAngularDegreesToMotorSteps(acceleration);
  if (motorASpeed > MOTOR_MAX_SPEED / 2) motorASpeed = MOTOR_MAX_SPEED / 2;
  if (motorASpeed < -MOTOR_MAX_SPEED / 2) motorASpeed = -MOTOR_MAX_SPEED / 2;

  if (motorASpeed > 0){
    motorA.spin(invertA * motorASpeed);
    motorB.spin(-invertB * motorASpeed);
  }
  else if (motorASpeed < 0) {
    motorA.spin(-invertA * motorASpeed);
    motorB.spin(invertB * motorASpeed);
  }
  else {
    motorA.stop();
    motorB.stop();
  }

  Serial.print(motorA.speed()); Serial.print(" "); Serial.println(motorB.speed());

  //Serial.println(motorASpeed);
  //Serial.println(motorA.getCurrentVelocityInStepsPerSecond());
}

void InitMotors() {
  pinMode(MOTOR_MAIN, OUTPUT);

  motorA.begin(MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
  motorB.begin(MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN);

  // Set acceleration
  motorA.setAcceleration(MOTOR_ACCELERATION);
  motorB.setAcceleration(MOTOR_ACCELERATION);
}

void MotorStop() {
  motorA.stop();
  motorB.stop();
}

void MotorMainOn() {
  digitalWrite(MOTOR_MAIN, HIGH);
}

void MotorMainOff() {
  digitalWrite(MOTOR_MAIN, LOW);
}

void MotorTask(void* pvParameters){
  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / MOTOR_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    motorA.loop();
    motorB.loop();

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}