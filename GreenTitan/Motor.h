#define LEFT 0
#define RIGHT 1

bool MOTOR_SIDE_INVERT = true;
bool MOTOR_LEFT_INVERT = true;
bool MOTOR_RIGHT_INVERT = true;

const float motorOptimalVoltage = 12;

void MotorDriveAngle(float angle, bool forward, float speedFactor = 1){
  //SpeedFactor
  if (speedFactor < 0) speedFactor = 0;
  if (speedFactor > 1) speedFactor = 1;

  //Angle measured from North clockwise
  if (angle < -90) angle = -90;
  if (angle > 90) angle = 90;

  float motorPercentLeft = ((angle - (-90.0)) / 180.0) * 2.0; if (motorPercentLeft > 1.0) motorPercentLeft = 1.0;
  float motorPercentRight = (1.0 - (angle - (-90.0)) / 180.0) * 2.0; if (motorPercentRight > 1.0) motorPercentRight = 1.0;

  float fullSpeedVal = (motorOptimalVoltage / batteryCurrentLevel); if (fullSpeedVal > 1) fullSpeedVal = 1; fullSpeedVal *= 255; //TODO: Change to 4095
  
  fullSpeedVal *= speedFactor; //Slow down if asked

  if (forward){
    if (!MOTOR_SIDE_INVERT){
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_left_A, motorPercentLeft * fullSpeedVal);
        digitalWrite(MOTOR_left_B, 0);
      }
      else {
        digitalWrite(MOTOR_left_A, 0);
        analogWrite(MOTOR_left_B, motorPercentLeft * fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_right_A, motorPercentRight * fullSpeedVal);
        digitalWrite(MOTOR_right_B, 0);
      }
      else {
        digitalWrite(MOTOR_right_A, 0);
        analogWrite(MOTOR_right_B, motorPercentRight * fullSpeedVal);
      }
    }
    else {
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_left_A, motorPercentRight * fullSpeedVal);
        digitalWrite(MOTOR_left_B, 0);
      }
      else {
        digitalWrite(MOTOR_left_A, 0);
        analogWrite(MOTOR_left_B, motorPercentRight * fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_right_A, motorPercentLeft * fullSpeedVal);
        digitalWrite(MOTOR_right_B, 0);
      }
      else {
        digitalWrite(MOTOR_right_A, 0);
        analogWrite(MOTOR_right_B, motorPercentLeft * fullSpeedVal);
      }
    }
  }
  else {
    if (!MOTOR_SIDE_INVERT){
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_left_B, motorPercentLeft * fullSpeedVal);
        digitalWrite(MOTOR_left_A, 0);
      }
      else {
        digitalWrite(MOTOR_left_B, 0);
        analogWrite(MOTOR_left_A, motorPercentLeft * fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_right_B, motorPercentRight * fullSpeedVal);
        digitalWrite(MOTOR_right_A, 0);
      }
      else {
        digitalWrite(MOTOR_right_B, 0);
        analogWrite(MOTOR_right_A, motorPercentRight * fullSpeedVal);
      }
    }
    else {
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_left_B, motorPercentRight * fullSpeedVal);
        digitalWrite(MOTOR_left_A, 0);
      }
      else {
        digitalWrite(MOTOR_left_B, 0);
        analogWrite(MOTOR_left_A, motorPercentRight * fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_right_B, motorPercentLeft * fullSpeedVal);
        digitalWrite(MOTOR_right_A, 0);
      }
      else {
        digitalWrite(MOTOR_right_B, 0);
        analogWrite(MOTOR_right_A, motorPercentLeft * fullSpeedVal);
      }
    }
  }
}

void MotorRotate(bool direction, float speedFactor = 1){
  //SpeedFactor
  if (speedFactor < 0) speedFactor = 0;
  if (speedFactor > 1) speedFactor = 1;

  float fullSpeedVal = (motorOptimalVoltage / batteryCurrentLevel); if (fullSpeedVal > 1) fullSpeedVal = 1; fullSpeedVal *= 255; //TODO: Change to 4095
  
  fullSpeedVal *= speedFactor; //Slow down if asked

  if (direction == LEFT){
    if (!MOTOR_SIDE_INVERT){
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_left_B, fullSpeedVal);
        digitalWrite(MOTOR_left_A, 0);
      }
      else {
        digitalWrite(MOTOR_left_B, 0);
        analogWrite(MOTOR_left_A, fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_right_A, fullSpeedVal);
        digitalWrite(MOTOR_right_B, 0);
      }
      else {
        digitalWrite(MOTOR_right_A, 0);
        analogWrite(MOTOR_right_B, fullSpeedVal);
      }
    }
    else {
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_left_A, fullSpeedVal);
        digitalWrite(MOTOR_left_B, 0);
      }
      else {
        digitalWrite(MOTOR_left_A, 0);
        analogWrite(MOTOR_left_B, fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_right_B, fullSpeedVal);
        digitalWrite(MOTOR_right_A, 0);
      }
      else {
        digitalWrite(MOTOR_right_B, 0);
        analogWrite(MOTOR_right_A, fullSpeedVal);
      }
    }
  }
  else {
    if (!MOTOR_SIDE_INVERT){
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_left_A, fullSpeedVal);
        digitalWrite(MOTOR_left_B, 0);
      }
      else {
        digitalWrite(MOTOR_left_A, 0);
        analogWrite(MOTOR_left_B, fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_right_B, fullSpeedVal);
        digitalWrite(MOTOR_right_A, 0);
      }
      else {
        digitalWrite(MOTOR_right_B, 0);
        analogWrite(MOTOR_right_A, fullSpeedVal);
      }
    }
    else {
      if (!MOTOR_LEFT_INVERT){
        analogWrite(MOTOR_left_B, fullSpeedVal);
        digitalWrite(MOTOR_left_A, 0);
      }
      else {
        digitalWrite(MOTOR_left_B, 0);
        analogWrite(MOTOR_left_A, fullSpeedVal);
      }

      if (!MOTOR_RIGHT_INVERT){
        analogWrite(MOTOR_right_A, fullSpeedVal);
        digitalWrite(MOTOR_right_B, 0);
      }
      else {
        digitalWrite(MOTOR_right_A, 0);
        analogWrite(MOTOR_right_B, fullSpeedVal);
      }
    }
  }
}

void InitMotors(){
  pinMode(MOTOR_left_A, OUTPUT);
  pinMode(MOTOR_left_B, OUTPUT);
  pinMode(MOTOR_right_A, OUTPUT);
  pinMode(MOTOR_right_B, OUTPUT);
  pinMode(MOTOR_main, OUTPUT);

  digitalWrite(MOTOR_left_A, LOW);
  digitalWrite(MOTOR_left_B, LOW);
  digitalWrite(MOTOR_right_A, LOW);
  digitalWrite(MOTOR_right_B, LOW);
  digitalWrite(MOTOR_main, LOW);
}

void MotorStop(){
  digitalWrite(MOTOR_left_A, LOW);
  digitalWrite(MOTOR_left_B, LOW);

  digitalWrite(MOTOR_right_A, LOW);
  digitalWrite(MOTOR_right_B, LOW);
}

void MotorMainOn(){
  digitalWrite(MOTOR_main, HIGH);
}
void MotorMainOff(){
  digitalWrite(MOTOR_main, LOW);
}