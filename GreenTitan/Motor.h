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
/*
void MotorForward(){
  MotorStop();

  digitalWrite(MOTOR_left_A, LOW);
  digitalWrite(MOTOR_left_B, HIGH);

  digitalWrite(MOTOR_right_A, LOW);
  digitalWrite(MOTOR_right_B, HIGH);
}
void MotorBackward(){
  MotorStop();

  digitalWrite(MOTOR_left_A, HIGH);
  digitalWrite(MOTOR_left_B, LOW);

  digitalWrite(MOTOR_right_A, HIGH);
  digitalWrite(MOTOR_right_B, LOW);
}
void MotorTurnLeft(){
  MotorStop();

  digitalWrite(MOTOR_right_A, LOW);
  digitalWrite(MOTOR_right_B, HIGH);

  digitalWrite(MOTOR_left_A, HIGH);
  digitalWrite(MOTOR_left_B, LOW);
}
void MotorTurnRight(){
  MotorStop();

  digitalWrite(MOTOR_left_A, LOW);
  digitalWrite(MOTOR_left_B, HIGH);

  digitalWrite(MOTOR_right_A, HIGH);
  digitalWrite(MOTOR_right_B, LOW);
}
void MotorPivotLeft(){
  MotorStop();

  digitalWrite(MOTOR_right_A, LOW);
  digitalWrite(MOTOR_right_B, HIGH);
}
void MotorPivotRight(){
  MotorStop();

  digitalWrite(MOTOR_left_A, LOW);
  digitalWrite(MOTOR_left_B, HIGH);
}
*/