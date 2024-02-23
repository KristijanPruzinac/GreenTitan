boolean MOTOR_SIDE_INVERT = false;
boolean MOTOR_LEFT_INVERT = false;
boolean MOTOR_RIGHT_INVERT = false;
boolean forward = true;

int leftMotor = 0;
int rightMotor = 0;

int leftVal = 20;
int rightVal = 20;

boolean leftInvert = MOTOR_LEFT_INVERT;
boolean rightInvert = MOTOR_RIGHT_INVERT;

void setup(){
  size(500, 500);
  
  //Switch sides
  if (MOTOR_SIDE_INVERT){
    int tmp = leftVal;
    leftVal = rightVal;
    rightVal = tmp;
  }
  
  //Switch direction
  if (!forward){
    leftInvert = !leftInvert;
    rightInvert = !rightInvert;
  }
}

void draw(){
  background(255);
  
  stroke(255, 0, 0);
  line(0, 50, width, 50);
  
  noStroke();
  fill(0);
  rectMode(CORNERS);
  if (!leftInvert){
    rect(50, 50, 55, 50 - leftVal);
  }
  else {
    rect(50, 50, 55, 50 + leftVal);
  }
  
  if (!rightInvert){
    rect(100, 50, 105, 50 - rightVal);
  }
  else {
    rect(100, 50, 105, 50 + rightVal);
  }
}
