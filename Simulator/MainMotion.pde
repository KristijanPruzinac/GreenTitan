//TODO: REMOVE HARDCODED VALUES (Filled at power on)
long mowerLon = -5672328;
long mowerLat = 3376391;

long targetPointLon = -5672328;
long targetPointLat = 3376391;

float mowerAzimuth = 0;
//--------------------------------------------------

//TODO: REMOVE SIMULATOR FUNCTIONALITY
void MotorStop(){MotorEmulation = "STOPPED";}
void MotorMainOn(){MotorMowing = true;}
void MotorMainOff(){MotorMowing = false;}
void MotorPivotLeft(){MotorEmulation = "MOVING"; TurnAmountLeft = 0; TurnAmountRight = 1;}
void MotorTurnLeft(){MotorEmulation = "ROTATING"; TurnAmountLeft = 0; TurnAmountRight = 1;}
void MotorPivotRight(){MotorEmulation = "MOVING"; TurnAmountLeft = 1; TurnAmountRight = 0;}
void MotorTurnRight(){MotorEmulation = "ROTATING"; TurnAmountLeft = 1; TurnAmountRight = 0;}
// -------------------------------

void MotionMoveToTarget(){
  float RotationAngle = ShortestRotation(mowerAzimuth, angleBetweenPoints(mowerLon, mowerLat, targetPointLon, targetPointLat));
  
  float DistanceToTarget = sqrt(pow(targetPointLon - mowerLon, 2) + pow(targetPointLat - mowerLat, 2));
  
  if (DistanceToTarget < 15){
    ArrayList<Long> targetPoint = AlgorithmNextPoint();
    targetPointLon = targetPoint.get(0);
    targetPointLat = targetPoint.get(1);
    
    MotionRotateToTarget();
  }
  else {
    if (RotationAngle < 0){
      MotorPivotLeft();
    }
    else {
      MotorPivotRight();
    }
  }
}

void MotionRotateToTarget(){
  float RotationAngle = ShortestRotation(mowerAzimuth, angleBetweenPoints(mowerLon, mowerLat, targetPointLon, targetPointLat));
  
  if (abs(RotationAngle) < 5){
    MotionMoveToTarget();
  }
  else {
    if (RotationAngle < 0){
      MotorTurnLeft();
    }
    else {
      MotorTurnRight();
    }
  }
}
