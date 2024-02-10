#include "Motion.h"

/*

float mowerAzimuth = 0;

void MotionMoveToTarget(){
  float PrevTargetAngle = angleBetweenPoints(prevPointLon, prevPointLat, targetPointLon, targetPointLat);
  float MowerTargetAngle = angleBetweenPoints(mowerLon, mowerLat, targetPointLon, targetPointLat);
  float AngleDiff = ShortestRotation(PrevTargetAngle, MowerTargetAngle);
  
  float MowerTargetDist = sqrt(pow(mowerLon - targetPointLon, 2) + pow(mowerLat - targetPointLat, 2));
  
  float DistAint = MowerTargetDist * cos(radians(abs(AngleDiff)));
  float DistOffset = MowerTargetDist * sin(radians(abs(AngleDiff)));
  
  //STRAYED FROM PATH
  if (DistOffset > MAX_DEVIATION){MainStop();}
  
  float RotationAngle = ShortestRotation(mowerAzimuth, MowerTargetAngle);
  
  //float DistanceToTarget = sqrt(pow(targetPointLon - mowerLon, 2) + pow(targetPointLat - mowerLat, 2));
  
  if (DistAint < 3 || AngleDiff > 90.0){
    ArrayList<int> targetPoint = AlgorithmNextPoint();
    targetPointLon = targetPoint.get(0);
    targetPointLat = targetPoint.get(1);
    
    MotionRotateToTarget();
  }
  else {
    float RotFactor = DistOffset / 5.0; if (RotFactor > 1){RotFactor = 1;}
    if (RotationAngle < 0){
      MotorGradualLeft(1 - RotFactor * 0.7);
    }
    else {
      MotorGradualRight(1 - RotFactor * 0.7);
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
*/