#include "Functions.h"

float RadDeg(float radVal){
  return (radVal / (2 * PI)) * 360.0;
}

float DegRad(float degVal){
  return (degVal / 360.0) * (2 * PI);
}

float Distance(int x1, int y1, int x2, int y2){
  return sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1));
}

// Angle is 0 for NORTH and revolves 360 degrees clockwise
float AngleBetweenPoints(int x1, int y1, int x2, int y2) {
  // Calculate the angle in radians
  float angleRad = atan2(y2 - y1, x2 - x1);

  // Convert radians to degrees, map and normalize
  return NormalizeAngle((angleRad * 180 / PI) + 90.0);
}

float NormalizeAngle(float angle){
  while (angle < 0){
      angle += 360;
    }
  while (angle > 360){
    angle -= 360;
  }
  return angle;
}

float absAngle(float angle){
  if (angle < 0){
    return -angle;
  }

  return angle;
}

//Returns how much current angle needs to rotate to reach target angle
//Returns positive if angle needs to rotate clockwise
float ShortestRotation(float targetAngle, float currentAngle){
  float CW = 0, CCW = 0;
  if (currentAngle < targetAngle){
    CW = targetAngle - currentAngle;
    CCW = currentAngle + (360.0 - targetAngle);
  }
  else if (currentAngle > targetAngle){
    CW = targetAngle + (360.0 - currentAngle);
    CCW = currentAngle - targetAngle;
  }

  if (CW < CCW){
    return -CW;
  }
  else {
    return CCW;
  }
}

void Error(String message){
  Serial.println("ERROR: " + message);
  //TODO: Implement additional user feedback and uncomment
  while (1){
    delay(100);
  }
}