#include <math.h>

// Calculate the angle between two points
// Angle is 0 for NORTH and revolves 360 degrees clockwise
float angleBetweenPoints(long x1, long y1, long x2, long y2) {
  // Calculate the angle in radians
  float angleRad = atan2(y2 - y1, x2 - x1);

  // Convert radians to degrees
  float angleDeg = angleRad * 180 / 3.1415926;

  // Adjust the angle to be between 0 and 360 degrees
  if (angleDeg < 0) {
    angleDeg += 360;
  }

  //Change to clockwise
  angleDeg = 360 - angleDeg;

  //Add 90 degrees
  angleDeg += 90;

  //Map
  if (angleDeg >= 360){
    angleDeg -= 360;
  }

  return angleDeg;
}

float Normalize(float angle){
  if (angle < 0) {
    angle += 360;
  } else if (angle > 360) {
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

long absVal(long val){
  if (val < 0){
    return -val;
  }

  return val;
}

String ShortestRotation(float currentAngle, float targetAngle, float straightLimit = 5.0){
  float CW = 0, CCW = 0;
  if (currentAngle < targetAngle){
    CW = targetAngle - currentAngle;
    CCW = currentAngle + (360.0 - targetAngle);
  }
  else if (currentAngle > targetAngle){
    CW = targetAngle + (360.0 - currentAngle);
    CCW = currentAngle - targetAngle;
  }

  if (absAngle(CW - CCW) < straightLimit){
    return "STRAIGHT";
  }
  else if (CW < CCW){
    return "CW";
  }
  else {
    return "CCW";
  }
}