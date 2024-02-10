float angleBetweenPoints(long x1, long y1, long x2, long y2);
float NormalizeAngle(float angle);
float absAngle(float angle);
float ShortestRotation(float targetAngle, float currentAngle);

/*
String[] splitStringByCharacters(String input, String separators) {
  ArrayList<String> result = new ArrayList<String>();
  int startIndex = 0;

  for (int i = 0; i < input.length(); i++) {
    char currentChar = input.charAt(i);
    if (separators.indexOf(currentChar) != -1) {
      result.add(input.substring(startIndex, i));
      startIndex = i + 1;
    }
  }

  result.add(input.substring(startIndex));
  
  return result.toArray(new String[result.size()]);
}*/

// Angle is 0 for NORTH and revolves 360 degrees clockwise
float angleBetweenPoints(long x1, long y1, long x2, long y2) {
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