#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "Arduino.h"
#include "Defines.h"

#include <math.h>

float RadDeg(float radVal);
float DegRad(float degVal);
float Distance(int x1, int y1, int x2, int y2);
float AngleBetweenPoints(int x1, int y1, int x2, int y2);
float NormalizeAngle(float angle);
float absAngle(float angle);
float ShortestRotation(float targetAngle, float currentAngle);
void Error(String message);

#endif