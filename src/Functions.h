#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "Arduino.h"
#include "Defines.h"

#include <math.h>

float RadDeg(float radVal);
float DegRad(float degVal);
float Distance(long long x1, long long y1, long long x2, long long y2);
float AngleBetweenPoints(long long x1, long long y1, long long x2, long long y2);
float NormalizeAngle(float angle);
float absAngle(float angle);
float ShortestRotation(float targetAngle, float currentAngle);
void Warning(String message);
void Error(String message);

#endif