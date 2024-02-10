#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "Arduino.h"
#include "Defines.h"

float angleBetweenPoints(long x1, long y1, long x2, long y2);
float NormalizeAngle(float angle);
float absAngle(float angle);
float ShortestRotation(float targetAngle, float currentAngle);

#endif