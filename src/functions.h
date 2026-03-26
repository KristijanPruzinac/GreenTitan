#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "Arduino.h"
#include "definitions.h"

#include <math.h>

#define LED_BUILTIN 2

void Warning(String message);
void Error(String message);
float gaussian_noise(float stddev);
void toggle_status_led();
void warning();
void error();

float AngleBetweenPoints(float start_x, float start_y, float end_x, float end_y);
float DistanceFromLine(float x, float y,
                       float start_x, float start_y,
                       float end_x,   float end_y);
float DistanceBetweenPoints(float x1, float y1, float x2, float y2);
bool has_passed_goal(float x, float y, float start_x, float start_y, float end_x, float end_y);
float normalize_angle(float radians);
float angle_diff(float current, float target);
int sign_of(float value);

#endif