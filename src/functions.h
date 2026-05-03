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

float AngleBetweenPoints(double start_x, double start_y, double end_x, double end_y);
double DistanceFromLine(double x, double y,
                        double start_x, double start_y,
                        double end_x,   double end_y);
double DistanceBetweenPoints(double x1, double y1, double x2, double y2);
bool has_passed_goal(double x, double y, double start_x, double start_y, double end_x, double end_y);
float normalize_angle(float radians);
float angle_diff(float current, float target);
int sign_of(double value);
#endif