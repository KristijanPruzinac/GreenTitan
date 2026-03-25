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

#endif