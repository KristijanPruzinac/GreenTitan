//FUNCTIONAL

#ifndef BATTERY_H
#define BATTERY_H

#include "Arduino.h"
#include "Defines.h"

extern int STATUS_BATTERY_LOW;

extern float BATTERY_LEVEL_MIN;
extern float BATTERY_LEVEL_MAX;

void InitBattery();
float BatteryRead();
int BatteryReadPercentage();
float BatteryCurrentVoltage();
void BatteryUpdate();
void BatteryCheck();

#endif