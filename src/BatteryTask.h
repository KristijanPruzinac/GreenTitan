#ifndef BATTERY_TASK_H
#define BATTERY_TASK_H

#include "Arduino.h"
#include "Defines.h"

extern int STATUS_BATTERY_LOW;
extern int STATUS_BATTERY_CHARGED;

extern float BATTERY_LEVEL_MIN;
extern float BATTERY_LEVEL_MAX;

void InitBattery();
float BatteryRead();
int BatteryReadPercentage();
float BatteryCurrentVoltage();
void BatteryUpdate();
void BatteryCheck();

void BatteryTask(void* pvParameters);

#endif