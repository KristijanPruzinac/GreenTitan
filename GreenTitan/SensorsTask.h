#ifndef SENSORS_TASK_H
#define SENSORS_TASK_H

#include "Arduino.h"
#include "Defines.h"

extern int SENSORS_SAMPLING_RATE;
extern int GPS_ACC_THRESHOLD;
extern int GPS_STABILITY_CHECK_DURATION;

extern int GPS_ACCURACY_STABLE;

extern int GpsGetAcc();

extern void BatteryUpdate();
extern void GyroRead();
extern bool GPSRead();

void SensorsRainSensor();
void SensorsLowPower();
void SensorsMowingTimeFrame();
void SensorsObstacle();
void SensorsGPSAccuracyLoss();
void SensorsMowerLifted();
void QueueSensorsMain();
void SensorsTask(void* pvParameters);

#endif