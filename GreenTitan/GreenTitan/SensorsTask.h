#ifndef SENSORS_TASK_H
#define SENSORS_TASK_H

#include "Arduino.h"
#include "Defines.h"

extern void BatteryUpdate();
extern void BatteryCheck();
extern void IMURead();
extern bool GPSRead();
extern void GPSCheck();

extern float BatteryCurrentVoltage();

void SensorsRainSensor();
void SensorsLowPower();
void SensorsMowingTimeFrame();
void SensorsObstacle();
void SensorsGPSAccuracyLoss();
void SensorsMowerLifted();
void QueueSensorsMain();
void SensorsTask(void* pvParameters);

extern float IMUGetAzimuth();
extern int GpsGetLon();
extern int GpsGetLat();
extern int GpsGetAcc();
extern void BluetoothWrite(String message);

#endif