#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

extern SemaphoreHandle_t IMUMutex;

extern bool IMU_INVERT;

bool InitIMU();
void IMUCalibrate();
void IMURead();

//TODO: REMOVE
extern void BluetoothWrite(String message);

float IMUGetAccX();
float IMUGetAccY();
float IMUGetAccZ();

float IMUGetGyroX();
float IMUGetGyroY();
float IMUGetGyroZ();

float IMUGetTemp();

#endif