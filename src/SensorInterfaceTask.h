#ifndef SENSOR_INTERFACE_TASK_H
#define SENSOR_INTERFACE_TASK_H

#include <Arduino.h>
#include "Defines.h"
#include "Functions.h"

extern SemaphoreHandle_t IMUMutex;
extern SemaphoreHandle_t GPSMutex;
extern SemaphoreHandle_t MotionMutex;
extern SemaphoreHandle_t SensorInterfaceMutex;
extern QueueHandle_t GPS_SensorInterface_Queue;

extern bool ENABLE_GPS;

extern int GPS_ACC_THRESHOLD;
extern int GPS_STABILITY_CHECK_DURATION;
extern int GPS_ACCURACY_STABLE;

extern float IMU_RotSpeedRaw;
extern float IMU_RotSpeed;
extern float IMU_RotAcc;
extern float IMU_HeadingChange;

extern long GPS_GetLon();
extern long GPS_GetLat();
extern long GPS_GetAcc();

extern bool MowerIsInMotion();

//Internal
void SensorInterface_CorrectHeading();
void SensorInterface_GPS_UpdateHeading();
void SensorInterface_IMU_UpdateHeading();
void GPS_CheckStabillity();

//IMU
float GetRotationSpeedRaw();
float GetRotationSpeed();
float GetRotationAcceleration();
void UpdateHeading();

//GPS
long GetLon();
long GetLat();
long GetGpsAcc();

//Interface
float GetHeading();
float GetHeadingReliability();
bool IsGpsStable();

void SensorInterfaceTask(void* pvParameters);

//DEBUG
void PlotIMUData();

#endif