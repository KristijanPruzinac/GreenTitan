#ifndef SENSORS_TASK_H
#define SENSORS_TASK_H

#include "Arduino.h"
#include "Defines.h"

extern QueueHandle_t SensorsMainQueue;

void SensorsSetup();
void SensorsRainSensor();
void SensorsLowPower();
void SensorsMowingTimeFrame();
void SensorsObstacle();
void SensorsGPSAccuracyLoss();
void SensorsMowerLifted();
void QueueSensorsMain();
void SensorsTask(void* pvParameters);

void SensorsSetup(){}
 
void SensorsRainSensor(){}
void SensorsLowPower(){}
void SensorsMowingTimeFrame(){}
void SensorsObstacle(){}
void SensorsGPSAccuracyLoss(){}
void SensorsMowerLifted(){}

void QueueSensorsMain(){}

void SensorsTask(void* pvParameters){
  while (1){
    //Read sensors
    BatteryUpdate();

    //Sensors
    if (STATUS_BATTERY_LOW) SensorsLowPower();

    delay(SENSORS_SAMPLING_RATE);
  }
}

#endif