#include "SensorsTask.h"
 
 /*
void SensorsRainSensor(){}
void SensorsLowPower(){} DONE
void SensorsChargedPower(){} DONE
void SensorsMowingTimeFrame(){}
void SensorsObstacle(){}
void SensorsGPSAccuracyLoss(){} DONE
void SensorsMowerLifted(){}
*/

void QueueSensorsMain(){}

void SensorsTask(void* pvParameters){
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / SENSORS_SAMPLING_RATE);

    xLastWakeTime = xTaskGetTickCount();

    //Read sensors
    BatteryUpdate();
    BatteryCheck();

    IMURead();

    GPSRead();
    GPSCheck();

    //TODO: Implement rain sensor

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}