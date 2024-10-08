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
  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / SENSORS_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    //Read sensors
    BatteryUpdate();
    BatteryCheck();

    //IMURead();
    GPSRead();


    if (counter % 2 == 0){
      //toSend += String(String(IMUGetHeading()) + " " + String(millis()) + "\n");
    }
    counter++;

    if (counter >= 10){
      counter = 0;

      //BluetoothWrite(toSend);

      toSend = "";
    }
    

    //TODO: Implement rain sensor

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}