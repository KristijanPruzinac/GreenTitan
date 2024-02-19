#include "SensorsTask.h"

unsigned long TimerBattery = -1;
unsigned long TimerGPS = -1;

bool TimerBatteryActive = false;
bool TimerGPSActive = false;
 
 /*
void SensorsRainSensor(){}
void SensorsLowPower(){}
void SensorsMowingTimeFrame(){}
void SensorsObstacle(){}
void SensorsGPSAccuracyLoss(){}
void SensorsMowerLifted(){}
*/

void QueueSensorsMain(){}

void SensorsTask(void* pvParameters){
  TimerBattery = millis();
  TimerGPS = millis();

  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / SENSORS_SAMPLING_RATE);

    xLastWakeTime = xTaskGetTickCount();

    //Read sensors
    BatteryUpdate();
    GyroRead();

    GPSRead();
    int gpsAccuracy = GpsGetAcc();
    if (gpsAccuracy <= GPS_ACC_THRESHOLD){ //Check to see if accuracy is within threshold, and if so try to check if it is stable
      if (!GPS_ACCURACY_STABLE){
        if (TimerGPSActive){
          if ((millis() - TimerGPS) / MILLIS_PER_SECOND > (unsigned long) GPS_STABILITY_CHECK_DURATION){ //If accuracy is stable for long enough, set GPS_ACCURACY_STABLE to true
            TimerGPSActive = false;
            GPS_ACCURACY_STABLE = true;
          }
        }
        else {  //Start timing
          TimerGPSActive = true;
          TimerGPS = millis();
        }
      }
    }
    else {  //Accuracy loss, immediately determines GPS_ACCURACY_STABLE = false
      GPS_ACCURACY_STABLE = false;
      TimerGPSActive = false;
    }

    //TODO: Implement rain sensor

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}