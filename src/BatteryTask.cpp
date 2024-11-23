#include "BatteryTask.h"
 
//Sampling and voltage divider
const int battery_sampling_count = 100;
const float battery_divider_factor = 1.0 / 11.0; // 1k ohm MEASURED , 10k ohm
int battery_readings[battery_sampling_count];

//Last read battery data
float batteryCurrentLevel = 0;
float batteryCurrentPercentage = 0;

void InitBattery(){
  pinMode(BATTERY_LEVEL_PIN, INPUT);

  for (int i = 0; i < battery_sampling_count; i++){
    battery_readings[i] = analogRead(BATTERY_LEVEL_PIN);
  }
}

float BatteryRead(){
  //Average over sampling data
  int avg = 0;
  for (int i = 0; i < battery_sampling_count; i++){
    avg += battery_readings[i];
  }
  avg /= battery_sampling_count;

  return batteryCurrentLevel = ((avg / 4096.0) * 3.3) / battery_divider_factor;
}

int BatteryReadPercentage(){
  float val = BatteryRead();

  if (val <= BATTERY_LEVEL_MIN){return 0;}
  if (val >= BATTERY_LEVEL_MAX){return 100;}

  return batteryCurrentPercentage = (int) ((val - BATTERY_LEVEL_MIN) / (BATTERY_LEVEL_MAX - BATTERY_LEVEL_MIN) * 101.0);
}

float BatteryCurrentVoltage(){
  return batteryCurrentLevel;
}

void BatteryUpdate(){
  //Shift values back
  for (int i = 0; i < battery_sampling_count - 1; i++){
    battery_readings[i] = battery_readings[i + 1];
  }

  //Read new value
  battery_readings[battery_sampling_count - 1] = analogRead(BATTERY_LEVEL_PIN);

  //Save last ADC and percentage values
  BatteryReadPercentage();
}

void BatteryCheck(){
  int BatPercentage = BatteryReadPercentage();
  if (BatPercentage <= BATTERY_LEVEL_LOW){
    STATUS_BATTERY_LOW = true;
  }
  else {
    STATUS_BATTERY_LOW = false;
  }

  if (BatPercentage >= BATTERY_LEVEL_CHARGED){
    STATUS_BATTERY_CHARGED = true;
  }
  else {
    STATUS_BATTERY_CHARGED = false;
  }
}

void BatteryTask(void* pvParameters){
  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / BATTERY_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    //Read sensors
    BatteryUpdate();
    BatteryCheck();

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