// -------------------------------------------------------- DEFINITIONS ---------------------------------------------------------------
#include "Defines.h"

// -------------------------------------------------------- STATUS INDICATORS ---------------------------------------------------------------
int STATUS_BATTERY_LOW = false;

// -------------------------------------------------------- CONFIGURABLES ---------------------------------------------------------------
int MOWER_OVERLAP = 85;
int MAX_DEVIATION = 50;
int BASE_LON = -5672328; //TODO: REMOVE HARDCODED VALUE (Filled at power on)
int BASE_LAT = 3376391; //TODO: REMOVE HARDCODED VALUE (Filled at power on)

float BATTERY_LEVEL_MIN = 16;
float BATTERY_LEVEL_MAX = 18;

int SENSORS_SAMPLING_RATE = 10;

bool MOTOR_SIDE_INVERT = true;
bool MOTOR_LEFT_INVERT = true;
bool MOTOR_RIGHT_INVERT = true;
float MOTOR_OPTIMAL_VOLTAGE = 12;

// -------------------------------------------------------- FREE RTOS ---------------------------------------------------------------

QueueHandle_t BluetoothMainQueue;
QueueHandle_t MainBluetoothQueue;

QueueHandle_t SensorsMainQueue;

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

//Peripherals
#include "GPS.h";
#include "Gyro.h";
#include "Battery.h";
#include "Motor.h";
#include "RainSensor.h";

//Program dependencies
#include "Functions.h";
#include "Motion.h";
#include "Algorithm.h";
#include "Configuration.h";

//Tasks
#include "MainTask.h";
#include "BluetoothTask.h";
#include "SensorsTask.h";

// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void InitFreeRtos(){

  //CREATE TASKS

  //Main task
  xTaskCreatePinnedToCore (
    MainTask,     // Function to implement the task
    "MainTask",   // Name of the task
    204800,      // Stack size in bytes
    NULL,      // Task input parameter
    2,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );

  //Sensors task
  xTaskCreatePinnedToCore (
    SensorsTask,     // Function to implement the task
    "SensorsTask",   // Name of the task
    8192,      // Stack size in bytes
    NULL,      // Task input parameter
    1,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );

  //Bluetooth task
  xTaskCreatePinnedToCore (
    BluetoothTask,     // Function to implement the task
    "BluetoothTask",   // Name of the task
    8192,      // Stack size in bytes
    NULL,      // Task input parameter
    0,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );

  //CREATE QUEUES

  //BluetoothMain queue
  BluetoothMainQueue = xQueueCreate(1000, sizeof(char));
 
  if(BluetoothMainQueue == NULL){
    Serial.println("ERROR: cannot create BluetoothMain queue!");
  }

  //MainBluetooth queue
  MainBluetoothQueue = xQueueCreate(1000, sizeof(char));
 
  if(MainBluetoothQueue == NULL){
    Serial.println("ERROR: cannot create MainBluetooth queue!");
  }


  //SensorsMain queue
  SensorsMainQueue = xQueueCreate(1000, sizeof(char));
 
  if(SensorsMainQueue == NULL){
    Serial.println("ERROR: cannot create SensorsMain queue!");
  }
}

// -------------------------------------------------------- PROGRAM START ---------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  InitFreeRtos();

  InitMotors();
  InitGyro();
  InitGPS();
  InitBluetooth();
  InitBattery();
  InitRainSensor();
}

void loop(){
  //TODO: Remove
  Serial.println(STATUS_BATTERY_LOW);
  delay(100);
}
