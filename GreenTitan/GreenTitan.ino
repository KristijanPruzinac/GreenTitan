// -------------------------------------------------------- PIN DEFINITIONS ---------------------------------------------------------------

//Relay
#define MOTOR_left_A 23
#define MOTOR_left_B 22
#define MOTOR_right_A 21
#define MOTOR_right_B 19
#define MOTOR_main 18

//Battery voltage pin
#define BATTERY_voltage_pin 32

// -------------------------------------------------------- FREE RTOS ---------------------------------------------------------------

QueueHandle_t BluetoothMainQueue;
QueueHandle_t MainBluetoothQueue;

QueueHandle_t InterruptsMainQueue;

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

//Peripherals
#include "GPS.h";
#include "Gyro.h";
#include "Motor.h";
#include "Battery.h";
#include "RainSensor.h";

//Program dependencies
#include "Functions.h";
//#include "Algorithm.h";
#include "Configuration.h";
//#include "Motion.h";

//Tasks
#include "MainTask.h";
#include "BluetoothTask.h";
#include "InterruptsTask.h";

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

  //Interrupts task
  xTaskCreatePinnedToCore (
    InterruptsTask,     // Function to implement the task
    "InterruptsTask",   // Name of the task
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


  //InterruptsMain queue
  InterruptsMainQueue = xQueueCreate(1000, sizeof(char));
 
  if(InterruptsMainQueue == NULL){
    Serial.println("ERROR: cannot create InterruptsMain queue!");
  }
}

// -------------------------------------------------------- PROGRAM START ---------------------------------------------------------------
void setup() {
  InitFreeRtos();

  InitMotors();
  InitGyro();
  InitGPS();
  InitBluetooth();
  InitBattery();
  InitRainSensor();

  Serial.begin(9600);
}

