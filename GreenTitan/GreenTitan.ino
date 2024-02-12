// -------------------------------------------------------- DEFINITIONS ---------------------------------------------------------------
#include "Defines.h"

#include <vector> //TODO: Remove

// -------------------------------------------------------- STATUS INDICATORS ---------------------------------------------------------------
int STATUS_BATTERY_LOW = false;

// -------------------------------------------------------- CONFIGURATION ---------------------------------------------------------------

bool CONFIG_PATH = false;
bool CONFIG_BATTERY = false;
bool CONFIG_MOTORS = false;
bool CONFIG_GYRO = false;
bool CONFIG_RAIN_SENSOR = false;

// -------------------------------------------------------- GLOBALS ---------------------------------------------------------------

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

  FileResult result = InitConfiguration();
  if (result != SUCCESS){
    //TODO: Implement proper error handling
    Serial.println("Configuration failed to initialize with error " + String(result) + " Rebooting...");
    abort();
  }
  InitMotors();
  InitGyro();
  InitGPS();
  InitBluetooth();
  InitBattery();
  InitRainSensor();

  //Points
  // Start the first outline

  /*
    AlgorithmCaptureStart();

    // Set points for the first outline
    AlgorithmCaptureSetNewPoint(-5672428, 3376291);
    AlgorithmCaptureSetNewPoint(-5672337, 3376280);
    AlgorithmCaptureSetNewPoint(-5672243, 3376319);
    AlgorithmCaptureSetNewPoint(-5672130, 3376305);
    AlgorithmCaptureSetNewPoint(-5672130, 3376240);
    AlgorithmCaptureSetNewPoint(-5672184, 3376188);
    AlgorithmCaptureSetNewPoint(-5672143, 3375989);
    AlgorithmCaptureSetNewPoint(-5672291, 3375864);
    AlgorithmCaptureSetNewPoint(-5672455, 3375851);
    AlgorithmCaptureSetNewPoint(-5672405, 3375956);
    AlgorithmCaptureSetNewPoint(-5672501, 3375991);
    AlgorithmCaptureSetNewPoint(-5672613, 3375940);
    AlgorithmCaptureSetNewPoint(-5672609, 3376093);
    AlgorithmCaptureSetNewPoint(-5672548, 3376230);

    // End the first outline
    AlgorithmCaptureNewOutline();

    // Set points for the second outline
    AlgorithmCaptureSetNewPoint(-5672355, 3376011);
    AlgorithmCaptureSetNewPoint(-5672368, 3375913);
    AlgorithmCaptureSetNewPoint(-5672262, 3375942);
    AlgorithmCaptureSetNewPoint(-5672215, 3376036);
    AlgorithmCaptureSetNewPoint(-5672315, 3376075);

    // End the second outline
    AlgorithmCaptureNewOutline();

    // Set points for the third outline
    AlgorithmCaptureSetNewPoint(-5672408, 3376080);
    AlgorithmCaptureSetNewPoint(-5672564, 3376021);
    AlgorithmCaptureSetNewPoint(-5672496, 3376222);
    AlgorithmCaptureSetNewPoint(-5672460, 3376164);
    AlgorithmCaptureSetNewPoint(-5672371, 3376208);
    AlgorithmCaptureSetNewPoint(-5672340, 3376148);

    AlgorithmCaptureEnd();
    */

    LoadConfiguration();

/*
    for (int i = 0; i < 100; i++){
      std::vector<int> nextPoint = AlgorithmNextPoint();
      Serial.println(String(nextPoint.at(0)) + " " + String(nextPoint.at(1)));
    }
    */
}

void loop(){
  delay(100);
}
