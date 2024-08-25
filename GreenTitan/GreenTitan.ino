// -------------------------------------------------------- DEFINITIONS ---------------------------------------------------------------
#include "Defines.h"

// -------------------------------------------------------- STATUS INDICATORS ---------------------------------------------------------------

int STATUS_BATTERY_LOW = false;
int STATUS_BATTERY_CHARGED = false;
int GPS_ACCURACY_STABLE = false;

// -------------------------------------------------------- CONFIGURATION ---------------------------------------------------------------

bool CONFIG_PATH = false;
bool CONFIG_BATTERY = false;
bool CONFIG_MOTORS = false;
bool CONFIG_GYRO = false;
bool CONFIG_RAIN_SENSOR = false;

// -------------------------------------------------------- GLOBALS ---------------------------------------------------------------

bool SETUP_COMPLETED = true;

int MOWER_OVERLAP = 15; //In cm
int MAX_DEVIATION = 50;
int BASE_LON = -5672328;
int BASE_LAT = 3376391;
int BASE_EXIT_LON = -5672328;
int BASE_EXIT_LAT = 3376391;

int GPS_ACC_THRESHOLD = 18;
int GPS_STABILITY_CHECK_DURATION = 300; //5 minutes

float BATTERY_LEVEL_MIN = 16;
float BATTERY_LEVEL_MAX = 18;

bool MOTOR_SIDE_INVERT = false;
bool MOTOR_LEFT_INVERT = false;
bool MOTOR_RIGHT_INVERT = true;
float MOTOR_OPTIMAL_VOLTAGE = 12;

//TODO: ADD TO CONFIG
bool IMU_INVERT = false;
float MOTION_ACC_FACTOR = 1.0;

SemaphoreHandle_t IMUMutex;
SemaphoreHandle_t GPSMutex;

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

//Peripherals
#include "IMU.h"
#include "GPS.h"
#include "Battery.h"
#include "RainSensor.h"

//Program dependencies
#include "Functions.h"
#include "Algorithm.h"
#include "Configuration.h"

//Tasks
#include "MotionTask.h"
#include "BluetoothTask.h"
#include "SensorsTask.h"
#include "Motor.h"

// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void InitFreeRtos(){
  IMUMutex = xSemaphoreCreateMutex();
  GPSMutex = xSemaphoreCreateMutex();

  //Motor task
  xTaskCreatePinnedToCore (
    MotorTask,     // Function to implement the task
    "MotorTask",   // Name of the task
    8192,      // Stack size in bytes
    NULL,      // Task input parameter
    2,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );

  //IMU task
  xTaskCreatePinnedToCore (
    IMUTask,     // Function to implement the task
    "IMUTask",   // Name of the task
    8192,      // Stack size in bytes
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
    2,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );

  //Motion task
  xTaskCreatePinnedToCore (
    MotionTask,     // Function to implement the task
    "MotionTask",   // Name of the task
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
}

extern void InitMotors();
extern bool InitIMU();
extern void InitGPS();
extern void InitBattery();
extern void InitRainSensor();

// -------------------------------------------------------- PROGRAM START ---------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
}

String Mode = "POWER_ON";
/*
POWER_ON

SETUP

START
RUNNING PAUSE
STOP
CHARGING
*/

void MainCharging();
void MainChargingStart();
void MainChargingStop();
void MainStop();
void MainPause();
void MainStart();
void MainRunning();
void MainPowerOn();
void MainSetup();

void loop(){
  if (Mode == "POWER_ON"){
    FileResult result = InitConfiguration();
    if (result != SUCCESS){
      Error("Configuration failed to initialize with error " + String(result));
    }

    Serial.println("Configuration loaded!");

    delay(200);

    InitMotors();
    if (!InitIMU()){
      Error("Gyro failed to initialize!");
    }
    InitGPS();
    InitBattery();
    InitRainSensor();

    Serial.println("Peripherals initialized!");

    delay(200);

    if (!InitBluetooth()){
      Error("Bluetooth failed to initialize!");
    }
    else {
      Serial.println("Bluetooth initialized!");
    }

    delay(200);

    InitFreeRtos();

    Serial.println("FreeRtos initialized!");

    //TODO: Move to setup
    IMUCalibrate();

    delay(2500);
    if (SETUP_COMPLETED){
      Mode = "START";
    }
    else {
      Mode = "SETUP";
    }
  }
  else if (Mode == "SETUP"){

  }
  else if (Mode == "START"){
    String message = BluetoothRead();

    if (message == "GPS_ACC"){
      BluetoothWrite(String(GpsGetAcc()));
    }
    else if (message == "CAPTURE_START") {
        AlgorithmCaptureStart();
        BluetoothWrite("Executed!");
    } else if (message == "GPS_POS"){
        //BluetoothWrite(String(GpsGetLon()) + " " + String(GpsGetLat()) + " " + String((int) IMUGetHeading()));
    } else if (message == "CAPTURE_BASE_POINT") {
        AlgorithmCaptureBasePoint();
        BluetoothWrite("Executed!");
    } else if (message == "CAPTURE_BASE_EXIT_POINT") {
        AlgorithmCaptureBaseExitPoint();
        BluetoothWrite("Executed!");
    } else if (message == "CAPTURE_NEW_OUTLINE") {
        AlgorithmCaptureNewOutline();
        BluetoothWrite("Executed!");
    } else if (message == "CAPTURE_NEW_POINT") {
        AlgorithmCaptureNewPoint();
        BluetoothWrite("Executed!");
    } else if (message == "REMOVE_OUTLINE") {
        bool result = AlgorithmCaptureRemoveOutline();
        BluetoothWrite(String(result));
    } else if (message == "REMOVE_POINT") {
        bool result = AlgorithmCaptureRemovePoint();
        BluetoothWrite(String(result));
    } else if (message == "CAPTURE_END") {
        bool result = AlgorithmCaptureEnd();
        BluetoothWrite(String(result));
    } else if (message == "GET_PATH_STRING"){
        BluetoothWrite(AlgorithmGetPathString());
    } else if (message == "SAVE_CONFIGURATION"){
        FileResult result = SaveConfiguration();
        BluetoothWrite(String(result));
    } else if (message == "TEST" || message == "GPS/GET/ACCURACY"){
      MotionSetTarget(GpsGetLon(), GpsGetLat() + 800);
    }
  }
  delay(10);
}