#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"

// -------------------------------------------------------- DEFINITIONS ---------------------------------------------------------------
#include "Defines.h"

// -------------------------------------------------------- STATUS INDICATORS ---------------------------------------------------------------

int STATUS_BATTERY_LOW = false;
int STATUS_BATTERY_CHARGED = false;

// -------------------------------------------------------- CONFIGURATION ---------------------------------------------------------------

// Save which peripherals have been configured
bool CONFIG_PATH = false;
bool CONFIG_BATTERY = false;
bool CONFIG_MOTORS = false;
bool CONFIG_GYRO = false;
bool CONFIG_RAIN_SENSOR = false;

// Enable or disable peripherals for testing
bool ENABLE_MOTORS = true;
bool ENABLE_IMU = true;
bool ENABLE_GPS = true;
bool ENABLE_BATTERY = true;
bool ENABLE_RAIN_SENSOR = true;
bool ENABLE_BLUETOOTH = true;

// -------------------------------------------------------- GLOBALS ---------------------------------------------------------------

bool SETUP_COMPLETED = true;

int MOWER_OVERLAP = 15; // In cm
int MAX_DEVIATION = 50;
long long BASE_LON = -5672328;
long long BASE_LAT = 3376391;
long long BASE_EXIT_LON = -5672328;
long long BASE_EXIT_LAT = 3376391;

int GPS_ACC_THRESHOLD = 30; //TODO: Revert to 18
int GPS_STABILITY_CHECK_DURATION = 15; // 5 minutes TODO: Revert to 5 minutes

float BATTERY_LEVEL_MIN = 16;
float BATTERY_LEVEL_MAX = 18;

bool MOTOR_SIDE_INVERT = true;
bool MOTOR_LEFT_INVERT = true;
bool MOTOR_RIGHT_INVERT = false;
float MOTOR_OPTIMAL_VOLTAGE = 12;

//TODO: ADD TO CONFIG
bool IMU_INVERT = false;
float MOTION_ACC_FACTOR = 1.0;

SemaphoreHandle_t IMUMutex;
SemaphoreHandle_t GPSMutex;
SemaphoreHandle_t MotionMutex;
SemaphoreHandle_t SensorInterfaceMutex;

QueueHandle_t GPS_SensorInterface_Queue;

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

// Peripherals
#include "IMUTask.h"
#include "GPSTask.h"
#include "BatteryTask.h"
#include "RainSensor.h"
#include "SensorInterfaceTask.h"

// Program dependencies
#include "Functions.h"
#include "Algorithm.h"
#include "Configuration.h"

// Tasks
#include "MotionTask.h"
#include "BluetoothTask.h"
#include "Motor.h"


// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void InitFreeRtos() {
    IMUMutex = xSemaphoreCreateMutex();
    GPSMutex = xSemaphoreCreateMutex();
    MotionMutex = xSemaphoreCreateMutex();
    SensorInterfaceMutex = xSemaphoreCreateMutex();

    GPS_SensorInterface_Queue = xQueueCreate(10, sizeof(GPS_SensorInterface_Unit));

    if (ENABLE_MOTORS) {
        xTaskCreatePinnedToCore(
            MotorTask,
            "MotorTask",
            8192,
            NULL,
            2,
            NULL,
            0
        );
    }

    if (ENABLE_IMU) {
        xTaskCreatePinnedToCore(
            IMUTask,
            "IMUTask",
            8192,
            NULL,
            2,
            NULL,
            0
        );
    }

    if (ENABLE_GPS) {
        xTaskCreatePinnedToCore(
            GPSTask,
            "GPSTask",
            8192,
            NULL,
            2,
            NULL,
            0
        );
    }

    if (ENABLE_BATTERY) {
        xTaskCreatePinnedToCore(
            BatteryTask,
            "BatteryTask",
            8192,
            NULL,
            1,
            NULL,
            0
        );
    }

    if (ENABLE_MOTORS && (ENABLE_IMU)) {
        xTaskCreatePinnedToCore(
            MotionTask,
            "MotionTask",
            8192,
            NULL,
            1,
            NULL,
            0
        );
    }

    if (ENABLE_BLUETOOTH) {
        xTaskCreatePinnedToCore(
            BluetoothTask,
            "BluetoothTask",
            8192,
            NULL,
            1,
            NULL,
            0
        );
    }

    xTaskCreatePinnedToCore(
            SensorInterfaceTask,
            "SensorInterfaceTask",
            8192,
            NULL,
            1,
            NULL,
            0
        );
}

void InitPeripherals() {
    if (ENABLE_BATTERY) InitBattery();
    if (ENABLE_RAIN_SENSOR) InitRainSensor();

    if (ENABLE_IMU) {
        if (!InitIMU()) {
            Error("IMU initialization failed.");
            return;
        }
    }

    if (ENABLE_GPS) {
        if (!InitGPS()) {
            Error("GPS initialization failed.");
            return;
        }
    }

    if (ENABLE_MOTORS) {
        if (!InitMotors()) {
            Error("Motor initialization failed.");
            return;
        }
    }

    if (ENABLE_BLUETOOTH) {
        if (!InitBluetooth()) {
            Error("Bluetooth initialization failed.");
            return;
        }
    }

    FileResult configResult = InitConfiguration();
    if (configResult != SUCCESS) {
        Error("Filesystem failed to initialize with FileResult error " + String(configResult));
    }

    Serial.println("All peripherals initialized successfully.");
}

// -------------------------------------------------------- PROGRAM START ---------------------------------------------------------------
void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    InitPeripherals();

    FileResult configResult = LoadConfiguration();
    if (configResult != SUCCESS) {
        configResult = SaveConfiguration();

        if (configResult != SUCCESS){
            Error("Unable to create missing configuration with FileResult error " + String(configResult));
        }

        Warning("Configuration file missing. Creating new configuration...");
    }
    else {
        Serial.println("Configuration loaded.");
    }
    
    delay(500);

    InitFreeRtos(); // Initialize FreeRTOS tasks

    delay(500);
}

String Mode = "POWER_ON";
/*
POWER_ON
SETUP
START
RUNNING PAUSE
STOP
CHARGING

SETUP_TEST
TEST
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

void loop() {
    if (Mode == "POWER_ON") {
        //TODO: Remove
        Mode = "TEST";
    } else if (Mode == "SETUP") {
        // Setup-specific logic
    } else if (Mode == "SETUP_TEST"){ // ### USED FOR TESTING ###

      AlgorithmCaptureStart();
      AlgorithmCaptureNewOutline();
      AlgorithmCaptureSetNewPoint(0, 0);
      AlgorithmCaptureSetNewPoint(20, 20);
      AlgorithmCaptureSetNewPoint(30, 50);
      AlgorithmCaptureSetNewPoint(10, 100);
      AlgorithmCaptureSetNewPoint(-20, 30);
      AlgorithmCaptureSetNewPoint(-25, -10);
      AlgorithmCaptureSetNewPoint(-10, 5);

      AlgorithmCaptureNewOutline();
      AlgorithmCaptureSetNewPoint(5, 30);
      AlgorithmCaptureSetNewPoint(5, 60);
      AlgorithmCaptureSetNewPoint(-10, 35);

      AlgorithmCaptureEnd();

      for (int i = 0; i < 200; i++){
        std::vector<long long> NextPointCoords = AlgorithmNextPoint();
        Serial.println(">gcode:" + String(NextPointCoords[0]) + ":" + String(NextPointCoords[1]) + "|xy");
      }

      delay(5000);

      AlgorithmCaptureStart();
      AlgorithmCaptureNewOutline();
      AlgorithmCaptureSetNewPoint(0, 0);
      AlgorithmCaptureSetNewPoint(-20, 20);
      AlgorithmCaptureSetNewPoint(-30, 50);
      AlgorithmCaptureSetNewPoint(-10, 100);
      AlgorithmCaptureSetNewPoint(20, 30);
      AlgorithmCaptureSetNewPoint(25, -10);
      AlgorithmCaptureSetNewPoint(10, 5);

      AlgorithmCaptureNewOutline();
      AlgorithmCaptureSetNewPoint(5, 30);
      AlgorithmCaptureSetNewPoint(50, 100);
      AlgorithmCaptureSetNewPoint(-100, 35);

      AlgorithmCaptureEnd();

      for (int i = 0; i < 200; i++){
        std::vector<long long> NextPointCoords = AlgorithmNextPoint();
        Serial.println(">gcode:" + String(NextPointCoords[0]) + ":" + String(NextPointCoords[1]) + "|xy");
      }

      delay(5000);

      AlgorithmCaptureStart();
      AlgorithmCaptureNewOutline();
      AlgorithmCaptureSetNewPoint(0, 0);
      AlgorithmCaptureSetNewPoint(0, 50);
      AlgorithmCaptureSetNewPoint(100, 50);
      AlgorithmCaptureSetNewPoint(-50, 150);
      AlgorithmCaptureSetNewPoint(85, 30);
      AlgorithmCaptureSetNewPoint(25, -10);
      AlgorithmCaptureSetNewPoint(10, 5);

      AlgorithmCaptureNewOutline();
      AlgorithmCaptureSetNewPoint(5, 30);
      AlgorithmCaptureSetNewPoint(50, 100);
      AlgorithmCaptureSetNewPoint(-100, 35);

      AlgorithmCaptureEnd();

      for (int i = 0; i < 200; i++){
        std::vector<long long> NextPointCoords = AlgorithmNextPoint();
        Serial.println(">gcode:" + String(NextPointCoords[0]) + ":" + String(NextPointCoords[1]) + "|xy");
      }

      delay(5000);

      SaveConfiguration();

      Mode = "TEST";
    } else if (Mode == "START") {
        String message = BluetoothRead();

        if (message == "GPS_ACC") {
            BluetoothWrite(String(GetGpsAcc()));
        } else if (message == "CAPTURE_START") {
            AlgorithmCaptureStart();
            BluetoothWrite("Executed!");
        } else if (message == "GPS_POS") {
            // BluetoothWrite(String(GetLon()) + " " + String(GetLat()) + " " + String((int) IMUGetHeading()));
        } else if (message == "CAPTURE_BASE_POINT") {
            AlgorithmCaptureBasePoint();
            BluetoothWrite("Executed!");
        } else if (message == "CAPTURE_BASE_EXIT_POINT") {
            AlgorithmCaptureBaseExitPoint();
            BluetoothWrite("Execulight themeted!");
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
        } else if (message == "GET_PATH_STRING") {
            BluetoothWrite(AlgorithmGetPathString());
        } else if (message == "SAVE_CONFIGURATION") {
            FileResult result = SaveConfiguration();
            BluetoothWrite(String(result));
        } else if (message == "MOWER_START") {
            Mode = "RUNNING";
        } else if (message == "TEST" || message == "GPS/GET/ACCURACY") {
            MotionSetTargetPoint(GetLon(), GetLat() + 800);
        }
    } else if (Mode == "RUNNING") {
      delay(200);
    } else if (Mode == "TEST") { // ### USED FOR TESTING ###
        Serial.println("TESTING STARTED");
        /*
        delay(5000);

        MotionSetTargetPointRotation(90); // Rotate to 90 degrees
        while (MowerIsInMotion()) {
            delay(100);
        }
        delay(1000);

        MotionSetTargetPointRotation(180); // Rotate to 90 degrees
        while (MowerIsInMotion()) {
            delay(100);
        }
        delay(1000);

        MotionSetTargetPointRotation(0); // Rotate to 90 degrees
        while (MowerIsInMotion()) {
            delay(100);
        }
        delay(1000);

        Mode = "RUNNING";
        */
        while (1){
            delay(5000);

            //PlotIMUData(); // Plot IMU data for debugging
/*
            while (!IsGpsStable()) {
                Serial.print(">");
                Serial.print("Accuracy:");
                Serial.print(GetGpsAcc());
                Serial.print("\r\n");
                delay(200);
            }*/

            long startLon = GetLon();
            long startLat = GetLat();

            for (int i = 0; i < 1; i++){
                MotionSetTargetPoint(startLon, startLat + 200);

                while (MowerIsInMotion()) {
                    Serial.print(">");
                    Serial.print("Heading:");
                    Serial.print(GetHeading());
                    Serial.print(",Accuracy:");
                    Serial.print(GetGpsAcc());
                    Serial.print(",Reliabillity:");
                    Serial.print(GetHeadingReliability());
                    Serial.print("\r\n");
                    delay(50);
                }

                MotionSetTargetPoint(startLon + 200, startLat + 200);

                while (MowerIsInMotion()) {
                    Serial.print(">");
                    Serial.print("Heading:");
                    Serial.print(GetHeading());
                    Serial.print(",Accuracy:");
                    Serial.print(GetGpsAcc());
                    Serial.print(",Reliabillity:");
                    Serial.print(GetHeadingReliability());
                    Serial.print("\r\n");
                    delay(50);
                }

                MotionSetTargetPoint(startLon + 200, startLat);

                while (MowerIsInMotion()) {
                    Serial.print(">");
                    Serial.print("Heading:");
                    Serial.print(GetHeading());
                    Serial.print(",Accuracy:");
                    Serial.print(GetGpsAcc());
                    Serial.print(",Reliabillity:");
                    Serial.print(GetHeadingReliability());
                    Serial.print("\r\n");
                    delay(50);
                }

                MotionSetTargetPoint(startLon, startLat);

                while (MowerIsInMotion()) {
                    Serial.print(">");
                    Serial.print("Heading:");
                    Serial.print(GetHeading());
                    Serial.print(",Accuracy:");
                    Serial.print(GetGpsAcc());
                    Serial.print(",Reliabillity:");
                    Serial.print(GetHeadingReliability());
                    Serial.print("\r\n");
                    delay(50);
                }
            }

            delay(50000);
        }
    }
    delay(10);
}
