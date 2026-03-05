#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"

// -------------------------------------------------------- DEFINITIONS ---------------------------------------------------------------
#include "Definitions.h"

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

bool MOTOR_SIDE_INVERT = false;
bool MOTOR_LEFT_INVERT = true;
bool MOTOR_RIGHT_INVERT = false;
float MOTOR_OPTIMAL_VOLTAGE = 12;

//TODO: ADD TO CONFIG
bool IMU_INVERT = false;
float MOTION_ACC_FACTOR = 1.0;

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

// Sensors
#include "sensor/imu_task.h"
#include "sensor/gps_task.h"

// Effectors
#include "effector/motor_task.h"


// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void InitFreeRtos() {
    if (ENABLE_MOTORS) {
        xTaskCreatePinnedToCore(
            motor_task,
            "MotorTask",
            2048,
            NULL,
            2,
            NULL,
            0
        );
    }

    if (ENABLE_IMU) {
        xTaskCreatePinnedToCore(
            imu_task,
            "IMUTask",
            2048,
            NULL,
            2,
            NULL,
            0
        );
    }

    if (ENABLE_GPS) {
        xTaskCreatePinnedToCore(
            gps_task,
            "GPSTask",
            2048,
            NULL,
            2,
            NULL,
            0
        );
    }
}

void InitPeripherals() {
    if (ENABLE_IMU) {
        if (!init_imu()) {
            Error("IMU initialization failed.");
            return;
        }
    }

    if (ENABLE_GPS) {
        if (!init_gps()) {
            Error("GPS initialization failed.");
            return;
        }
    }

    if (ENABLE_MOTORS) {
        if (!init_motors()) {
            Error("Motor initialization failed.");
            return;
        }
    }
}

// -------------------------------------------------------- PROGRAM START ---------------------------------------------------------------
void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    InitPeripherals();
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

void loop() {
    vTaskDelay(1000);
}