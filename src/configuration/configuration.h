#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "Arduino.h"

#include "../definitions.h"
#include "../functions.h"

#include <Preferences.h>

enum FileResult {
    SUCCESS = 0,
    INIT_FAILED,
    FAILED_OPEN,
    FAILED_WRITE,
    ALGORITHM_FAILED,
};

// Config flags
extern bool CONFIG_PATH;
extern bool CONFIG_BATTERY;
extern bool CONFIG_MOTORS;
extern bool CONFIG_GYRO;
extern bool CONFIG_RAIN_SENSOR;
extern bool CONFIG_DATUM;

// Globals
extern bool SETUP_COMPLETED;

extern int MOWER_OVERLAP;
extern int MAX_DEVIATION;
extern double BASE_LON;
extern double BASE_LAT;
extern int64_t BASE_EXIT_X_CM;
extern int64_t BASE_EXIT_Y_CM;

extern int GPS_ACC_THRESHOLD;
extern int GPS_STABILITY_CHECK_DURATION_SECONDS;

extern float BATTERY_LEVEL_MIN;
extern float BATTERY_LEVEL_MAX;

extern bool MOTOR_SIDE_INVERT;
extern bool MOTOR_LEFT_INVERT;
extern bool MOTOR_RIGHT_INVERT;
extern float MOTOR_OPTIMAL_VOLTAGE;

extern bool IMU_INVERT;
extern float MOTION_ACC_FACTOR;

extern String AlgorithmGetPathString();
extern bool AlgorithmPopulatePathFromString(String& readData);

FileResult InitConfiguration();
FileResult SaveConfiguration();
FileResult LoadConfiguration();

#endif