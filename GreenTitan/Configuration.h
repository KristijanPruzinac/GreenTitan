#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "Arduino.h"
#include "Defines.h"

#include "FS.h"
#include <LittleFS.h>

//Config
extern bool CONFIG_PATH ;
extern bool CONFIG_BATTERY;
extern bool CONFIG_MOTORS;
extern bool CONFIG_GYRO;
extern bool CONFIG_RAIN_SENSOR;

//Globals
extern int MOWER_OVERLAP;
extern int MAX_DEVIATION;
extern int BASE_LON;
extern int BASE_LAT;

extern int GPS_ACC_THRESHOLD;
extern int GPS_STABILITY_CHECK_DURATION;

extern float BATTERY_LEVEL_MIN;
extern float BATTERY_LEVEL_MAX;

extern bool MOTOR_SIDE_INVERT;
extern bool MOTOR_LEFT_INVERT;
extern bool MOTOR_RIGHT_INVERT;
extern float MOTOR_OPTIMAL_VOLTAGE;

extern String AlgorithmGetPathString();
extern bool AlgorithmPopulatePathFromString(String& readData);

FileResult listDir(fs::FS &fs, const char * dirname, uint8_t levels);
FileResult createDir(fs::FS &fs, const char * path);
FileResult removeDir(fs::FS &fs, const char * path);
FileResult readFile(fs::FS &fs, const char *path, String &fileContent);
FileResult writeFile(fs::FS &fs, const char * path, const char * message);
FileResult appendFile(fs::FS &fs, const char * path, const char * message);
FileResult renameFile(fs::FS &fs, const char * path1, const char * path2);
FileResult deleteFile(fs::FS &fs, const char * path);

FileResult InitConfiguration();
FileResult SaveConfiguration();
FileResult LoadConfiguration();

#endif