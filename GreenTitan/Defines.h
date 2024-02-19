#ifndef DEFINES_H
#define DEFINES_H

//PINOUT

//Gps
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

//Motor
#define MOTOR_LEFT_A 23
#define MOTOR_LEFT_B 19
#define MOTOR_RIGHT_A 13
#define MOTOR_RIGHT_B 27
#define MOTOR_MAIN 18

//Battery
#define BATTERY_LEVEL_PIN 36

//Gyro
#define GYRO_SDA_PIN 21
#define GYRO_SCL_PIN 22

//Rain sensor
#define RAIN_SENSOR_PIN 39

//Constants
#define LEFT 0
#define RIGHT 1
#define MILLIS_PER_SECOND 1000

//STATUS
#define BATTERY_LEVEL_LOW 8

//CONSTANTS
#define SENSORS_SAMPLING_RATE 5

enum FileResult {
    SUCCESS,
    FAILED_OPEN,
    NOT_A_DIRECTORY,
    MKDIR_FAILED,
    RMDIR_FAILED,
    FAILED_READ,
    FAILED_WRITE,
    APPEND_FAILED,
    RENAME_FAILED,
    DELETE_FAILED,
    INIT_FAILED,
    ALGORITHM_FAILED
};

#endif