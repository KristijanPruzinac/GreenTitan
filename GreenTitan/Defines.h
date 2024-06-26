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

//CONSTANTS
#define LEFT 0
#define RIGHT 1
#define FORWARD 1
#define BACKWARD 0
#define MILLIS_PER_SECOND 1000

#define MOTOR_ANGLE_MIN -90
#define MOTOR_ANGLE_MAX 90
#define MOTOR_MIN_SPEED 0.5

#define WAITING 0
#define ROTATING 1
#define MOVING 2
#define TEST 3

#define MOTION_ACCEPTED_DIST_TO_POINT 3
#define MOTION_ACCEPTED_ROTATION_TO_POINT 15

#define MOTION_ROTATION_FACTOR 1

#define DAC_MAX_VALUE 4096
#define DAC_MAX_BITS 12

#define BATTERY_LEVEL_LOW 8
#define BATTERY_LEVEL_CHARGED 95

#define MAG_CALIBRATION_SAMPLING_RATE 50
#define SENSORS_SAMPLING_RATE 10
#define GPS_SAMPLING_RATE 5

#define SERIAL_BAUDRATE 115200
#define GPS_BAUDRATE 19200
#define COMMUNICATION_TIMEOUT 150

#define MAG_CALIBRATION_SAMPLE_SIZE 500

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