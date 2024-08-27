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

#define WAITING 0
#define ROTATING 1
#define MOVING 2
#define TEST 3

//Motion
#define MOTION_ACCEPTED_DIST_TO_POINT 3
#define MOTION_ACCEPTED_ROTATION_TO_POINT 10

#define MOTION_ROTATION_FACTOR 1

#define MOTION_CORRECTION_SPEED 60
#define MOTION_CORRECTION_ACCELERATION 60
#define MOTION_CORRECTION_ACCELERATION_FACTOR 0.5
#define MOTION_CORRECTION_REACTION 5 //Adjust

#define ACCELERATION_CORRECTION_REACTION 2 //Adjust

#define GPS_HEADING_CORRECTION_FACTOR 0.02
#define GPS_HEADING_CORRECTION_ANGLE 2.5
#define GPS_MIN_DIST_BETWEEN_READINGS 10

//Dac
#define DAC_MAX_VALUE 4096
#define DAC_MAX_BITS 12

#define BATTERY_LEVEL_LOW 8
#define BATTERY_LEVEL_CHARGED 95

//Sensors
#define MAG_CALIBRATION_SAMPLE_SIZE 500

#define MAG_CALIBRATION_SAMPLING_RATE 50
#define MOTION_UPDATE_FREQUENCY 10
#define IMU_UPDATE_FREQUENCY 50
#define SENSORS_UPDATE_FREQUENCY 10
#define MOTOR_UPDATE_FREQUENCY 1000
#define GPS_UPDATE_FREQUENCY 5

//COMMUNICATION
#define SERIAL_BAUDRATE 115200
#define GPS_BAUDRATE 19200
#define COMMUNICATION_TIMEOUT 150

//MOTOR 26 13 33 32 19
#define MOTOR_B_STEP_PIN 26
#define MOTOR_B_DIR_PIN 33
#define MOTOR_A_STEP_PIN 19
#define MOTOR_A_DIR_PIN 32

#define MOTOR_STEPS_PER_REV 200
#define MOTOR_DIAMETER 35
#define WHEEL_DIAMETER 20
#define MOTOR_ANGLE_MIN -90
#define MOTOR_ANGLE_MAX 90
#define MOTOR_MAX_SPEED 250
#define MOTOR_ACCELERATION 600
#define MOTOR_MIN_PULSE_WIDTH 100

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