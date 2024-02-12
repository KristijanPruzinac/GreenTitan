#ifndef DEFINES_H
#define DEFINES_H

//Motor
#define MOTOR_LEFT_A 23
#define MOTOR_LEFT_B 22
#define MOTOR_RIGHT_A 21
#define MOTOR_RIGHT_B 19
#define MOTOR_MAIN 18

//Battery
#define BATTERY_LEVEL_PIN 36
#define BATTERY_LEVEL_LOW 8

//Constants
#define LEFT 0
#define RIGHT 1

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