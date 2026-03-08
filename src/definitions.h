#ifndef DEFINES_H
#define DEFINES_H

#define SIMULATION_ENABLED true

//GPS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float accuracy;
} gps_data_t;

//Motor
#define MOTOR_LEFT_A 23
#define MOTOR_LEFT_B 19
#define MOTOR_RIGHT_A 13
#define MOTOR_RIGHT_B 27
#define MOTOR_MAIN 18

//Battery
#define BATTERY_LEVEL_PIN 36

//IMU
#define GYRO_SDA_PIN 21
#define GYRO_SCL_PIN 22

typedef struct {
  float acc_x;
  float acc_y;
  float acc_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;
} IMU_data_t;

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

#define GPS_HEADING_CORRECTION_FACTOR 0.02f
#define GPS_HEADING_CORRECTION_ANGLE 2.5f
#define GPS_MIN_DIST_BETWEEN_READINGS 6

//DAC
#define DAC_MAX_VALUE 4096
#define DAC_MAX_BITS 12

#define BATTERY_LEVEL_LOW 8
#define BATTERY_LEVEL_CHARGED 95

//Sensors
#define MAIN_UPDATE_FREQUENCY 100
#define MOTION_UPDATE_FREQUENCY 10
#define BATTERY_UPDATE_FREQUENCY 10
#define IMU_UPDATE_FREQUENCY 10
#define SENSORS_INTERFACE_UPDATE_FREQUENCY 25
#define MOTOR_UPDATE_FREQUENCY 1000
#define GPS_UPDATE_FREQUENCY 5

//COMMUNICATION
#define SERIAL_BAUDRATE 115200
#define GPS_BAUDRATE 19200
#define COMMUNICATION_TIMEOUT 150

//MOTOR AND ODOMETRY
#define MOTOR_B_STEP_PIN 26
#define MOTOR_B_DIR_PIN 33
#define MOTOR_A_STEP_PIN 19
#define MOTOR_A_DIR_PIN 32

#define MOTOR_STEPS_PER_REV 200
#define WHEEL_RADIUS 0.1f
#define WHEEL_BASE 0.5f

#define MOTOR_MAX_SPEED 250
#define MOTOR_ACCELERATION 600

enum motor_instruction {
  MOTOR_STOP,
  MOTOR_MAIN_ON,
  MOTOR_MAIN_OFF,
  MOTOR_MOVE,
};

typedef struct {
  char instruction;
  float left_speed_m_per_s;
  float right_speed_m_per_s;
} motor_data_t;

typedef struct {
    float x;
    float y;
    float theta;
    float linear_vel;
    float angular_vel;
} odom_data_t;

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
    ALGORITHM_FAILED,
};

#endif