#ifndef DEFINES_H
#define DEFINES_H

#define SIMULATION_ENABLED true
#define MOTION_TELEPORT_MODE      false
#define MOTION_TELEPORT_DELAY_MS  1000

//DEBUG
#define DEBUG_RX_PIN 13
#define DEBUG_TX_PIN 27

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
    double latitude;
    double longitude;
    float altitude;
    float accuracy;
} gps_data_t;

//Motor
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

//Motion
#define MOTION_CORRECTION_GAIN 0.3f

#define MOTION_FORWARD_SPEED_NORMAL  0.5f
#define MOTION_GOAL_SLOW_DOWN_DISTANCE    0.15f

// Speed scaling based on heading error
#define MOTION_HEADING_ERROR_FULL_SPEED   0.1f   // ~6° - full speed below
#define MOTION_HEADING_ERROR_MIN_SPEED    0.26f  // ~15° - 50% speed here
#define MOTION_MIN_SPEED_SCALE            0.5f
#define MOTION_HEADING_ERROR_ROTATE_ONLY  0.52f  // ~30° - rotate only above

//DAC
#define DAC_MAX_VALUE 4096
#define DAC_MAX_BITS 12

#define BATTERY_LEVEL_LOW 8
#define BATTERY_LEVEL_CHARGED 95

//Sensors
#define MOTION_UPDATE_FREQUENCY 10
#define BATTERY_UPDATE_FREQUENCY 10
#define IMU_UPDATE_FREQUENCY 10
#define MOTOR_UPDATE_FREQUENCY 1000
#define GPS_UPDATE_FREQUENCY 5

//COMMUNICATION
#define SERIAL_BAUDRATE 921600
#define DEBUG_BAUDRATE 115200
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
#define MAX_LINEAR_VEL  0.5f
#define MAX_ANGULAR_VEL 1.0f

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
  float linear_vel;
  float angular_vel;
} motor_data_t;

typedef struct {
    double x;
    double y;
    float theta;
    float linear_vel;
    float angular_vel;
} odom_data_t;

// Fused pose from ROS2 EKF
typedef struct {
    double x;
    double y;
    float yaw;
    float vx;
    float omega;
} fused_pose_data_t;

enum motion_mode {
    WAITING,
    MOVING,
    MOVING_REVERSE,
};

typedef struct {
    int mode;
    double start_x;
    double start_y;
    double end_x;
    double end_y;
} motion_command_t;

enum robot_state {
    ROBOT_STATE_IDLE,
    ROBOT_STATE_MOWING,
    ROBOT_STATE_PAUSED,
    ROBOT_STATE_RETURNING_TO_BASE,
    ROBOT_STATE_ENTERING_CHARGING_STATION,
    ROBOT_STATE_CHARGING,
    ROBOT_STATE_LEAVING_CHARGING_STATION,
    ROBOT_STATE_RECORDING_OUTLINE,
    ROBOT_STATE_MANUAL,
};

enum controller_signal {
    SIGNAL_MOTION_DONE,
    SIGNAL_LOW_BATTERY,
    SIGNAL_RAIN,
    SIGNAL_OBSTACLE,
    SIGNAL_MAIN_CHARGING_START,
    SIGNAL_BATTERY_CHARGED,
    SIGNAL_START_MOWING,
    SIGNAL_MANUAL_ON,
    SIGNAL_MANUAL_OFF,
};

typedef struct {
    int signal;
} controller_signal_t;

typedef struct {
    double x;
    double y;
} point_t;

enum algorithm_command {
    CMD_ALGO_START_MOWING,        // Start full mowing algorithm
    CMD_ALGO_PAUSE,               // Pause algorithm
    CMD_ALGO_RESUME,              // Resume algorithm
    CMD_ALGO_ABORT,               // Abort current operation (full or line only)
    CMD_ALGO_START_RECORDING,     // Start recording outline
    CMD_ALGO_CAPTURE_BASE,        // Capture charging station location
    CMD_ALGO_CAPTURE_BASE_EXIT,   // Capture base exit point
    CMD_ALGO_NEW_OUTLINE,         // Start new outline
    CMD_ALGO_CAPTURE_POINT,       // Capture current position as point
    CMD_ALGO_REMOVE_OUTLINE,      // Remove last outline
    CMD_ALGO_REMOVE_POINT,        // Remove last point
    CMD_ALGO_END_RECORDING,       // Finish recording and generate paths
    CMD_ALGO_CLEAR_ALL,           // Clear all outlines
    CMD_ALGO_GET_PATH_STRING,     // Request path string (for saving)
    CMD_ALGO_LOAD_PATH_STRING,    // Load path from string
};

typedef struct {
    int command;
    bool full_abort;              // Used with CMD_ALGO_ABORT (true = full abort, false = skip line)
    double lon;                   // Used with manual point capture
    double lat;                   // Used with manual point capture
    String* path_data;            // Used with CMD_ALGO_LOAD_PATH_STRING
} algorithm_command_t;

typedef struct {
    int signal;
    String* path_data;            // Used with SIGNAL_ALGO_PATH_STRING
} algorithm_signal_t;

// -------------------------------------------------------- DDS SIGNALS ---------------------------------------------------------------

enum algorithm_signal {
    SIGNAL_ALGO_PATH_READY,       // Path generation complete
    SIGNAL_ALGO_PATH_FAILED,      // Path generation failed
    SIGNAL_ALGO_MOWING_COMPLETE,  // All mowing finished
    SIGNAL_ALGO_POINT_ADDED,      // Point successfully added
    SIGNAL_ALGO_OUTLINE_ADDED,    // New outline created
    SIGNAL_ALGO_RECORDING_ENDED,  // Recording finished
    SIGNAL_ALGO_PATH_STRING,      // Response with path string data
};

typedef struct {
    int outline_index;
    int point_index;
    double x;
    double y;
} intersection_point_t;

typedef struct {
    double latitude;
    double longitude;
} datum_data_t;

typedef struct {
    double x;
    double y;
    float theta;
} sim_pose_set_t;

#endif