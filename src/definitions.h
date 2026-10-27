#ifndef DEFINES_H
#define DEFINES_H

#define SIMULATION_ENABLED false
#define MOTION_TELEPORT_MODE      false
#define HARDCODED_DATUM_AND_PATH false
#define PRINT_BT_COMMANDS false

// Hardware-in-the-loop: sensors and motors talk to the simulator over micro-ROS (env esp32dev_hil)
#ifndef HIL_ENABLED
#define HIL_ENABLED 0
#endif
// Mirror the debug console over micro-ROS (esp/log, esp/console)
#ifndef DEBUG_OVER_ROS
#define DEBUG_OVER_ROS 0
#endif

#define MOTION_TELEPORT_DELAY_MS  1000

//DEBUG
#define DEBUG_RX_PIN 13
#define DEBUG_TX_PIN 27

//GPS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

typedef struct {
    double latitude;          // deg
    double longitude;         // deg
    double altitude;          // meters (MSL)
    double accuracy;          // horizontal accuracy (meters)
    double heading;           // radians (0 = east, increasing north)
    double heading_accuracy;  // radians
    double speed;             // meters per second (2D ground speed) – optional, keep for compatibility
    double speed_accuracy;    // meters per second
    double vel_north;         // m/s (positive = north)
    double vel_east;          // m/s (positive = east)
    double vel_accuracy;      // m/s (same for both axes, from sAcc)
    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
} gps_data_t;

//Motor
#define MOTOR_MAIN 18

//Battery
#define BATTERY_LEVEL_PIN 36

//IMU
#define GYRO_SDA_PIN 21
#define GYRO_SCL_PIN 22

#define GYRO_DEADBAND 0.04f
#define GNSS_ANTENNA_BEHIND_AXLE 0.180f   // m, antenna (base_link) behind the wheel axle, Mower V3 CAD

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
} IMU_data_t;

//Rain sensor
#define RAIN_SENSOR_PIN 39

//CONSTANTS
#define LEFT 0
#define RIGHT 1
#define FORWARD 1
#define BACKWARD 0
#define MILLIS_PER_SECOND 1000

//Motion (robot description for the navigation core, src/navigation; see ARCHITECTURE §6)
#define MOTION_FORWARD_SPEED_NORMAL  0.3f   // 0.5 m/s was unstable with >0.3 s pose latency (2026-10-27)
// Path acceleration and braking. Stays under the motor ramp (MOTOR_ACCELERATION ~0.35 m/s^2),
// so the wheels follow the planned speed and the mower stops where the path does.
#define MOTION_GOAL_DECEL                 0.25f  // m/s^2
#define MOTION_GOAL_MIN_SPEED             0.05f  // m/s, so the last centimetres still close
// Corners: rounded into axle arcs of this radius where the legs leave room (the blade, 0.18 m
// behind, runs just outside them); tighter or sharper corners turn on the spot.
#define MOTION_CORNER_RADIUS              0.30f  // m
#define MOTION_MIN_CORNER_RADIUS          0.22f  // m
#define MOTION_MAX_ROUNDED_TURN           2.6f   // rad, ~150 deg
// Mower body around the wheel axle centre (Mower V3 CAD, x forward = big wheels)
#define MOWER_FRONT_OVERHANG              0.12f  // m, cover ahead of the axle
#define MOWER_REAR_OVERHANG               0.415f // m, casters behind it
#define MOWER_HALF_WIDTH                  0.20f  // m, outside of the drive wheels
// Hard limit: the planned (axle) path keeps half the width plus this from the outline and from
// the inner outlines. Turns swing the overhangs further out; the motion task warns where.
#define MOWER_EDGE_CLEARANCE              0.05f  // m
#define MOWER_EDGE_INSET                  (MOWER_HALF_WIDTH + MOWER_EDGE_CLEARANCE)
// Infill rows and their turns keep this from every outline: there the body can turn on the spot
// in any direction (it sweeps a circle of its farthest corner) and stays inside the clearance.
// The strip between the first outline lap and it is mowed by further laps.
#define MOWER_TURN_INSET                  (fmaxf(hypotf(MOWER_REAR_OVERHANG, MOWER_HALF_WIDTH), \
                                                 hypotf(MOWER_FRONT_OVERHANG, MOWER_HALF_WIDTH)) + MOWER_EDGE_CLEARANCE)
// Corners along an edge are driven so that the rear swings out at most this past the half
// width; the rest of the clearance is for tracking error
#define MOTION_EDGE_SWING                 0.02f  // m
#define MOTION_PATH_TOLERANCE             0.03f  // m, near-straight waypoints merged within this
// Turning on the spot (path start, sharp corners, or far off the path heading)
#define MOTION_MAX_TRACK_HEADING_ERROR    1.0f   // rad, ~57 deg: beyond this turn on the spot first
#define MOTION_TURN_DONE                  0.05f  // rad, ~3 deg: then hand over to the tracker
#define MOTION_ROTATE_GAIN                1.5f   // rad/s per rad of heading error
#define CONTROL_YAW_TAU                   1.0f   // s, control heading follows the EKF yaw this slowly, gyro in between
#define CONTROL_YAW_SNAP                  0.5f   // rad, ~30 deg: further off, take the EKF yaw directly
#define FUSED_YAW_LAG                     0.15   // s, EKF yaw behind its own stamp (HIL 2026-10-27; measure on the robot)
#define MOTION_POSE_TIMEOUT_MS            500    // stop driving if /fused_pose (10 Hz) goes quiet
#define MOTION_STALL_DIST                0.10f  // m: driving forward but moved less than this ...
#define MOTION_STALL_TIME_MS              4000   // ... for this long = stalled (obstacle signal)
#define MOTION_ROTATE_TIMEOUT_MS          10000  // rotate-only longer than this = stalled (180 deg ~3.5 s)

//Datum capture
#define DATUM_CAPTURE_DRIVE_DURATION_MS  800
#define DATUM_CAPTURE_DRIVE_SPEED        0.3f
#define DATUM_CAPTURE_MIN_DISPLACEMENT_M 0.0f
#define DATUM_BASE_EXIT_DISTANCE_CM      50

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
#define MOTOR_MICROSTEPS 1      // must match the DM542T DIP setting
#define MOTOR_GEAR_RATIO 10     // MG23-G10
#define MOTOR_STEPS_PER_WHEEL_REV (MOTOR_STEPS_PER_REV * MOTOR_MICROSTEPS * MOTOR_GEAR_RATIO)
#define WHEEL_RADIUS 0.0749f    // Mower V3 CAD, not yet measured
#define WHEEL_BASE 0.3647f      // Mower V3 CAD track, not yet measured
#define MAX_LINEAR_VEL  0.5f
#define MAX_ANGULAR_VEL 1.0f

#define MOTOR_MAX_SPEED 200
#define MOTOR_ACCELERATION 1500 // steps/s^2 at the motor, ~0.35 m/s^2 at the wheel (150 before the 10:1 gearbox)

// Manual mode
#define MANUAL_LINEAR_SPEED   0.3f
#define MANUAL_ANGULAR_SPEED  0.6f
#define MANUAL_WATCHDOG_MS    500

// Shared manual-mode state (defined in main.cpp, written only by controller_task)
extern volatile bool     MANUAL_MODE_ACTIVE;
extern volatile uint32_t LAST_MANUAL_MOVE_MS;
extern volatile bool     MANUAL_MOTORS_STOPPED;

// Shared manual-mode state (defined in main.cpp, written only by controller_task)
extern volatile bool     MANUAL_MODE_ACTIVE;
extern volatile uint32_t LAST_MANUAL_MOVE_MS;
extern volatile bool     MANUAL_MOTORS_STOPPED;

// Shared datum-capture state (defined in main.cpp, written only by controller_task)
extern volatile bool     DATUM_CAPTURE_ACTIVE;
extern volatile int      LAST_DATUM_CAPTURE_RESULT;

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

// Fused pose from ROS2 EKF
typedef struct {
    double x;
    double y;
    float yaw;
    float vx;
    float omega;
} fused_pose_data_t;

enum motion_mode {
    WAITING,          // stop and drop the queued path
    MOVING,           // append end_x/end_y to the path (start_x/y unused: the path starts at the robot)
    MOVING_REVERSE,
    MOTION_PAUSE,     // stop, keep the queued path
    MOTION_RESUME,
};

typedef struct {
    int mode;
    double start_x;
    double start_y;
    double end_x;
    double end_y;
    uint8_t tool;     // nav_tool (navigation/nav_path.h): blade on the way to this point; 0 = unchanged
    int8_t edge;      // the way to this point runs along an outline on this side: +1 left, -1 right, 0 none
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
    ROBOT_STATE_CAPTURING_DATUM,
};

enum controller_signal {
    SIGNAL_MOTION_DONE,         // stopped at the end of the queued path
    SIGNAL_MOTION_WANT_POINT,   // room in the motion queue for the next planner point
    SIGNAL_LOW_BATTERY,
    SIGNAL_RAIN,
    SIGNAL_OBSTACLE,
    SIGNAL_MAIN_CHARGING_START,
    SIGNAL_BATTERY_CHARGED,
    SIGNAL_START_MOWING,
    SIGNAL_MANUAL_ON,
    SIGNAL_MANUAL_OFF,
    SIGNAL_START_DATUM_CAPTURE,
    SIGNAL_STOP_MOWING,
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

enum datum_capture_result {
    DATUM_RESULT_NONE = 0,           // no capture completed yet, or already consumed by BT
    DATUM_RESULT_OK,
    DATUM_RESULT_FAIL_STATE,         // controller wasn't IDLE
    DATUM_RESULT_FAIL_ACCURACY,      // GPS accuracy degraded during drive
    DATUM_RESULT_FAIL_DISPLACEMENT,  // robot didn't move far enough
};

#endif