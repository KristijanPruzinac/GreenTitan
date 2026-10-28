#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <geometry_msgs/msg/quaternion.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#include "esp_dds.h"
#include "esp_timer.h"

#include "definitions.h"
#include "debug/debug_serial.h"
#include "hil/hil.h"

// -------------------------------------------------------- CONFIGURATION ---------------------------------------------------------------

// Enable or disable peripherals for testing
bool ENABLE_MOTORS = true;
bool ENABLE_IMU = true;
bool ENABLE_GPS = true;

bool ENABLE_MOTION = true;

// -------------------------------------------------------- GLOBALS ---------------------------------------------------------------
DebugSerial SerialDebug(1);

volatile bool TASKS_STARTED = false;

bool SETUP_COMPLETED = true;

double BASE_LON = 0;
double BASE_LAT = 0;
int64_t BASE_EXIT_X_CM = 0;
int64_t BASE_EXIT_Y_CM = 0;

int GPS_ACC_THRESHOLD = 3000; //TODO: Revert to 18
int GPS_STABILITY_CHECK_DURATION_SECONDS = 15; // 5 minutes TODO: Revert to 5 minutes

float MOTION_ACC_FACTOR = 1.0;

int MOWER_OVERLAP = 15;
int MAX_DEVIATION = 50;

float BATTERY_LEVEL_MIN = 16;
float BATTERY_LEVEL_MAX = 18;

bool CONFIG_PATH = false;
bool CONFIG_BATTERY = false;
bool CONFIG_MOTORS = false;
bool CONFIG_GYRO = false;
bool CONFIG_RAIN_SENSOR = false;
bool CONFIG_DATUM = false;

bool IMU_INVERT = true;


volatile bool     MANUAL_MODE_ACTIVE    = false;
volatile uint32_t LAST_MANUAL_MOVE_MS   = 0;
volatile bool     MANUAL_MOTORS_STOPPED = true;

volatile bool     DATUM_CAPTURE_ACTIVE       = false;
volatile int      LAST_DATUM_CAPTURE_RESULT  = DATUM_RESULT_NONE;

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

// Sensors
#include "sensor/imu_task.h"
#include "sensor/gps_task.h"

// Effectors
#include "motor/motor_task.h"

//Motion
#include "motion/motion_task.h"

//Controller
#include "controller/controller_task.h"

// Configuration
#include "configuration/configuration.h"

// Visualization
#include "visualization/path_markers.h"

// Simulation
#include "simulation/sim_task.h"

// -------------------------------------------------------- Micro-ROS ------------------------------------------------------------------
rcl_publisher_t imu_publisher;
rcl_publisher_t gps_publisher;
rcl_subscription_t fused_pose_subscriber;

rcl_publisher_t datum_publisher;
rcl_publisher_t path_data_publisher;

sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry gps_msg;
// EKF pose from fused_pose_relay (robot_package): [stamp_sec, stamp_nanosec, x, y, yaw, yaw_var, vx, vy, omega]
std_msgs__msg__Float64MultiArray fused_pose_msg;
static double fused_pose_buf[9];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// String buffers for message frame_id fields (micro-ROS needs preallocated storage)
char gps_frame_id_buf[16];
char gps_child_frame_id_buf[16];
char imu_frame_id_buf[16];

void fused_pose_callback(const void* msgin);
void hil_ros_init();
void hil_ros_tick();

static bool pending_datum_publish = false;
static int datum_publish_count = 0;
static bool datum_published = false;

bool pending_path_publish = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

bool sync_time()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously

    if (RMW_RET_OK != rmw_uros_sync_session(timeout_ms)) return false; // not synchronized

    if (rmw_uros_epoch_synchronized()) {
        int64_t time_ns = rmw_uros_epoch_nanos();
        timespec tp;
        tp.tv_sec = time_ns / 1000000000;
        tp.tv_nsec = time_ns % 1000000000;
        clock_settime(CLOCK_REALTIME, &tp);
        return true;
    }
    return false;
}

void init_micro_ros(){
    while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        delay(100);
    }

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // Datum publisher
    RCCHECK(rclc_publisher_init_default(
        &datum_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "esp/datum"));

    // IMU publisher: best effort like the EKF subscription. A reliable publish waits for the
    // agent's ack (one serial round-trip each), which stalled this loop.
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    // GPS publisher: stays reliable, Odometry (~700 B) doesn't fit a best-effort frame
    RCCHECK(rclc_publisher_init_default(
        &gps_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "gps/odom"));
    
    // Compact and best effort: the EKF's 724-byte Odometry does not fit the serial MTU, and the
    // agent dropped the session when it could not split it (HIL 2026-10-28). An old pose is no use.
    fused_pose_msg.data.data = fused_pose_buf;
    fused_pose_msg.data.capacity = sizeof(fused_pose_buf) / sizeof(fused_pose_buf[0]);
    fused_pose_msg.data.size = 0;
    RCCHECK(rclc_subscription_init_best_effort(
        &fused_pose_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "esp/fused_pose"));

    RCCHECK(rclc_publisher_init_default(
        &path_data_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "algo/path_data"));
    
    // GPS
    strcpy(gps_frame_id_buf, "odom");
    gps_msg.header.frame_id.data = gps_frame_id_buf;
    gps_msg.header.frame_id.capacity = sizeof(gps_frame_id_buf);
    gps_msg.header.frame_id.size = strlen(gps_frame_id_buf);
    strcpy(gps_child_frame_id_buf, "base_link");
    gps_msg.child_frame_id.data = gps_child_frame_id_buf;
    gps_msg.child_frame_id.capacity = sizeof(gps_child_frame_id_buf);
    gps_msg.child_frame_id.size = strlen(gps_child_frame_id_buf);

    // IMU
    strcpy(imu_frame_id_buf, "imu_link");
    imu_msg.header.frame_id.data = imu_frame_id_buf;
    imu_msg.header.frame_id.capacity = sizeof(imu_frame_id_buf);
    imu_msg.header.frame_id.size = strlen(imu_frame_id_buf);

    // create executor
    const size_t hil_handles = (HIL_ENABLED ? 2 : 0) + (DEBUG_OVER_ROS ? 1 : 0);
    RCCHECK(rclc_executor_init(&executor, &support.context, 2 + hil_handles, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &fused_pose_subscriber,
        &fused_pose_msg,
        &fused_pose_callback,
        ON_NEW_DATA
    ));

    hil_ros_init();

    bool success = sync_time();
    if (!success) {
        error();
    }
}

void publish_datum() {
    if (!CONFIG_DATUM) return;

    sensor_msgs__msg__NavSatFix datum_msg = {};
    datum_msg.latitude  = BASE_LAT;
    datum_msg.longitude = BASE_LON;
    datum_msg.altitude  = 0.0;
    datum_msg.status.status  = 0;
    datum_msg.status.service = 1;

    RCSOFTCHECK(rcl_publish(&datum_publisher, &datum_msg, NULL));
    SerialDebug.printf("Datum published: lat=%.7f lon=%.7f\r\n", BASE_LAT, BASE_LON);
}

void datum_set_callback(dds_callback_context_t* context) {
    datum_data_t* data = (datum_data_t*)context->message_data.data;
    BASE_LAT = data->latitude;
    BASE_LON = data->longitude;
    pending_datum_publish = true;
    datum_publish_count = 0;
}

// Recent gyro z samples (rad/s) with their stamps, so the GPS yaw can use the turn rate at the
// GPS epoch (the fix arrives ~0.2 s late). 16 samples at 10 Hz cover the EKF's 1 s history.
#define GYRO_HISTORY 16
static struct { double t; float gz; } gyro_hist[GYRO_HISTORY];
static int gyro_hist_n = 0, gyro_hist_head = 0;   // head = next slot to write

// Gyro z at time t, interpolated. *spread = change between the two samples around t: how far the
// true rate can be off there (0 in a steady turn, large where a turn starts or stops).
static float gyro_z_at(double t, float* spread) {
    *spread = 0.0f;
    if (gyro_hist_n == 0) return 0.0f;
    int newest = (gyro_hist_head + GYRO_HISTORY - 1) % GYRO_HISTORY;
    int prev = (newest + GYRO_HISTORY - 1) % GYRO_HISTORY;
    if (t >= gyro_hist[newest].t) {
        if (gyro_hist_n > 1) *spread = fabsf(gyro_hist[newest].gz - gyro_hist[prev].gz);
        return gyro_hist[newest].gz;
    }
    for (int k = 1; k < gyro_hist_n; k++) {
        int a = (newest + GYRO_HISTORY - k) % GYRO_HISTORY;      // older sample
        int b = (a + 1) % GYRO_HISTORY;
        if (t >= gyro_hist[a].t) {
            double f = (t - gyro_hist[a].t) / (gyro_hist[b].t - gyro_hist[a].t);
            *spread = fabsf(gyro_hist[b].gz - gyro_hist[a].gz);
            return gyro_hist[a].gz + f * (gyro_hist[b].gz - gyro_hist[a].gz);
        }
    }
    return gyro_hist[(gyro_hist_head + GYRO_HISTORY - gyro_hist_n) % GYRO_HISTORY].gz;   // oldest
}

// Yaw turned between t0 and t1 by the local gyro (trapezoid over the history, newest held after it)
static double gyro_turn_between(double t0, double t1) {
    const double step = 0.02;
    double turn = 0.0, s;
    float spread, prev = gyro_z_at(t0, &spread);
    for (double t = t0; t < t1; t += s) {
        s = fmin(step, t1 - t);
        float g = gyro_z_at(t + s, &spread);
        turn += 0.5 * (prev + g) * s;
        prev = g;
    }
    return turn;
}

void imu_topic_callback(dds_callback_context_t* context) {
    IMU_data_t* data = (IMU_data_t*)context->message_data.data;

    if (!datum_published) return;

    imu_msg.header.stamp.sec = data->timestamp_sec;
    imu_msg.header.stamp.nanosec = data->timestamp_nsec;

    imu_msg.linear_acceleration.x = data->acc_x;
    imu_msg.linear_acceleration.y = data->acc_y;
    imu_msg.linear_acceleration.z = data->acc_z;

    imu_msg.angular_velocity.x = data->gyro_x;
    imu_msg.angular_velocity.y = data->gyro_y;
    imu_msg.angular_velocity.z = data->gyro_z;

    // MPU6050 can't give absolute orientation - tell EKF to ignore it
    imu_msg.orientation_covariance[0] = -1;

    // Covariances - diagonal elements only (row-major 3x3 matrix)
    imu_msg.angular_velocity_covariance[0] = 0.01f;
    imu_msg.angular_velocity_covariance[4] = 0.01f;
    imu_msg.angular_velocity_covariance[8] = 0.01f;

    imu_msg.linear_acceleration_covariance[0] = 0.1f;
    imu_msg.linear_acceleration_covariance[4] = 0.1f;
    imu_msg.linear_acceleration_covariance[8] = 0.1f;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    double t = data->timestamp_sec + data->timestamp_nsec * 1e-9;
    int newest = (gyro_hist_head + GYRO_HISTORY - 1) % GYRO_HISTORY;
    if (gyro_hist_n == 0 || t > gyro_hist[newest].t) {           // keep stamps strictly increasing
        gyro_hist[gyro_hist_head].t = t;
        gyro_hist[gyro_hist_head].gz = data->gyro_z;
        gyro_hist_head = (gyro_hist_head + 1) % GYRO_HISTORY;
        if (gyro_hist_n < GYRO_HISTORY) gyro_hist_n++;
    }
}

static float fused_yaw_for_twist = 0.0f;   // latest EKF yaw; both callbacks run in the loop task
static double fused_yaw_stamp = 0.0;       // its stamp (s)
static double fused_yaw_var = 1e6;         // its variance from the EKF; large = yaw not known yet
static double fused_x_last = 0.0, fused_y_last = 0.0;   // latest EKF position (console line test)
static bool fused_pose_seen = false;

void gps_topic_callback(dds_callback_context_t* context) {
    gps_data_t* data = (gps_data_t*)context->message_data.data;

    if (!datum_published) return;

    // Convert lat/lon to local meters relative to datum
    const double METERS_PER_LAT = 111111.0;
    double y_north = (data->latitude  - BASE_LAT) * METERS_PER_LAT;
    double x_east  = (data->longitude - BASE_LON) * METERS_PER_LAT * cos(BASE_LAT * M_PI / 180.0);

    // Header timestamp
    gps_msg.header.stamp.sec     = data->timestamp_sec;
    gps_msg.header.stamp.nanosec = data->timestamp_nsec;

    // Pose: position in odom frame
    gps_msg.pose.pose.position.x = x_east;
    gps_msg.pose.pose.position.y = y_north;
    gps_msg.pose.pose.position.z = 0.0;

    // Pose covariance (6x6, row-major: x, y, z, roll, pitch, yaw)
    // Only fill diagonals we care about; leave the rest at zero.
    double pos_var = (double)data->accuracy * (double)data->accuracy;
    gps_msg.pose.covariance[0]  = pos_var;      // x
    gps_msg.pose.covariance[7]  = pos_var;      // y
    gps_msg.pose.covariance[14] = 1e6;          // z — huge, we don't care
    gps_msg.pose.covariance[21] = 1e6;          // roll
    gps_msg.pose.covariance[28] = 1e6;          // pitch

    // Yaw from the GPS velocity vector (2026-10-27). A diff-drive mower can't move sideways, so
    // the wheel-axle velocity points along the body, forwards or backwards:
    // - The antenna (base_link) sits GNSS_ANTENNA_BEHIND_AXLE behind the axle, so in a turn it
    //   also moves sideways by w x r. That part is removed with the gyro rate at the GPS epoch;
    //   the current rate is wrong whenever a turn starts or stops within the fix's latency.
    // - Forwards or backwards is the sign of the wheel step rates (a sign only, never odometry).
    //   Wheels stopped (coasting, a bounce after braking): nothing is fused, the gyro holds the
    //   heading. Not the yaw estimate: a flipped estimate would then confirm itself for good.
    //   Wheels and motion disagreeing (pushed back, a forward->reverse change) fail the gate below.
    // - The yaw variance is what the velocity uncertainty gives, (sigma / speed)^2, so slow driving
    //   counts little. sigma combines the GPS sAcc and the lever-arm error from the gyro rate
    //   changing between samples. Below 3 sigma the direction is noise and nothing is fused.
    float cy = cosf(fused_yaw_for_twist), sy = sinf(fused_yaw_for_twist);
    float gyro_spread;
    double lever = -GNSS_ANTENNA_BEHIND_AXLE *                          // body-left speed of the antenna
                   gyro_z_at(data->timestamp_sec + data->timestamp_nsec * 1e-9, &gyro_spread);
    double lever_err = GNSS_ANTENNA_BEHIND_AXLE * gyro_spread;
    double axle_e = data->vel_east  + sy * lever;
    double axle_n = data->vel_north - cy * lever;
    double axle_speed = hypot(axle_e, axle_n);
    double s_acc = data->vel_accuracy;
    double sigma = hypot(s_acc, lever_err);
    double yaw_var = 1e6;                                              // ignore yaw measurement
    double yaw_meas = fused_yaw_for_twist;
    if (s_acc > 0 && axle_speed > 3.0 * sigma) {
        double travel = atan2(axle_n, axle_e);
        int dir = 0;                                                   // +1 forwards, -1 backwards
        float left, right;
        motor_get_step_rates(&left, &right);                           // ramped rates: where the wheels turn now
        if (left + right > 0) dir = 1; else if (left + right < 0) dir = -1;
        if (dir != 0) {
            yaw_meas = normalize_angle(dir > 0 ? travel : travel + M_PI);
            yaw_var = (sigma / axle_speed) * (sigma / axle_speed);
            // Outlier gate: the axle can still slide sideways (caster drag, wheel slip in a pivot).
            // A yaw more than 3 sigma of the combined uncertainty away from the EKF's yaw at the
            // GPS epoch is not fused. At boot the EKF's yaw variance is huge, so the first lock passes.
            // The EKF's yaw is its state FUSED_YAW_LAG before its stamp; the gyro carries it to the epoch.
            double gps_t = data->timestamp_sec + data->timestamp_nsec * 1e-9;
            double fused_t = fused_yaw_stamp - FUSED_YAW_LAG;
            double yaw_then = fused_yaw_for_twist + (gps_t >= fused_t ? gyro_turn_between(fused_t, gps_t)
                                                                      : -gyro_turn_between(gps_t, fused_t));
            double innov = angle_diff(yaw_meas, yaw_then);
            if (innov * innov > 9.0 * (yaw_var + fused_yaw_var)) yaw_var = 1e6;
        }
    }
    gps_msg.pose.covariance[35] = yaw_var;
    double half = yaw_meas * 0.5;
    gps_msg.pose.pose.orientation.x = 0.0;
    gps_msg.pose.pose.orientation.y = 0.0;
    gps_msg.pose.pose.orientation.z = sin(half);
    gps_msg.pose.pose.orientation.w = cos(half);

    // Twist: robot_localization reads it in child_frame_id (base_link), so the GPS east/north
    // velocity is rotated into the body frame with the current fused yaw. The EKF rotates it
    // back with the same yaw, so its position prediction follows the GPS velocity even when
    // that yaw is off; yaw itself is not corrected by this. (Until 2026-10-27 east/north went
    // in unrotated, so between fixes the EKF predicted motion in the wrong direction whenever
    // yaw was not ~0: at yaw 90 deg it moved west while the mower drove north.)
    gps_msg.twist.twist.linear.x =  cy * data->vel_east + sy * data->vel_north;   // forward
    gps_msg.twist.twist.linear.y = -sy * data->vel_east + cy * data->vel_north;   // left
    gps_msg.twist.twist.linear.z = 0.0;
    gps_msg.twist.twist.angular.x = 0.0;
    gps_msg.twist.twist.angular.y = 0.0;
    gps_msg.twist.twist.angular.z = 0.0;

    double vel_var = (double)data->vel_accuracy * (double)data->vel_accuracy;
    if (vel_var < 1e-6) vel_var = 0.01;  // floor — never zero
    gps_msg.twist.covariance[0]  = vel_var;
    gps_msg.twist.covariance[7]  = vel_var;
    gps_msg.twist.covariance[14] = 1e6;
    gps_msg.twist.covariance[21] = 1e6;
    gps_msg.twist.covariance[28] = 1e6;
    gps_msg.twist.covariance[35] = 1e6;

    RCSOFTCHECK(rcl_publish(&gps_publisher, &gps_msg, NULL));
}

void fused_pose_callback(const void* msgin) {
    const std_msgs__msg__Float64MultiArray* msg = (const std_msgs__msg__Float64MultiArray*)msgin;
    if (msg->data.size < 9) return;
    const double* d = msg->data.data;
    double stamp_sec = d[0], stamp_nsec = d[1];
    double x = d[2];
    double y = d[3];
    float yaw = (float)d[4];
    fused_yaw_for_twist = yaw;
    fused_yaw_stamp = stamp_sec + stamp_nsec * 1e-9;
    fused_yaw_var = d[5];
    fused_x_last = x;
    fused_y_last = y;
    fused_pose_seen = true;

    // The pose is the EKF state at its stamp, which is already old when it gets here (~0.06 s
    // in HIL). Motion control needs the pose now, so predict it forward over the measured age:
    // position with the EKF's GPS velocity (no wheel odometry), yaw with the local gyro samples.
    // The EKF's yaw also trails its own stamp by FUSED_YAW_LAG (its filtered yaw rate lags the
    // gyro), so the gyro turn starts that much earlier. Uncorrected, the ~0.3 s total made the
    // tracker weave +-20 cm in HIL (2026-10-27). The clock is synced at boot.
    float vx = (float)d[6], vy = (float)d[7];
    float omega = (float)d[8];   // passed on, not used for the prediction
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    double age = (now.tv_sec - stamp_sec) + (now.tv_nsec - stamp_nsec) * 1e-9;
    if (age > 0 && age < MOTION_POSE_TIMEOUT_MS / 1000.0) {           // otherwise the clock is off: no prediction
        float turn = (float)gyro_turn_between(fused_yaw_stamp - FUSED_YAW_LAG, fused_yaw_stamp + age);
        yaw += turn;
        float mid = yaw - 0.5f * (float)gyro_turn_between(fused_yaw_stamp, fused_yaw_stamp + age);
        x += (cosf(mid) * vx - sinf(mid) * vy) * age;   // along the heading halfway through
        y += (sinf(mid) * vx + cosf(mid) * vy) * age;
    }

    // Control heading: the local gyro carries it, the EKF yaw pulls it in with time constant
    // CONTROL_YAW_TAU. Each 5 Hz GPS heading fix moves the EKF yaw by up to ~5 deg and the
    // tracker chased every jump (HIL 2026-10-27); this keeps the EKF as the reference without them.
    double now_t = now.tv_sec + now.tv_nsec * 1e-9;
    static double control_yaw_t = 0.0;
    static float control_yaw = 0.0f;
    if (control_yaw_t == 0.0 || now_t - control_yaw_t > 1.0 ||
        fabsf(angle_diff(yaw, control_yaw)) > CONTROL_YAW_SNAP) {
        control_yaw = yaw;   // first pose, a gap, or far off: take the EKF as it is
    } else {
        control_yaw += (float)gyro_turn_between(control_yaw_t, now_t);
        float k = fminf(1.0f, (float)(now_t - control_yaw_t) / CONTROL_YAW_TAU);
        control_yaw += k * angle_diff(control_yaw, yaw);
    }
    control_yaw = normalize_angle(control_yaw);
    control_yaw_t = now_t;
    yaw = control_yaw;

    // Package and publish internally via your DDS system
    fused_pose_data_t data = {
        .x     = x,
        .y     = y,
        .yaw   = normalize_angle(yaw),
        .vx    = vx,
        .omega = omega,
    };

    if (TASKS_STARTED) {
        dds_result_t result = DDS_PUBLISH("/fused_pose", data);
        if (result != DDS_SUCCESS) {
            SerialDebug.printf("Fused pose publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
        }
    } else {
        SerialDebug.println("Tasks not started, skipping fused pose publish");
    }
}

// -------------------------------------------------------- HIL / ROS console --------------------------------------------------------
// Simulator link (HIL_ENABLED): GNSS bytes and IMU samples in, stepper rates out.
// Console mirror (DEBUG_OVER_ROS): SerialDebug output on esp/log, input from esp/console.
rcl_subscription_t hil_gnss_subscriber;
rcl_subscription_t hil_imu_subscriber;
rcl_subscription_t console_subscriber;
rcl_publisher_t hil_stepper_publisher;
rcl_publisher_t log_publisher;

static std_msgs__msg__UInt8MultiArray hil_gnss_msg;
static uint8_t hil_gnss_buf[256];
static sensor_msgs__msg__Imu hil_imu_msg;
static char hil_imu_frame_buf[16];
static std_msgs__msg__Float32MultiArray hil_stepper_msg;
static float hil_stepper_buf[2];
static std_msgs__msg__String console_msg;
static char console_buf[64];
static std_msgs__msg__String log_msg;
static char log_buf[400];

static void hil_gnss_callback(const void* msgin) {
    const std_msgs__msg__UInt8MultiArray* msg = (const std_msgs__msg__UInt8MultiArray*)msgin;
    hil_gnss_push(msg->data.data, msg->data.size);
}

static void hil_imu_callback(const void* msgin) {
    const sensor_msgs__msg__Imu* msg = (const sensor_msgs__msg__Imu*)msgin;
    hil_imu_sample_t sample = {
        (float)msg->linear_acceleration.x, (float)msg->linear_acceleration.y, (float)msg->linear_acceleration.z,
        (float)msg->angular_velocity.x, (float)msg->angular_velocity.y, (float)msg->angular_velocity.z,
    };
    hil_imu_set(sample);
}

static void console_callback(const void* msgin) {
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    SerialDebug.inject_input(msg->data.data, msg->data.size);
}

void hil_ros_init() {
    if (HIL_ENABLED) {
        hil_init();

        hil_gnss_msg.data.data = hil_gnss_buf;
        hil_gnss_msg.data.capacity = sizeof(hil_gnss_buf);
        hil_gnss_msg.data.size = 0;
        RCCHECK(rclc_subscription_init_default(
            &hil_gnss_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "hil/gnss_ubx"));
        RCCHECK(rclc_executor_add_subscription(&executor, &hil_gnss_subscriber, &hil_gnss_msg,
                                               &hil_gnss_callback, ON_NEW_DATA));

        hil_imu_msg.header.frame_id.data = hil_imu_frame_buf;
        hil_imu_msg.header.frame_id.capacity = sizeof(hil_imu_frame_buf);
        RCCHECK(rclc_subscription_init_best_effort(
            &hil_imu_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "hil/imu"));
        RCCHECK(rclc_executor_add_subscription(&executor, &hil_imu_subscriber, &hil_imu_msg,
                                               &hil_imu_callback, ON_NEW_DATA));

        hil_stepper_msg.data.data = hil_stepper_buf;
        hil_stepper_msg.data.capacity = 2;
        hil_stepper_msg.data.size = 2;
        RCCHECK(rclc_publisher_init_best_effort(
            &hil_stepper_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "hil/stepper_rate"));
    }
    if (DEBUG_OVER_ROS) {
        console_msg.data.data = console_buf;
        console_msg.data.capacity = sizeof(console_buf);
        RCCHECK(rclc_subscription_init_default(
            &console_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp/console"));
        RCCHECK(rclc_executor_add_subscription(&executor, &console_subscriber, &console_msg,
                                               &console_callback, ON_NEW_DATA));

        log_msg.data.data = log_buf;
        log_msg.data.capacity = sizeof(log_buf);
        RCCHECK(rclc_publisher_init_default(
            &log_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp/log"));
    }
}

// Called every loop tick (10 ms)
void hil_ros_tick() {
    if (HIL_ENABLED) {
        motor_get_step_rates(&hil_stepper_buf[0], &hil_stepper_buf[1]);
        RCSOFTCHECK(rcl_publish(&hil_stepper_publisher, &hil_stepper_msg, NULL));
    }
    if (DEBUG_OVER_ROS) {
        size_t n = SerialDebug.drain_output(log_buf, sizeof(log_buf) - 1);
        if (n > 0) {
            log_buf[n] = 0;
            log_msg.data.size = n;
            RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
        }
    }
}

// Tear down every micro-ROS entity without waiting for the (gone) agent
static void destroy_micro_ros() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&datum_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&gps_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&path_data_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&fused_pose_subscriber, &node));
    if (HIL_ENABLED) {
        RCSOFTCHECK(rcl_subscription_fini(&hil_gnss_subscriber, &node));
        RCSOFTCHECK(rcl_subscription_fini(&hil_imu_subscriber, &node));
        RCSOFTCHECK(rcl_publisher_fini(&hil_stepper_publisher, &node));
    }
    if (DEBUG_OVER_ROS) {
        RCSOFTCHECK(rcl_subscription_fini(&console_subscriber, &node));
        RCSOFTCHECK(rcl_publisher_fini(&log_publisher, &node));
    }
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

// Agent watchdog, called every loop tick. Pings once per second; after 3 misses the
// entities are destroyed and recreated as soon as the agent answers again (agent or
// host restart, cable pulled). Returns true while the link is up.
#define ROS_PING_PERIOD_MS 1000
#define ROS_PING_MISSES    3
static bool ros_link_check() {
    static bool connected = true;   // setup() only returns once the agent answered
    static int misses = 0;
    static unsigned long last_ping = 0;

    unsigned long now = millis();
    if (now - last_ping < ROS_PING_PERIOD_MS) return connected;
    last_ping = now;

    bool alive = rmw_uros_ping_agent(100, 1) == RMW_RET_OK;
    if (connected) {
        misses = alive ? 0 : misses + 1;
        if (misses >= ROS_PING_MISSES) {
            SerialDebug.println("[ROS] Agent lost, waiting for it");
            destroy_micro_ros();
            connected = false;
        }
    } else if (alive) {
        init_micro_ros();
        connected = true;
        misses = 0;
        if (CONFIG_PATH) pending_path_publish = true;   // the new host side has no path yet
        SerialDebug.println("[ROS] Agent reconnected");
    }
    return connected;
}

// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

// Stack headroom report (console 'h'): FreeRTOS high-water mark per task, in bytes.
static TaskHandle_t task_handles[5];   // motor, imu, gps, motion, controller
static void print_task_report() {
    for (TaskHandle_t t : task_handles) {
        if (t) SerialDebug.printf("[SYS] %-14s stack free min %u B\r\n", pcTaskGetName(t), (unsigned)uxTaskGetStackHighWaterMark(t));
    }
    SerialDebug.printf("[SYS] loop stack free min %u B, heap free %u B (min %u B)\r\n",
                       (unsigned)uxTaskGetStackHighWaterMark(NULL), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
}

void init_freertos() {
    if (ENABLE_MOTORS) {
        xTaskCreatePinnedToCore(
            motor_task,
            "MotorTask",
            2048,
            NULL,
            2,
            &task_handles[0],
            0
        );
    }

    if (ENABLE_IMU) {
        xTaskCreatePinnedToCore(
            imu_task,
            "IMUTask",
            4096,  // float printf (gyro calibration log) overflowed 2048
            NULL,
            1,
            &task_handles[1],
            0
        );
    }

    if (ENABLE_GPS) {
        xTaskCreatePinnedToCore(
            gps_task,
            "GPSTask",
            4096,   // 2816 left 584 B free (float printf), 2026-10-27
            NULL,
            1,
            &task_handles[2],
            0
        );
    }

    if (ENABLE_MOTION) {
        xTaskCreatePinnedToCore(
            motion_task,
            "MotionTask",
            4096,   // the MPC's Eigen matrices are on the heap, not this stack
            NULL,
            1,
            &task_handles[3],
            0
        );
    }
}

void init_peripherals() {
    if (ENABLE_IMU) {
        if (!init_imu()) {
            SerialDebug.println("[INIT] IMU failed, disabling");
            ENABLE_IMU = false;
        }
        SerialDebug.println("[INIT] IMU OK");
    }

    if (ENABLE_GPS) {
        if (!init_gps()) {
            SerialDebug.println("[INIT] GPS failed, disabling");
            ENABLE_GPS = false;
        }
        SerialDebug.println("[INIT] GPS OK");
    }

    if (ENABLE_MOTORS) {
        if (!init_motors()) {
            SerialDebug.println("[INIT] Motors failed, disabling");
            ENABLE_MOTORS = false;
        }
        SerialDebug.println("[INIT] Motors OK");
    }


    FileResult configResult = InitConfiguration();
    if (configResult != SUCCESS) {
        SerialDebug.printf("[INIT] Preferences init failed (code %d)\r\n", configResult);
    } else {
        SerialDebug.println("[INIT] Preferences OK");
    }
}

static void init_test_path() {
    SerialDebug.println("[TEST] Installing hardcoded test path");

    // Datum (arbitrary - host EKF not running for this test)
    BASE_LAT = 45.5;
    BASE_LON = 18.0;

    // Base exit point: fixed distance in the +x direction (yaw = 0 means east-aligned)
    BASE_EXIT_X_CM = DATUM_BASE_EXIT_DISTANCE_CM;
    BASE_EXIT_Y_CM = 0;

    MOWER_OVERLAP = 15;

    AlgorithmCaptureStart();

    // Outline 0: outer perimeter (3m x 3m). Point 0 is the base. A 0.6 m bush in the middle; on a
    // 2 m lawn it would leave 0.7 m to the edge, too narrow for the body to turn in.
    AlgorithmCaptureNewOutline();
    AlgorithmCaptureSetNewPoint(0,   0);     // base / charging station
    AlgorithmCaptureSetNewPoint(300, 0);
    AlgorithmCaptureSetNewPoint(300, 300);
    AlgorithmCaptureSetNewPoint(0,   300);

    AlgorithmCaptureNewOutline();
    AlgorithmCaptureSetNewPoint(180, 150);
    AlgorithmCaptureSetNewPoint(171, 171);
    AlgorithmCaptureSetNewPoint(150, 180);
    AlgorithmCaptureSetNewPoint(129, 171);
    AlgorithmCaptureSetNewPoint(120, 150);
    AlgorithmCaptureSetNewPoint(129, 129);
    AlgorithmCaptureSetNewPoint(150, 120);
    AlgorithmCaptureSetNewPoint(171, 129);

    if (AlgorithmCaptureEnd()) {
        SerialDebug.println("[TEST] Path generated successfully");
        CONFIG_PATH = true;
    } else {
        SerialDebug.println("[TEST] Path generation FAILED");
        CONFIG_PATH = false;
    }

    CONFIG_DATUM = true;
    pending_datum_publish = true;
    datum_publish_count = 0; 

    if (CONFIG_PATH) {
        pending_path_publish = true;
    }
}

static void publish_manual_move(float lin, float ang) {
    if (!MANUAL_MODE_ACTIVE) {
        SerialDebug.println("[DEBUG] Not in manual mode (press 'm' first)");
        return;
    }
    motor_data_t cmd = { MOTOR_MOVE, lin, ang };
    DDS_PUBLISH("/motor", cmd);
    LAST_MANUAL_MOVE_MS = millis();
    MANUAL_MOTORS_STOPPED = false;
}

// -------------------------------------------------------- PROGRAM START ---------------------------------------------------------------
void main_task(void* parameter);
void setup() {
    SerialDebug.begin(DEBUG_BAUDRATE, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);
    SerialDebug.setTimeout(COMMUNICATION_TIMEOUT);
    SerialDebug.println("[BOOT] Starting...");

    SerialDebug.printf("[BOOT] Reset reason: %d\r\n", esp_reset_reason());

    dds_init();
    SerialDebug.println("[BOOT] DDS initialized");

    Serial.begin(SERIAL_BAUDRATE);

    SerialDebug.println("[BOOT] Connecting to micro-ROS agent...");
    set_microros_serial_transports(Serial);
    init_micro_ros();
    SerialDebug.println("[BOOT] micro-ROS ready");

    SerialDebug.println("[BOOT] Press 'p' within 1.5 seconds to clear saved datum...");
    unsigned long wait_start = millis();
    while (millis() - wait_start < 1500) {
        if (SerialDebug.available()) {
            char c = SerialDebug.read();
            if (c == 'p') {
                Preferences p;
                p.begin("config", false);
                p.putBool("CFG_DATUM", false);
                p.putDouble("BASE_LON", 0.0);
                p.putDouble("BASE_LAT", 0.0);
                p.end();
                SerialDebug.println("[BOOT] Datum cleared");
                break;
            }
        }
        delay(10);
    }

    SerialDebug.println("[BOOT] Initializing peripherals...");
    init_peripherals();

    SerialDebug.println("[BOOT] Loading configuration...");
    FileResult configResult = LoadConfiguration();
    if (configResult != SUCCESS) {
        SerialDebug.printf("[BOOT] No configuration found (code %d), saving defaults\r\n", configResult);
        configResult = SaveConfiguration();
        if (configResult != SUCCESS) {
            SerialDebug.printf("[BOOT] Failed to save configuration (code %d)\r\n", configResult);
        }
    } else {
        SerialDebug.println("[BOOT] Configuration loaded");
        SerialDebug.printf("[BOOT] Datum=%d, Path=%d, Battery=%d\r\n", CONFIG_DATUM, CONFIG_PATH, CONFIG_BATTERY);

        if (CONFIG_PATH) {
            pending_path_publish = true;
        }
    }

    if (CONFIG_DATUM) {
        SerialDebug.printf("[BOOT] Datum will publish after tasks start: lat=%.7f lon=%.7f\r\n", BASE_LAT, BASE_LON);
        pending_datum_publish = true;
    }

    // TEST: install hardcoded path (overrides any saved config)
    if (HARDCODED_DATUM_AND_PATH) {
        init_test_path();
    }

    SerialDebug.println("[BOOT] Starting FreeRTOS tasks...");
    init_freertos();

    xTaskCreatePinnedToCore(controller_task, "ControllerTask", 4096, NULL, 1, &task_handles[4], 0);
    SerialDebug.println("[BOOT] Controller task started");

    SerialDebug.println("[BOOT] Boot complete");

    SerialDebug.println("");
    SerialDebug.println("=== GreenTitan Serial Controls ===");
    SerialDebug.println("--- Movement ---");
    SerialDebug.println("  m       toggle manual mode");
    SerialDebug.println("  w/s     forward / backward");
    SerialDebug.println("  a/d     turn left / right");
    SerialDebug.println("  space   immediate stop");
    SerialDebug.println("--- Path Recording ---");
    SerialDebug.println("  c       start new path (clears)");
    SerialDebug.println("  o       start new outline");
    SerialDebug.println("  p       add point to outline");
    SerialDebug.println("  e       end path & save");
    SerialDebug.println("--- Other ---");
    SerialDebug.println("  g       capture GPS datum");
    SerialDebug.println("  t       start mowing");
    SerialDebug.println("  x       stop mowing");
    SerialDebug.println("  l       line test: 1.5 m straight ahead");
    SerialDebug.println("  h       task stacks / heap");
    SerialDebug.println("  k       simulate obstacle");
    SerialDebug.println("  b       simulate rain");
    SerialDebug.println("==================================");
    SerialDebug.println("");
}

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void loop() {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(15, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 10 * 1000); // 10 ms

    // ------- THREAD SETUP CODE START -------

    dds_result_t result;
    result = DDS_SUBSCRIBE("/datum/set", datum_set_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Topic subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }
    result = DDS_SUBSCRIBE("/imu", imu_topic_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Topic subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }
    result = DDS_SUBSCRIBE("/gps", gps_topic_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Topic subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }

    // ------- THREAD SETUP CODE END -------

    vTaskDelay(500);
    
    while(1) {
        uint32_t notification_value;
        xTaskNotifyWait(0x00, 0xFF, &notification_value, portMAX_DELAY);

        if (notification_value & DDS_NOTIFY_BIT) {
            DDS_TAKE_MUTEX(&thread_context);
            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
        if (notification_value & THREAD_NOTIFY_BIT) {
            DDS_TAKE_MUTEX(&thread_context);

            if (pending_datum_publish) {
                static unsigned long last_datum_publish = 0;
                unsigned long now = millis();
                if (now - last_datum_publish > 1000) {
                    publish_datum();
                    last_datum_publish = now;
                    datum_publish_count++;
                    if (datum_publish_count >= 1) {
                        pending_datum_publish = false;
                        datum_publish_count = 0;
                        datum_published = true;
                        SerialDebug.println("[MAIN] Datum publish complete");
                    }
                }
            }

            if (pending_path_publish && CONFIG_DATUM && datum_published) {
                publish_path_markers(&path_data_publisher);
                pending_path_publish = false;
            }

            // Manual mode watchdog — auto-stop if no move command in MANUAL_WATCHDOG_MS
            if (MANUAL_MODE_ACTIVE && !MANUAL_MOTORS_STOPPED) {
                if ((int32_t)(millis() - LAST_MANUAL_MOVE_MS) > MANUAL_WATCHDOG_MS) {
                    motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
                    DDS_PUBLISH("/motor", stop);
                    MANUAL_MOTORS_STOPPED = true;
                }
            }

            // Debug: manual control via serial
            if (SerialDebug.available()) {
                char c = SerialDebug.read();
                if (c == 'm') {
                    int sig = MANUAL_MODE_ACTIVE ? SIGNAL_MANUAL_OFF : SIGNAL_MANUAL_ON;
                    SerialDebug.printf("[DEBUG] Manual toggle: requesting %s\r\n",
                                    sig == SIGNAL_MANUAL_ON ? "ON" : "OFF");
                    controller_signal_t signal = { sig };
                    DDS_PUBLISH("/controller/signal", signal);
                }
                else if (c == 'w') publish_manual_move( MANUAL_LINEAR_SPEED,  0.0f);
                else if (c == 's') publish_manual_move(-MANUAL_LINEAR_SPEED,  0.0f);
                else if (c == 'a') publish_manual_move( 0.0f,  MANUAL_ANGULAR_SPEED);
                else if (c == 'd') publish_manual_move( 0.0f, -MANUAL_ANGULAR_SPEED);
                else if (c == 'v') {   // "v<lin m/s>,<ang rad/s>\n": any manual motion, e.g. v0.2,-0.4
                    char buf[24]; int len = 0;
                    while (SerialDebug.available() && len < (int)sizeof(buf) - 1) {
                        char ch = SerialDebug.read();
                        if (ch == '\n' || ch == '\r') break;
                        buf[len++] = ch;
                    }
                    buf[len] = '\0';
                    float lin, ang;
                    if (sscanf(buf, "%f,%f", &lin, &ang) == 2) publish_manual_move(lin, ang);
                    else SerialDebug.println("[DEBUG] Usage: v<lin>,<ang>  e.g. v0.2,-0.4");
                }
                else if (c == ' ') {
                    if (MANUAL_MODE_ACTIVE) {
                        motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
                        DDS_PUBLISH("/motor", stop);
                        MANUAL_MOTORS_STOPPED = true;
                        SerialDebug.println("[DEBUG] Manual stop");
                    }
                }

                // Path Recording (the planner owns the path while a mission runs)
                else if ((c == 'c' || c == 'o' || c == 'p' || c == 'e') && !controller_path_edit_allowed()) {
                    SerialDebug.println("[DEBUG] Path edit rejected: mission running (x stops it)");
                }
                else if (c == 'c') {  // Clear / start new path capture
                    AlgorithmCaptureStart();
                    CONFIG_PATH = false;
                    SaveConfiguration();
                    SerialDebug.println("[DEBUG] Path capture started");
                }
                else if (c == 'o') {  // New outline
                    AlgorithmCaptureNewOutline();
                    SerialDebug.println("[DEBUG] New outline started");
                }
                else if (c == 'p') {  // Add point
                    if (!CONFIG_DATUM) {
                        SerialDebug.println("[DEBUG] Cannot add point: no datum set");
                    } else {
                        AlgorithmCaptureNewPoint();
                        SerialDebug.println("[DEBUG] Point added");
                    }
                }
                else if (c == 'e') {  // End path capture & save
                    if (AlgorithmCaptureEnd()) {
                        CONFIG_PATH = true;
                        SaveConfiguration();
                        publish_path_markers(&path_data_publisher);
                        SerialDebug.println("[DEBUG] Path capture completed and saved");
                    } else {
                        SerialDebug.println("[DEBUG] Path capture failed");
                    }
                }
                
                // Other Controls
                else if (c == 'g') {  // Capture GPS datum
                    controller_signal_t signal = { SIGNAL_START_DATUM_CAPTURE };
                    DDS_PUBLISH("/controller/signal", signal);
                    SerialDebug.println("[DEBUG] Triggering datum capture");
                }
                else if (c == 't') {  // Start mowing (t for staRt)
                    SerialDebug.println("[DEBUG] Triggering MOWER/START via serial");
                    controller_signal_t signal = { SIGNAL_START_MOWING };
                    DDS_PUBLISH("/controller/signal", signal);
                }
                else if (c == 'l') {  // Line test: follow a 1.5 m line straight ahead, then stop
                    if (MANUAL_MODE_ACTIVE || !controller_path_edit_allowed() || !fused_pose_seen || !imu_ready()) {
                        SerialDebug.println("[DEBUG] Line test needs IDLE (not manual), a fused pose and a calibrated gyro");
                    } else {
                        motion_command_t cmd = { MOVING, fused_x_last, fused_y_last,
                                                 fused_x_last + 1.5 * cos(fused_yaw_for_twist),
                                                 fused_y_last + 1.5 * sin(fused_yaw_for_twist) };
                        DDS_PUBLISH("/motion/command", cmd);
                        SerialDebug.printf("[DEBUG] Line test from (%.2f, %.2f) to (%.2f, %.2f)\r\n",
                                           cmd.start_x, cmd.start_y, cmd.end_x, cmd.end_y);
                    }
                }
                else if (c == 'h') {  // Health: stack headroom and heap
                    print_task_report();
                }
                else if (c == 'x') {  // Stop mowing: halt, blade off, back to IDLE
                    SerialDebug.println("[DEBUG] Triggering mowing stop via serial");
                    controller_signal_t signal = { SIGNAL_STOP_MOWING };
                    DDS_PUBLISH("/controller/signal", signal);
                }
                else if (c == 'k') {  // Obstacle (k for blocK)
                    SerialDebug.println("[DEBUG] Triggering obstacle avoidance via serial");
                    controller_signal_t signal = { SIGNAL_OBSTACLE };
                    DDS_PUBLISH("/controller/signal", signal);
                }
                else if (c == 'b') {  // Rain
                    SerialDebug.println("[DEBUG] Triggering rain sensor via serial");
                    controller_signal_t signal = { SIGNAL_RAIN };
                    DDS_PUBLISH("/controller/signal", signal);
                }
            }

            if (ros_link_check()) {
                hil_ros_tick();
                RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
                // spin_some takes at most one message per subscription; a second pass drains
                // what queued up meanwhile (fused_pose, HIL samples) before it fills the history
                RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));
            }

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}