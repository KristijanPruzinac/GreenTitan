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

#include "esp_dds.h"
#include "esp_timer.h"

// -------------------------------------------------------- DEFINITIONS ---------------------------------------------------------------
#include "definitions.h"

// -------------------------------------------------------- STATUS INDICATORS ---------------------------------------------------------------

int STATUS_BATTERY_LOW = false;
int STATUS_BATTERY_CHARGED = false;

// -------------------------------------------------------- CONFIGURATION ---------------------------------------------------------------

// Save which peripherals have been configured
bool CONFIG_PATH = false;
bool CONFIG_BATTERY = false;
bool CONFIG_MOTORS = false;
bool CONFIG_GYRO = false;
bool CONFIG_RAIN_SENSOR = false;

// Enable or disable peripherals for testing
bool ENABLE_MOTORS = true;
bool ENABLE_IMU = true;
bool ENABLE_GPS = true;
bool ENABLE_BATTERY = true;
bool ENABLE_RAIN_SENSOR = true;
bool ENABLE_BLUETOOTH = true;

// -------------------------------------------------------- GLOBALS ---------------------------------------------------------------

bool SETUP_COMPLETED = true;

int MOWER_OVERLAP = 15; // In cm
int MAX_DEVIATION = 50;
long long BASE_LON = -5672328;
long long BASE_LAT = 3376391;
long long BASE_EXIT_LON = -5672328;
long long BASE_EXIT_LAT = 3376391;

int GPS_ACC_THRESHOLD = 30; //TODO: Revert to 18
int GPS_STABILITY_CHECK_DURATION = 15; // 5 minutes TODO: Revert to 5 minutes

float BATTERY_LEVEL_MIN = 16;
float BATTERY_LEVEL_MAX = 18;

bool MOTOR_SIDE_INVERT = false;
bool MOTOR_LEFT_INVERT = true;
bool MOTOR_RIGHT_INVERT = false;
float MOTOR_OPTIMAL_VOLTAGE = 12;

//TODO: ADD TO CONFIG
bool IMU_INVERT = false;
float MOTION_ACC_FACTOR = 1.0;

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

// Sensors
#include "sensor/imu_task.h"
#include "sensor/gps_task.h"

// Effectors
#include "effector/motor_task.h"

// Simulation
#include "simulation/sim_task.h"

// -------------------------------------------------------- Micro-ROS ------------------------------------------------------------------

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t gps_publisher;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__NavSatFix gps_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
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

    // Odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    // IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    // GPS publisher
    RCCHECK(rclc_publisher_init_default(
        &gps_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "gps/fix"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
}

void odom_topic_callback(dds_callback_context_t* context) {
    odom_data_t* data = (odom_data_t*)context->message_data.data;

    odom_msg.pose.pose.position.x = data->x;
    odom_msg.pose.pose.position.y = data->y;

    odom_msg.pose.pose.position.z = 0.0f;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(data->theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(data->theta / 2.0f);

    odom_msg.twist.twist.linear.x = data->linear_vel;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.linear.z = 0.0f;

    odom_msg.twist.twist.angular.x = 0.0f;
    odom_msg.twist.twist.angular.y = 0.0f;
    odom_msg.twist.twist.angular.z = data->angular_vel;

    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen("odom");
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.size = strlen("base_link");

    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void imu_topic_callback(dds_callback_context_t* context) {
    IMU_data_t* data = (IMU_data_t*)context->message_data.data;

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

    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

void gps_topic_callback(dds_callback_context_t* context) {
    gps_data_t* data = (gps_data_t*)context->message_data.data;

    gps_msg.latitude  = data->latitude;
    gps_msg.longitude = data->longitude;
    gps_msg.altitude  = data->altitude;

    float acc = data->accuracy * data->accuracy; // variance = accuracy^2

    gps_msg.position_covariance[0] = acc;  // xx
    gps_msg.position_covariance[4] = acc;  // yy
    gps_msg.position_covariance[8] = acc * 4.0f; // zz (altitude less accurate)

    gps_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_DIAGONAL_KNOWN;

    gps_msg.status.status = 0;  // STATUS_FIX
    gps_msg.status.service = 1; // SERVICE_GPS

    gps_msg.header.frame_id.data = (char*)"gps_link";
    gps_msg.header.frame_id.size = strlen("gps_link");

    RCSOFTCHECK(rcl_publish(&gps_publisher, &gps_msg, NULL));
}

// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void init_freertos() {
    if (ENABLE_MOTORS) {
        xTaskCreatePinnedToCore(
            motor_task,
            "MotorTask",
            4096,
            NULL,
            2,
            NULL,
            0
        );
    }

    if (ENABLE_IMU) {
        xTaskCreatePinnedToCore(
            imu_task,
            "IMUTask",
            4096,
            NULL,
            1,
            NULL,
            0
        );
    }

    if (ENABLE_GPS) {
        xTaskCreatePinnedToCore(
            gps_task,
            "GPSTask",
            4096,
            NULL,
            1,
            NULL,
            0
        );
    }
}

void init_peripherals() {
    if (ENABLE_IMU) {
        if (!init_imu()) {
            ENABLE_IMU = false;
            return;
        }
    }

    if (ENABLE_GPS) {
        if (!init_gps()) {
            ENABLE_GPS = false;
            return;
        }
    }

    if (ENABLE_MOTORS) {
        if (!init_motors()) {
            ENABLE_MOTORS = false;
            return;
        }
    }
}

// -------------------------------------------------------- PROGRAM START ---------------------------------------------------------------
void main_task(void* parameter);
void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    set_microros_serial_transports(Serial);
    init_micro_ros();
    init_peripherals(); //TODO: Make init functions check if peripherals are actually connected
    init_freertos();

    if (SIMULATION_ENABLED){
        xTaskCreatePinnedToCore(
            sim_task,
            "SimTask",
            4096,
            NULL,
            1,
            NULL,
            0
        );
    }

    // Start main task
    xTaskCreatePinnedToCore(
        main_task,
        "MainTask",
        4096,
        NULL,
        1,
        NULL,
        0
    );
}

void loop() { vTaskDelay(portMAX_DELAY); }

String Mode = "POWER_ON";
/*
POWER_ON
SETUP
START
RUNNING PAUSE
STOP
CHARGING

SETUP_TEST
TEST
*/

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void main_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(20, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 100 * 1000); // 100 ms

    // ------- THREAD SETUP CODE START -------

    dds_result_t result;
    result = DDS_SUBSCRIBE("/odom", odom_topic_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        Serial.printf("Topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }
    result = DDS_SUBSCRIBE("/imu", imu_topic_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        Serial.printf("Topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }
    result = DDS_SUBSCRIBE("/gps", gps_topic_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        Serial.printf("Topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    // ------- THREAD SETUP CODE END -------

    vTaskDelay(500);
    
    while(1) {
        // Wait for any notification (message or timer)
        uint32_t notification_value;
        xTaskNotifyWait(0x00, 0xFF, &notification_value, portMAX_DELAY);
        
        if (notification_value & DDS_NOTIFY_BIT) { // DDS message notification
            DDS_TAKE_MUTEX(&thread_context);
            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
        if (notification_value & THREAD_NOTIFY_BIT) { // Timer tick notification
            DDS_TAKE_MUTEX(&thread_context);

            // ------- THREAD LOOP CODE START -------

            RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}