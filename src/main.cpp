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

#include "definitions.h"

// -------------------------------------------------------- CONFIGURATION ---------------------------------------------------------------

// Enable or disable peripherals for testing
bool ENABLE_MOTORS = true;
bool ENABLE_IMU = true;
bool ENABLE_GPS = true;
bool ENABLE_BLUETOOTH = true;

bool ENABLE_MOTION = true;

// -------------------------------------------------------- GLOBALS ---------------------------------------------------------------
HardwareSerial SerialDebug(1);

bool SETUP_COMPLETED = true;

double BASE_LON = -56.72328;
double BASE_LAT = 33.76391;
double BASE_EXIT_LON = -56.72328;
double BASE_EXIT_LAT = 33.76391;

int GPS_ACC_THRESHOLD = 30; //TODO: Revert to 18
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

bool IMU_INVERT = false;

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

// Bluetooth
#include "bluetooth/bluetooth_task.h"

// Simulation
#include "simulation/sim_task.h"

// -------------------------------------------------------- Micro-ROS ------------------------------------------------------------------
rcl_publisher_t datum_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t gps_publisher;
rcl_subscription_t fused_pose_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__NavSatFix gps_msg;
nav_msgs__msg__Odometry fused_pose_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void fused_pose_callback(const void* msgin);

unsigned long time_offset = 0;
static bool pending_datum_publish = false;
static int datum_publish_count = 0;
static bool datum_published = false;

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
    
    RCCHECK(rclc_subscription_init_default(
        &fused_pose_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odometry/filtered/global"
    ));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &fused_pose_subscriber,
        &fused_pose_msg,
        &fused_pose_callback,
        ON_NEW_DATA
    ));

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

void odom_topic_callback(dds_callback_context_t* context) {
    odom_data_t* data = (odom_data_t*)context->message_data.data;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    odom_msg.header.stamp.sec = ts.tv_sec;
    odom_msg.header.stamp.nanosec = ts.tv_nsec;

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

    odom_msg.pose.covariance[0]  = 0.001f;  // x
    odom_msg.pose.covariance[7]  = 0.001f;  // y
    odom_msg.pose.covariance[35] = 0.005f;   // yaw

    odom_msg.twist.covariance[0] = 0.001f;  // vx
    odom_msg.twist.covariance[35] = 0.01f;

    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen("odom");
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.size = strlen("base_link");

    if (datum_published) {
        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    }
}

void imu_topic_callback(dds_callback_context_t* context) {
    IMU_data_t* data = (IMU_data_t*)context->message_data.data;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    imu_msg.header.stamp.sec = ts.tv_sec;
    imu_msg.header.stamp.nanosec = ts.tv_nsec;

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

    if (datum_published) {
        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
}

void gps_topic_callback(dds_callback_context_t* context) {
    gps_data_t* data = (gps_data_t*)context->message_data.data;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    gps_msg.header.stamp.sec = ts.tv_sec;
    gps_msg.header.stamp.nanosec = ts.tv_nsec;

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

    if (datum_published) {
        RCSOFTCHECK(rcl_publish(&gps_publisher, &gps_msg, NULL));
    }
}

void fused_pose_callback(const void* msgin) {
    const nav_msgs__msg__Odometry* msg =
        (const nav_msgs__msg__Odometry*)msgin;

    // Extract x, y
    double x = (double)msg->pose.pose.position.x;
    double y = (double)msg->pose.pose.position.y;

    SerialDebug.printf("Fused pose received: x=%.2f, y=%.2f\r\n", x, y);

    // Quaternion → yaw (2D, so only z/w matter)
    float qz = (float)msg->pose.pose.orientation.z;
    float qw = (float)msg->pose.pose.orientation.w;
    float yaw = atan2f(2.0f * qw * qz, 1.0f - 2.0f * qz * qz);

    // Package and publish internally via your DDS system
    fused_pose_data_t data = {
        .x     = x,
        .y     = y,
        .yaw   = normalize_angle(yaw),
        .vx    = (float)msg->twist.twist.linear.x,
        .omega = (float)msg->twist.twist.angular.z,
    };

    dds_result_t result = DDS_PUBLISH("/fused_pose", data);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Fused pose publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }
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

    if (ENABLE_MOTION) {
        xTaskCreatePinnedToCore(
            motion_task,
            "MotionTask",
            4096,
            NULL,
            1,
            NULL,
            0
        );
    }

    if (ENABLE_BLUETOOTH) {
        xTaskCreatePinnedToCore(
            bluetooth_task,
            "BluetoothTask",
            8192,
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
            SerialDebug.println("[INIT] IMU failed, disabling");
            ENABLE_IMU = false;
            return;
        }
        SerialDebug.println("[INIT] IMU OK");
    }

    if (ENABLE_GPS) {
        if (!init_gps()) {
            SerialDebug.println("[INIT] GPS failed, disabling");
            ENABLE_GPS = false;
            return;
        }
        SerialDebug.println("[INIT] GPS OK");
    }

    if (ENABLE_MOTORS) {
        if (!init_motors()) {
            SerialDebug.println("[INIT] Motors failed, disabling");
            ENABLE_MOTORS = false;
            return;
        }
        SerialDebug.println("[INIT] Motors OK");
    }

    if (ENABLE_BLUETOOTH) {
        if (!InitBluetooth()) {
            SerialDebug.println("[INIT] Bluetooth failed, disabling");
            ENABLE_BLUETOOTH = false;
            return;
        }
        SerialDebug.println("[INIT] Bluetooth OK");
    }

    FileResult configResult = InitConfiguration();
    if (configResult != SUCCESS) {
        SerialDebug.printf("[INIT] Preferences init failed (code %d)\r\n", configResult);
    } else {
        SerialDebug.println("[INIT] Preferences OK");
    }
}

// -------------------------------------------------------- PROGRAM START ---------------------------------------------------------------
void main_task(void* parameter);
void setup() {
    SerialDebug.begin(DEBUG_BAUDRATE, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);
    SerialDebug.setTimeout(COMMUNICATION_TIMEOUT);
    SerialDebug.println("[BOOT] Starting...");

    dds_init();
    SerialDebug.println("[BOOT] DDS initialized");

    Serial.begin(SERIAL_BAUDRATE);

    SerialDebug.println("[BOOT] Connecting to micro-ROS agent...");
    set_microros_serial_transports(Serial);
    init_micro_ros();
    SerialDebug.println("[BOOT] micro-ROS ready");

    SerialDebug.println("[BOOT] Press 'p' within 1 second to clear saved datum...");
    unsigned long wait_start = millis();
    while (millis() - wait_start < 1000) {
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
    }

    if (CONFIG_DATUM) {
        SerialDebug.printf("[BOOT] Datum will publish after tasks start: lat=%.7f lon=%.7f\r\n", BASE_LAT, BASE_LON);
        pending_datum_publish = true;
    }

    SerialDebug.println("[BOOT] Starting FreeRTOS tasks...");
    init_freertos();

    xTaskCreatePinnedToCore(controller_task, "ControllerTask", 4096, NULL, 1, NULL, 0);
    SerialDebug.println("[BOOT] Controller task started");

    xTaskCreatePinnedToCore(main_task, "MainTask", 4096, NULL, 1, NULL, 0);
    SerialDebug.println("[BOOT] Main task started");

    SerialDebug.println("[BOOT] Boot complete");
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
    thread_context.queue = xQueueCreate(30, sizeof(dds_callback_context_t));
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
    result = DDS_SUBSCRIBE("/odom", odom_topic_callback, &thread_context);
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

            RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}