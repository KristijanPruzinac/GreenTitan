#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// -------------------------------------------------------- DEFINITIONS ---------------------------------------------------------------
#include "Definitions.h"

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

// -------------------------------------------------------- Micro-ROS ------------------------------------------------------------------

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void InitMicroRos(){
    while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_platformio_node_publisher"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;
}


// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void InitFreeRtos() {
    if (ENABLE_MOTORS) {
        xTaskCreatePinnedToCore(
            motor_task,
            "MotorTask",
            2048,
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
            2048,
            NULL,
            2,
            NULL,
            0
        );
    }

    if (ENABLE_GPS) {
        xTaskCreatePinnedToCore(
            gps_task,
            "GPSTask",
            2048,
            NULL,
            2,
            NULL,
            0
        );
    }
}

void InitPeripherals() {
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
void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    set_microros_serial_transports(Serial);

    InitMicroRos();
    //InitPeripherals(); //TODO: Make init functions check if peripherals are actually connected
    //InitFreeRtos(); 

    delay(500);
}

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

void loop() {
    vTaskDelay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}