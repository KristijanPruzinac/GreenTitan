#include "motor_task.h"

// create the stepper motor objects
ContinuousStepper<StepperDriver> motorA;
ContinuousStepper<StepperDriver> motorB;

static float MOTOR_METERS_PER_STEP = 2 * PI * WHEEL_RADIUS / MOTOR_STEPS_PER_REV;

static float odom_x = 0;
static float odom_y = 0;
static float odom_theta = 0;

bool init_motors() {
  pinMode(MOTOR_MAIN, OUTPUT);

  motorA.begin(MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
  motorB.begin(MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN);

  // Set acceleration
  motorA.setAcceleration(MOTOR_ACCELERATION);
  motorB.setAcceleration(MOTOR_ACCELERATION);

  return true;
}

static void motor_stop() {
  motorA.stop();
  motorB.stop();
}

static void motor_main_on() {
  digitalWrite(MOTOR_MAIN, HIGH);
}

static void motor_main_off() {
  digitalWrite(MOTOR_MAIN, LOW);
}

static void motor_move(float leftSpeed, float rightSpeed) {
  motorA.spin(leftSpeed);
  motorB.spin(rightSpeed);
}

static void motor_topic_callback(dds_callback_context_t* context) {
  motor_data_t* data = (motor_data_t*)context->message_data.data;

  if (data->instruction == MOTOR_STOP) {
    motor_stop();
  }
  else if (data->instruction == MOTOR_MAIN_ON) {
    motor_main_on();
  }
  else if (data->instruction == MOTOR_MAIN_OFF) {
    motor_main_off();
  }
  else if (data->instruction == MOTOR_MOVE) {
    motor_move(data->left_speed_m_per_s, data->right_speed_m_per_s);
  }
}

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void motor_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 1000 * 1000 / MOTOR_UPDATE_FREQUENCY);

    // ------- THREAD SETUP CODE START -------

    dds_result_t result;
    result = DDS_SUBSCRIBE("/motor", motor_topic_callback, &thread_context);
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

            motorA.loop();
            motorB.loop();

            float leftStepsPerSecond = motorA.speed();
            float rightStepsPerSecond = motorB.speed();
            float leftSpeed = leftStepsPerSecond * MOTOR_METERS_PER_STEP;
            float rightSpeed = rightStepsPerSecond * MOTOR_METERS_PER_STEP;

            // Odometry update
            float dt = 1.0f / MOTOR_UPDATE_FREQUENCY;
            float leftDist  = leftSpeed  * dt;
            float rightDist = rightSpeed * dt;
            float linear      = (leftDist + rightDist) / 2.0f;
            float deltaTheta  = (rightDist - leftDist) / WHEEL_BASE;

            odom_x     += linear * cos(odom_theta);
            odom_y     += linear * sin(odom_theta);
            odom_theta += deltaTheta;

            // Publish odometry
            odom_data_t data = {
                odom_x,
                odom_y,
                odom_theta,
                (leftSpeed + rightSpeed) / 2.0f,
                (rightSpeed - leftSpeed) / WHEEL_BASE,
            };

            result = DDS_PUBLISH("/odom", data);
            if (result != DDS_SUCCESS) {
                Serial.printf("Topic publish failed: %s\n", DDS_RESULT_TO_STRING(result));
            }

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}