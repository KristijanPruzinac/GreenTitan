#include "motor_task.h"

static dds_thread_context_t thread_context;

// Create the stepper motor objects
ContinuousStepper<StepperDriver> motorA;
ContinuousStepper<StepperDriver> motorB;

static double MOTOR_METERS_PER_STEP = 2 * PI * WHEEL_RADIUS / MOTOR_STEPS_PER_REV;

double odom_x = 0;
double odom_y = 0;
double odom_theta = 0;
double odom_linear_velocity  = 0;
double odom_angular_velocity = 0;

// Simulation
static double sim_left_speed = 0;
static double sim_right_speed = 0;
static double sim_linear_velocity  = 0;
static double sim_angular_velocity = 0;
static double sim_pos_x     = 0;
static double sim_pos_y     = 0;
static double sim_pos_theta = 0;

bool init_motors() {
  if (SIMULATION_ENABLED) return true;

  pinMode(MOTOR_MAIN, OUTPUT);

  motorA.begin(MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
  motorB.begin(MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN);

  // Set acceleration
  motorA.setAcceleration(MOTOR_ACCELERATION);
  motorB.setAcceleration(MOTOR_ACCELERATION);

  return true;
}

static void motor_stop() {
  if (SIMULATION_ENABLED){
    sim_left_speed = 0;
    sim_right_speed = 0;
    return;
  }

  motorA.stop();
  motorB.stop();

  xQueueReset(thread_context.queue); // Clear any pending motor commands to prevent them from executing after a stop command
}

static void motor_main_on() {
  if (SIMULATION_ENABLED) return;

  digitalWrite(MOTOR_MAIN, HIGH);
}

static void motor_main_off() {
  if (SIMULATION_ENABLED) return;

  digitalWrite(MOTOR_MAIN, LOW);
}

static void motor_move(double leftSpeed, double rightSpeed) {
  if (SIMULATION_ENABLED) {
    sim_left_speed = leftSpeed;
    sim_right_speed = rightSpeed;
    return;
  }
  
  motorA.spin(leftSpeed);
  motorB.spin(rightSpeed);
}

static void motor_move_vel(double linear_vel, double angular_vel) {
    // Clamp velocities to safe limits
    linear_vel = fmin(fmax(linear_vel, -MAX_LINEAR_VEL), MAX_LINEAR_VEL);
    angular_vel = fmin(fmax(angular_vel, -MAX_ANGULAR_VEL), MAX_ANGULAR_VEL);
    
    // Differential drive kinematics
    // Convert linear and angular velocities to wheel speeds
    double left_speed = linear_vel - (angular_vel * WHEEL_BASE / 2.0);
    double right_speed = linear_vel + (angular_vel * WHEEL_BASE / 2.0);
    
    // Convert to wheel rotational speed (rad/s) if needed by your motors
    double left_rotational_speed = left_speed / WHEEL_RADIUS;
    double right_rotational_speed = right_speed / WHEEL_RADIUS;

    double left_rotational_speed_steps = left_rotational_speed * MOTOR_STEPS_PER_REV;
    double right_rotational_speed_steps = right_rotational_speed * MOTOR_STEPS_PER_REV;
    
    if (SIMULATION_ENABLED) {
        sim_left_speed = left_speed;
        sim_right_speed = right_speed;
        return;
    }
    
    // Send to motors (assuming motorA.spin takes m/s)
    motorA.spin(left_rotational_speed_steps);
    motorB.spin(right_rotational_speed_steps);
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
    motor_move_vel(data->linear_vel, data->angular_vel);
  }
}

static int divide_frequency = 100;
static int divide_counter = 0;
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

            divide_counter++;
            if (divide_counter == divide_frequency) {
              divide_counter = 0;
            }

            if (!SIMULATION_ENABLED){
              motorA.loop();
              motorB.loop();
            }

            if (divide_counter == 0) {
              double leftSpeed, rightSpeed;

              if (SIMULATION_ENABLED) {
                  leftSpeed  = sim_left_speed;
                  rightSpeed = sim_right_speed;
              } else {
                  leftSpeed  = motorA.speed() * MOTOR_METERS_PER_STEP;
                  rightSpeed = motorB.speed() * MOTOR_METERS_PER_STEP;
              }

              // Odometry update
              double dt = (1.0 / MOTOR_UPDATE_FREQUENCY) * divide_frequency;
              double leftDist  = leftSpeed  * dt;
              double rightDist = rightSpeed * dt;
              double linear      = (leftDist + rightDist) / 2.0;
              double deltaTheta  = (rightDist - leftDist) / WHEEL_BASE;
              
              // Update odometry / simulate data for gps and imu
              odom_x     += linear * cos(odom_theta);
              odom_y     += linear * sin(odom_theta);
              odom_theta += deltaTheta;

              odom_linear_velocity  = (leftSpeed + rightSpeed) / 2.0f;
              odom_angular_velocity = (rightSpeed - leftSpeed) / WHEEL_BASE;

              // Simulation for odometry sensor output
              if (SIMULATION_ENABLED){
                int chance = esp_random() % 5;
                if (chance != 0){
                  sim_pos_x     += linear * cos(sim_pos_theta);
                  sim_pos_y     += linear * sin(sim_pos_theta);
                  sim_pos_theta += deltaTheta;

                  sim_linear_velocity  = (leftSpeed + rightSpeed) / 2.0f;
                  sim_angular_velocity = (rightSpeed - leftSpeed) / WHEEL_BASE;
                }
              }

              // Publish odometry
              odom_data_t data;
              if (SIMULATION_ENABLED){
                data = {
                    sim_pos_x,
                    sim_pos_y,
                    (float) sim_pos_theta,
                    (float) sim_linear_velocity,
                    (float) sim_angular_velocity,
                };
              }
              else {
                data = {
                    odom_x,
                    odom_y,
                    (float) odom_theta,
                    (float) ((leftSpeed + rightSpeed) / 2.0),
                    (float) (rightSpeed - leftSpeed) / WHEEL_BASE,
                };
              }

              result = DDS_PUBLISH("/odom", data);
              if (result != DDS_SUCCESS) {
                  SerialDebug.printf("Odom Topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
              }
            }

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}