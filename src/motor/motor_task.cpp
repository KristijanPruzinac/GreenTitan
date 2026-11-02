#include "motor_task.h"

static dds_thread_context_t thread_context;

// STEP pulses come from the LEDC peripheral, not from loop(): the software ticker
// toggled STEP once per 1 kHz loop() and topped out at ~500 steps/s.
class LedcOscillator {
public:
  void init(uint8_t pin) {
    channel_ = next_channel_;
    next_channel_ += 2;            // channels 0, 2 -> separate LEDC timers
    ledcSetup(channel_, 1000, 8);
    ledcAttachPin(pin, channel_);
    ledcWrite(channel_, 0);
  }

  void start(frequency_t frequency) {
    if (frequency != frequency_) {
      ledcChangeFrequency(channel_, frequency, 8);
      frequency_ = frequency;
    }
    if (!active_) {
      ledcWrite(channel_, 128);    // 50 % duty
      active_ = true;
    }
  }

  void stop() {
    ledcWrite(channel_, 0);
    active_ = false;
  }

  frequency_t minFrequency() const { return MOTOR_MIN_STEP_RATE; }  // below this the motor just stops

private:
  static uint8_t next_channel_;
  uint8_t channel_ = 0;
  frequency_t frequency_ = 0;
  bool active_ = false;
};
uint8_t LedcOscillator::next_channel_ = 0;

// Create the stepper motor objects
ContinuousStepper<StepperDriver, OscillatorTicker<LedcOscillator>> motorA;
ContinuousStepper<StepperDriver, OscillatorTicker<LedcOscillator>> motorB;

static double MOTOR_METERS_PER_STEP = 2 * PI * WHEEL_RADIUS / MOTOR_STEPS_PER_WHEEL_REV;

// The acceleration ramp runs here, on the real time between motor ticks. The library's own ramp
// adds acceleration x step period once per step period, but loop() only runs every 1 ms: above
// 1000 steps/s (0.24 m/s) it ramped at well under MOTOR_ACCELERATION, and in HIL no wheel ever
// changed speed faster than ~0.25 m/s^2 (2026-10-30). The library now takes each speed at once.
static double target_rate_a = 0, target_rate_b = 0;   // steps/s the wheels should reach
static double rate_a = 0, rate_b = 0;                  // steps/s the ramp has them at now
static int64_t ramp_last_us = 0;

static void ramp_motors() {
  int64_t now = esp_timer_get_time();
  double dv = MOTOR_ACCELERATION * fmin(0.01, (now - ramp_last_us) * 1e-6);
  ramp_last_us = now;
  rate_a += fmax(-dv, fmin(dv, target_rate_a - rate_a));
  rate_b += fmax(-dv, fmin(dv, target_rate_b - rate_b));
  motorA.spin(fabs(rate_a) < MOTOR_MIN_STEP_RATE ? 0.0 : rate_a);
  motorB.spin(fabs(rate_b) < MOTOR_MIN_STEP_RATE ? 0.0 : rate_b);
}

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

  // The ramp is ramp_motors(); the library follows each step rate at once
  motorA.setAcceleration(MOTOR_LIBRARY_ACCELERATION);
  motorB.setAcceleration(MOTOR_LIBRARY_ACCELERATION);

  return true;
}

void motor_get_step_rates(float* left, float* right) {
  *left = motorA.speed();
  *right = motorB.speed();
}

static void motor_stop() {
  if (SIMULATION_ENABLED){
    sim_left_speed = 0;
    sim_right_speed = 0;
    return;
  }

  target_rate_a = 0;
  target_rate_b = 0;

  xQueueReset(thread_context.queue); // Clear any pending motor commands to prevent them from executing after a stop command
}

static volatile bool blade_on = false;
bool motor_blade_on() { return blade_on; }

static void motor_main_on() {
  blade_on = true;
  if (SIMULATION_ENABLED) return;

  digitalWrite(MOTOR_MAIN, HIGH);
}

static void motor_main_off() {
  blade_on = false;
  if (SIMULATION_ENABLED) return;

  digitalWrite(MOTOR_MAIN, LOW);
}

static void motor_move(double leftSpeed, double rightSpeed) {
  if (SIMULATION_ENABLED) {
    sim_left_speed = leftSpeed;
    sim_right_speed = rightSpeed;
    return;
  }
  
  target_rate_a = leftSpeed;
  target_rate_b = rightSpeed;
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

    double left_rotational_speed_steps = left_rotational_speed / (2 * PI) * MOTOR_STEPS_PER_WHEEL_REV;
    double right_rotational_speed_steps = right_rotational_speed / (2 * PI) * MOTOR_STEPS_PER_WHEEL_REV;
    
    if (SIMULATION_ENABLED) {
        sim_left_speed = left_speed;
        sim_right_speed = right_speed;
        return;
    }
    
    // ramp_motors() takes the wheels there
    target_rate_a = left_rotational_speed_steps;
    target_rate_b = right_rotational_speed_steps;
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

static void sim_pose_set_callback(dds_callback_context_t* context) {
    if (!SIMULATION_ENABLED) return;
    sim_pose_set_t* data = (sim_pose_set_t*)context->message_data.data;
    sim_pos_x = data->x;
    sim_pos_y = data->y;
    sim_pos_theta = data->theta;
    odom_x = data->x;
    odom_y = data->y;
    odom_theta = data->theta;
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

    result = DDS_SUBSCRIBE("/sim/pose_set", sim_pose_set_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Sim pose set subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
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
              ramp_motors();
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
            }

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}