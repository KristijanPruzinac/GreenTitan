#include "motion_task.h"
#include "PID.h"

static dds_thread_context_t thread_context;

static PIDController heading_pid;

// PID tuning parameters (adjust these)
#define PID_KP      0.8f    // Proportional gain
#define PID_KI      0.01f    // Integral gain
#define PID_KD      3.0f    // Derivative gain
#define PID_TAU     0.1f   // Derivative low-pass filter time constant
#define PID_LIM_MIN -3.2f    // Min rotation speed (rad/s)
#define PID_LIM_MAX 3.2f   // Max rotation speed (rad/s)
#define PID_LIM_MIN_INT -0.01f // Min integrator windup
#define PID_LIM_MAX_INT 0.01f // Max integrator windup
#define SAMPLE_TIME_S 0.1f   // Sample time (seconds)

static void PIDController_Init(PIDController *pid) {
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;
}

static float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    float proportional = pid->Kp * error;
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;
}

static double x;
static double y;
static float yaw;
static float linear_vel;
static float rotation_vel;

static double start_x = 0;
static double start_y = 0;
static double end_x = 0;
static double end_y = 0;

static int mode = WAITING; // WAITING, MOVING, MOVING_REVERSE

static bool     teleport_active   = false;
static uint32_t teleport_deadline = 0;
static double   teleport_target_x = 0;
static double   teleport_target_y = 0;

static bool rotate_only_active = false;

static void motion_start(double p_start_x, double p_start_y, double p_end_x, double p_end_y) {
    mode    = MOVING;
    start_x = p_start_x;
    start_y = p_start_y;
    end_x   = p_end_x;
    end_y   = p_end_y;
    rotate_only_active = false;
}

static void motion_stop() {
    mode = WAITING;

    motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", stop);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Failed to send motor command: %d\r\n", result);
    }

    xQueueReset(thread_context.queue); // Clear any pending motion commands to prevent them from executing after a stop command
}

static void pose_updated_callback(dds_callback_context_t* context) {
    fused_pose_data_t* data = (fused_pose_data_t*)context->message_data.data;
    
    x = data->x;
    y = data->y;
    yaw = data->yaw;
    linear_vel = data->vx;
    rotation_vel = data->omega;
    
    if (mode == MOVING) {
        float line_angle = AngleBetweenPoints(start_x, start_y, end_x, end_y);
        double distance_from_line = DistanceFromLine(x, y, start_x, start_y, end_x, end_y);

        // Heading error against pure line direction, used to drive rotate-only state
        float raw_heading_error = angle_diff(yaw, line_angle);
        float raw_heading_error_abs = fabsf(raw_heading_error);

        // Update rotate-only state with hysteresis (do this BEFORE computing correction)
        if (rotate_only_active) {
            if (raw_heading_error_abs < MOTION_HEADING_ERROR_FULL_SPEED / 2) {
                rotate_only_active = false;
            }
        } else if (raw_heading_error_abs >= MOTION_HEADING_ERROR_ROTATE_ONLY) {
            rotate_only_active = true;
        }

        // Correction: zero if rotate-only, else lateral pull
        float correction;
        if (rotate_only_active) {
            correction = 0.0f;

            // Reset PID state during rotate-only so transition to forward is clean
            heading_pid.integrator = 0.0f;
            heading_pid.differentiator = 0.0f;
            heading_pid.prevMeasurement = yaw;
            heading_pid.prevError = 0.0f;
        } else {
            correction = correction = sign_of(distance_from_line) * atan(fabs(distance_from_line) / MOTION_CORRECTION_GAIN);
        }

        float target_yaw = normalize_angle(line_angle + correction);

        float heading_error = angle_diff(yaw, target_yaw);
        float rotation_cmd = PIDController_Update(&heading_pid, heading_error, 0.0f);

        double distance_to_goal = DistanceBetweenPoints(x, y, end_x, end_y);
        float heading_error_abs = fabsf(heading_error);

        // Speed scale: rotate-only forces zero, otherwise gradual ramp on heading_error
        float speed_scale;
        if (rotate_only_active) {
            speed_scale = 0.0f;
        } else if (heading_error_abs <= MOTION_HEADING_ERROR_FULL_SPEED) {
            speed_scale = 1.0f;
        } else if (heading_error_abs >= MOTION_HEADING_ERROR_MIN_SPEED) {
            speed_scale = MOTION_MIN_SPEED_SCALE;
        } else {
            float t = (heading_error_abs - MOTION_HEADING_ERROR_FULL_SPEED) / 
                    (MOTION_HEADING_ERROR_MIN_SPEED - MOTION_HEADING_ERROR_FULL_SPEED);
            speed_scale = 1.0f - t * (1.0f - MOTION_MIN_SPEED_SCALE);
        }

        // Base speed based on distance to goal
        float base_speed = MOTION_FORWARD_SPEED_NORMAL;
        if (distance_to_goal < MOTION_GOAL_SLOW_DOWN_DISTANCE) {
            base_speed = MOTION_FORWARD_SPEED_NORMAL * (distance_to_goal / MOTION_GOAL_SLOW_DOWN_DISTANCE);
            if (base_speed < 0.1f) base_speed = 0.1f;  // floor
        }

        float linear_cmd = base_speed * speed_scale;
        
        // Check if goal reached
        bool passed_goal = has_passed_goal(x, y, start_x, start_y, end_x, end_y);
        if (passed_goal) {
            SerialDebug.printf("Goal reached! Final position: (%.2f, %.2f)\r\n", x, y);

            motion_stop();

            controller_signal_t signal = { SIGNAL_MOTION_DONE };
            dds_result_t result = DDS_PUBLISH("/controller/signal", signal);
            if (result != DDS_SUCCESS) {
                SerialDebug.printf("Controller signal topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
            }
        }
        else {
            // Create and publish motor command
            motor_data_t msg;
            msg.instruction = MOTOR_MOVE;
            msg.linear_vel = linear_cmd;
            msg.angular_vel = rotation_cmd;
            
            dds_result_t result = DDS_PUBLISH("/motor", msg);
            if (result != DDS_SUCCESS) {
                SerialDebug.printf("Motor Topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
            }
        }
        
        // Debug output (optional - can comment out for performance)
        //SerialDebug.printf("%f %f\r\n", linear_cmd, rotation_cmd);
        //SerialDebug.printf("Target=%.2f, Yaw=%.2f, Err=%.2f, Rot=%.2f, Lin=%.2f, Dist=%.2f\r\n", target_yaw, yaw, heading_error, rotation_cmd, linear_cmd, distance_to_goal);
    }
}

static void motion_command_callback(dds_callback_context_t* context) {
    motion_command_t* data = (motion_command_t*)context->message_data.data;
    
    if (data->mode == WAITING) {
        SerialDebug.printf("Motion command: WAITING\r\n");
        motion_stop();
    }
    else if (data->mode == MOVING) {
        SerialDebug.printf("Motion command: MOVING to (%.2f, %.2f)\r\n", data->end_x, data->end_y);

        if (MOTION_TELEPORT_MODE) {
            teleport_active   = true;
            teleport_deadline = millis() + MOTION_TELEPORT_DELAY_MS;
            teleport_target_x = data->end_x;
            teleport_target_y = data->end_y;
            mode = WAITING;  // ensure pose_updated_callback doesn't run line-follow logic
            return;
        }

        motion_start(data->start_x, data->start_y, data->end_x, data->end_y);
    }
}

static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void motion_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 100 * 1000); // 100 ms

    // ------- THREAD SETUP CODE START -------

    heading_pid = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
			  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };

    PIDController_Init(&heading_pid);

    dds_result_t result;
    result = DDS_SUBSCRIBE("/fused_pose", pose_updated_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Fused pose topic subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }

    result = DDS_SUBSCRIBE("/motion/command", motion_command_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Motion command topic subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
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

            if (teleport_active && (int32_t)(millis() - teleport_deadline) >= 0) {
                teleport_active = false;
                SerialDebug.printf("[MOTION] Teleport done at (%.2f, %.2f)\r\n", teleport_target_x, teleport_target_y);

                // Inject simulated pose into motor_task so /odom and sim GPS reflect the teleport
                sim_pose_set_t sim_pose = { teleport_target_x, teleport_target_y, 0.0f };
                DDS_PUBLISH("/sim/pose_set", sim_pose);

                // Publish synthetic pose at target so the algorithm sees the robot move
                fused_pose_data_t fake = {
                    teleport_target_x,
                    teleport_target_y,
                    0.0f,   // yaw (algorithm doesn't read this)
                    0.0f,   // vx
                    0.0f    // omega
                };
                dds_result_t pose_result = DDS_PUBLISH("/fused_pose", fake);
                if (pose_result != DDS_SUCCESS) {
                    SerialDebug.printf("[MOTION] Synthetic pose publish failed: %s\r\n", DDS_RESULT_TO_STRING(pose_result));
                }

                // Then signal motion complete so controller asks for next point
                controller_signal_t signal = { SIGNAL_MOTION_DONE };
                dds_result_t sig_result = DDS_PUBLISH("/controller/signal", signal);
                if (sig_result != DDS_SUCCESS) {
                    SerialDebug.printf("[MOTION] Signal publish failed: %s\r\n", DDS_RESULT_TO_STRING(sig_result));
                }
            }

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}