#include "motion_task.h"
#include "PID.h"

static PIDController heading_pid;

// PID tuning parameters (adjust these)
#define PID_KP      0.8f    // Proportional gain
#define PID_KI      0.01f    // Integral gain
#define PID_KD      1.5f    // Derivative gain
#define PID_TAU     0.1f   // Derivative low-pass filter time constant
#define PID_LIM_MIN -2.6f    // Min rotation speed (rad/s)
#define PID_LIM_MAX 2.6f   // Max rotation speed (rad/s)
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

static float x;
static float y;
static float yaw;
static float linear_vel;
static float rotation_vel;

static float start_x = 0;
static float start_y = 0;
static float end_x = 3;
static float end_y = 3;

static int mode = MOVING;

static void pose_updated_callback(dds_callback_context_t* context) {
    fused_pose_data_t* data = (fused_pose_data_t*)context->message_data.data;
    
    x = data->x;
    y = data->y;
    yaw = data->yaw;
    linear_vel = data->vx;
    rotation_vel = data->omega;
    
    if (mode == MOVING) {
        // Calculate desired heading (line following with correction)
        float line_angle = AngleBetweenPoints(start_x, start_y, end_x, end_y);
        float distance_from_line = DistanceFromLine(x, y, start_x, start_y, end_x, end_y);
        
        // Calculate correction angle based on distance from line
        float correction = sign_of(distance_from_line)
                        * fminf(HALF_PI, fabsf(distance_from_line) / MOTION_MAX_CORRECTION_DIST * HALF_PI);
        
        // Target heading = line angle + correction to return to line
        float target_yaw = normalize_angle(line_angle + correction);
        
        // Calculate heading error (PID setpoint = target_yaw, measurement = current yaw)
        float heading_error = angle_diff(yaw, target_yaw);
        
        // Update PID to get rotation command
        float rotation_cmd = PIDController_Update(&heading_pid, heading_error, 0.0f);
        
        // Calculate distance to goal for speed control
        float distance_to_goal = DistanceBetweenPoints(x, y, end_x, end_y);
        
        // Calculate heading error magnitude
        float heading_error_abs = fabsf(heading_error);

        float speed_scale;
        if (heading_error_abs <= MOTION_HEADING_ERROR_FULL_SPEED) {
            speed_scale = 1.0f;
        } else if (heading_error_abs >= MOTION_HEADING_ERROR_MIN_SPEED) {
            speed_scale = MOTION_MIN_SPEED_SCALE;
        } else {
            // Linear interpolation between full speed and min speed
            float t = (heading_error_abs - MOTION_HEADING_ERROR_FULL_SPEED) / 
                    (MOTION_HEADING_ERROR_MIN_SPEED - MOTION_HEADING_ERROR_FULL_SPEED);
            speed_scale = 1.0f - t * (1.0f - MOTION_MIN_SPEED_SCALE);
        }

        // Base speed based on distance to goal
        float base_speed = MOTION_FORWARD_SPEED_NORMAL;
        if (distance_to_goal < MOTION_GOAL_SLOW_DOWN_DISTANCE) {
            base_speed = MOTION_FORWARD_SPEED_SLOW;
        }

        // Apply both scalings
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
        
        // Create and publish motor command
        motor_data_t msg;
        msg.instruction = MOTOR_MOVE;
        msg.linear_vel = linear_cmd;
        msg.angular_vel = rotation_cmd;
        
        dds_result_t result = DDS_PUBLISH("/motor", msg);
        if (result != DDS_SUCCESS) {
            SerialDebug.printf("Motor Topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
        }
        
        // Debug output (optional - can comment out for performance)
        SerialDebug.printf("%f %f\r\n", linear_cmd, rotation_cmd);
        //SerialDebug.printf("Target=%.2f, Yaw=%.2f, Err=%.2f, Rot=%.2f, Lin=%.2f, Dist=%.2f\r\n", target_yaw, yaw, heading_error, rotation_cmd, linear_cmd, distance_to_goal);
    }
}

static void motion_start(float p_start_x, float p_start_y, float p_end_x, float p_end_y) {
    mode    = MOVING;
    start_x = p_start_x;
    start_y = p_start_y;
    end_x   = p_end_x;
    end_y   = p_end_y;
}

static void motion_stop() {
    mode = WAITING;
}

static void motion_command_callback(dds_callback_context_t* context) {
    motion_command_t* data = (motion_command_t*)context->message_data.data;
    
    if (data->mode == WAITING) {
        SerialDebug.printf("Motion command: WAITING\r\n");
        motion_stop();
    }
    else if (data->mode == MOVING) {
        SerialDebug.printf("Motion command: MOVING\r\n");
        motion_start(data->start_x, data->start_y, data->end_x, data->end_y);
    }
}

static dds_thread_context_t thread_context;
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

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}