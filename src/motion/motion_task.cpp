#include "motion_task.h"
#include "../sensor/imu_task.h"
#include "../navigation/nav_path.h"
#include "../navigation/nav_tracker.h"

// Mower glue around the navigation core (src/navigation): fills its robot description, feeds it
// the planner's points, drives the motors from its output and switches the blade per leg.

static const NavConfig nav_config = {
    WHEEL_BASE,                    // wheel_base
    0.0f,                          // tool_offset: the axle follows the path; the blade trails it and
                                   // cuts inside the curves, so no part swings out (2026-10-27)
    MOTION_FORWARD_SPEED_NORMAL,   // v_max
    MOTION_GOAL_MIN_SPEED,         // v_min
    MAX_ANGULAR_VEL,               // w_max
    MAX_LINEAR_VEL,                // wheel_v_max
    MOTION_GOAL_DECEL,             // accel
    MOTION_CORNER_RADIUS,          // corner_radius
    MOTION_MIN_CORNER_RADIUS,      // min_radius
    MOTION_MAX_ROUNDED_TURN,       // max_rounded_turn
    MOTION_PATH_TOLERANCE,         // path_tolerance
    MOTION_MAX_TRACK_HEADING_ERROR,// max_heading_error
    400.0f, 20.0f, 0.1f, 0.01f,    // MPC weights: lateral, heading, curvature, curvature rate
};

static dds_thread_context_t thread_context;

static double x;
static double y;
static float yaw;
static float linear_vel;
static float rotation_vel;

static int mode = WAITING; // WAITING, MOVING, MOTION_PAUSE

static bool     teleport_active   = false;
static uint32_t teleport_deadline = 0;
static double   teleport_target_x = 0;
static double   teleport_target_y = 0;

static NavPath    path;
static NavTracker tracker;
static bool       turning = false;      // on the spot: path start, a sharp corner, or far off heading
static float      last_curvature = 0.0f;
static float      last_speed = 0.0f;
static uint8_t    applied_tool = NAV_TOOL_KEEP;

static uint32_t last_pose_ms  = 0;
static bool     pose_stale    = false;

// Stall check: driving but the fused (GPS) position has not moved. Uses only the sign of the
// command, never its magnitude. Seen in HIL 2026-10-27: high-centred on a molehill.
static bool     stall_anchor_set = false;
static double   stall_anchor_x, stall_anchor_y;
static uint32_t stall_anchor_ms;
static uint32_t rotate_since_ms;   // turn-on-the-spot start; a turn that never ends is a stall too

static void signal_controller(int sig) {
    controller_signal_t signal = { sig };
    dds_result_t result = DDS_PUBLISH("/controller/signal", signal);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[MOTION] Controller signal publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }
}

static void publish_motor(char instruction, float v, float w) {
    motor_data_t msg = { instruction, v, w };
    dds_result_t result = DDS_PUBLISH("/motor", msg);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[MOTION] Motor publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }
}

static void apply_tool() {
    uint8_t tool = path.tool();
    if (tool == NAV_TOOL_KEEP || tool == applied_tool) return;
    publish_motor(tool == NAV_TOOL_ON ? MOTOR_MAIN_ON : MOTOR_MAIN_OFF, 0.0f, 0.0f);
    applied_tool = tool;
}

// At most one request open, and only with room in the queue, so a point never arrives to a full one
static bool point_requested = false;

static void request_point() {
    if (point_requested || path.full()) return;
    point_requested = true;
    signal_controller(SIGNAL_MOTION_WANT_POINT);
}

static void start_turn() {
    turning = true;
    rotate_since_ms = millis();
    last_speed = 0.0f;
    last_curvature = 0.0f;
}

// Stop the wheels and drop the path; also drops queued messages (a real stop)
static void motion_stop() {
    mode = WAITING;
    path.clear();
    point_requested = false;
    publish_motor(MOTOR_STOP, 0.0f, 0.0f);
    xQueueReset(thread_context.queue); // Clear any pending motion commands to prevent them from executing after a stop command
}

static void motion_add_point(double px, double py, uint8_t tool) {
    point_requested = false;
    if (!imu_ready()) {   // the controller and console check this too; never drive without gyro
        SerialDebug.printf("[MOTION] Gyro not calibrated, not driving\r\n");
        return;
    }
    bool starting = !path.active();
    double ax = x + GNSS_ANTENNA_BEHIND_AXLE * cos(yaw);
    double ay = y + GNSS_ANTENNA_BEHIND_AXLE * sin(yaw);
    NavWaypoint wp = { px, py, tool };
    bool added = path.push(wp, ax, ay);
    if (starting && path.active()) {
        if (mode != MOTION_PAUSE) mode = MOVING;
        stall_anchor_set = false;
        last_pose_ms = millis();   // grace period until the first pose arrives
        start_turn();
        apply_tool();
    }
    if (!added && !path.active()) {   // a point on top of where it stands: nothing to drive
        signal_controller(SIGNAL_MOTION_DONE);
        return;
    }
    request_point();
}

static void pose_updated_callback(dds_callback_context_t* context) {
    fused_pose_data_t* data = (fused_pose_data_t*)context->message_data.data;

    x = data->x;
    y = data->y;
    yaw = data->yaw;
    linear_vel = data->vx;
    rotation_vel = data->omega;
    last_pose_ms = millis();
    if (pose_stale) {
        pose_stale = false;
        stall_anchor_set = false;   // the wheels were stopped, don't count that as a stall
        SerialDebug.printf("[MOTION] Pose resumed\r\n");
    }

    if (mode != MOVING || !path.active()) return;

    // The navigation core works on the wheel axle, the point a diff drive steers directly.
    // base_link (antenna = blade) trails it by GNSS_ANTENNA_BEHIND_AXLE.
    double ax = x + GNSS_ANTENNA_BEHIND_AXLE * cos(yaw);
    double ay = y + GNSS_ANTENNA_BEHIND_AXLE * sin(yaw);
    NavReference ref;
    if (!path.update(ax, ay, yaw, last_speed / MOTION_UPDATE_FREQUENCY, &ref)) return;
    if (ref.passed_waypoint) {
        apply_tool();
        request_point();
    }

    float linear_cmd = 0.0f, rotation_cmd = 0.0f;
    if (!turning && fabsf(ref.heading_error) > nav_config.max_heading_error) {
        start_turn();   // beyond the tracker's linear model: face the path first
    }
    if (!turning && ref.at_stop) {
        if (path.advance_past_stop()) {
            start_turn();   // sharp corner: the axle is on it, turn to the next leg
            apply_tool();
            request_point();
            return;
        }
        SerialDebug.printf("[MOTION] Path end at (%.2f, %.2f)\r\n", x, y);
        mode = WAITING;
        publish_motor(MOTOR_STOP, 0.0f, 0.0f);
        signal_controller(SIGNAL_MOTION_DONE);
        return;
    }
    if (turning) {
        float e = -ref.heading_error;
        if (fabsf(e) < MOTION_TURN_DONE) {
            turning = false;
        } else {
            rotation_cmd = fmaxf(-MAX_ANGULAR_VEL, fminf(MAX_ANGULAR_VEL, MOTION_ROTATE_GAIN * e));
        }
    }
    if (!turning) {
        float curvature = tracker.solve(ref, last_curvature);
        // Planned speed, ramped up no faster than the path acceleration (the wheel ramps then
        // keep the commanded curvature), never below creep speed so the end still closes
        linear_cmd = fmaxf(nav_config.v_min,
                           fminf(ref.speed, last_speed + nav_config.accel / MOTION_UPDATE_FREQUENCY));
        rotation_cmd = linear_cmd * curvature;
        if (fabsf(rotation_cmd) > MAX_ANGULAR_VEL) {   // keep the curvature, slow down instead
            linear_cmd = MAX_ANGULAR_VEL / fabsf(curvature);
            rotation_cmd = copysignf(MAX_ANGULAR_VEL, rotation_cmd);
        }
        last_curvature = curvature;
        last_speed = linear_cmd;
    }

    // Stall: the time only counts while driving (not turning on the spot)
    if (!stall_anchor_set || linear_cmd == 0.0f ||
        DistanceBetweenPoints(x, y, stall_anchor_x, stall_anchor_y) > MOTION_STALL_DIST) {
        stall_anchor_set = true;
        stall_anchor_x = x;
        stall_anchor_y = y;
        stall_anchor_ms = millis();
    }
    bool stalled = (millis() - stall_anchor_ms) > MOTION_STALL_TIME_MS ||
                   (turning && (millis() - rotate_since_ms) > MOTION_ROTATE_TIMEOUT_MS);
    if (stalled) {
        SerialDebug.printf("[MOTION] Stalled at (%.2f, %.2f) (%s), stopping\r\n",
                           x, y, turning ? "turn did not finish" : "no progress");
        motion_stop();
        signal_controller(SIGNAL_OBSTACLE);
        return;
    }

    publish_motor(MOTOR_MOVE, linear_cmd, rotation_cmd);
}

static void motion_command_callback(dds_callback_context_t* context) {
    motion_command_t* data = (motion_command_t*)context->message_data.data;

    if (data->mode == WAITING) {
        SerialDebug.printf("Motion command: WAITING\r\n");
        motion_stop();
    }
    else if (data->mode == MOTION_PAUSE) {
        if (mode == MOVING) mode = MOTION_PAUSE;
        publish_motor(MOTOR_STOP, 0.0f, 0.0f);
    }
    else if (data->mode == MOTION_RESUME) {
        if (mode == MOTION_PAUSE && path.active()) {
            mode = MOVING;
            stall_anchor_set = false;
            last_pose_ms = millis();
            start_turn();   // face the path again before tracking
        }
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

        motion_add_point(data->end_x, data->end_y, data->tool);
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

    path.configure(nav_config);
    if (!tracker.begin(nav_config)) {
        SerialDebug.printf("[MOTION] Tracker setup failed\r\n");
    }

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

            // Pose watchdog: without fresh poses the last MOTOR_MOVE would keep running.
            // Stop the wheels but stay in MOVING, so the next pose resumes path following.
            if (mode == MOVING && !pose_stale && (millis() - last_pose_ms) > MOTION_POSE_TIMEOUT_MS) {
                pose_stale = true;
                publish_motor(MOTOR_STOP, 0.0f, 0.0f);
                last_speed = 0.0f;
                SerialDebug.printf("[MOTION] Pose timeout, motors stopped\r\n");
            }

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
                signal_controller(SIGNAL_MOTION_DONE);
            }

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}
