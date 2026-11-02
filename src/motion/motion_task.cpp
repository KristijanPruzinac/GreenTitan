#include "motion_task.h"
#include "../sensor/imu_task.h"
#include "../navigation/nav_path.h"
#include "../navigation/nav_tracker.h"
#if MOTION_USE_MPC
#include "../navigation/nav_mpc_tracker.h"
#endif

// Mower glue around the navigation core (src/navigation): fills its robot description, feeds it
// the planner's points, drives the motors from its output and switches the blade per leg.

static void warn_body_swing(double sx, double sy, float beyond) {
    SerialDebug.printf("[MOTION] Turn at (%.2f, %.2f): body swings %.0f cm past the edge clearance\r\n",
                       sx, sy, beyond * 100.0f);
}

static const NavConfig nav_config = {
    WHEEL_BASE,                    // wheel_base
    0.0f,                          // tool_offset: the axle follows the path (2026-10-27); with the
                                   // blade as tool point it swung outward entering every curve
    MOTION_FORWARD_SPEED_NORMAL,   // v_max
    MOTION_GOAL_MIN_SPEED,         // v_min
    MAX_ANGULAR_VEL,               // w_max
    MAX_LINEAR_VEL,                // wheel_v_max
    MOTION_GOAL_DECEL,             // accel
    MOWER_FRONT_OVERHANG,          // front_overhang
    MOWER_REAR_OVERHANG,           // rear_overhang
    MOWER_HALF_WIDTH,              // half_width
    MOWER_EDGE_CLEARANCE,          // edge_clearance
    MOTION_EDGE_SWING,             // edge_swing
    warn_body_swing,               // on_body_swing
    MOTION_CORNER_RADIUS,          // corner_radius
    MOTION_MIN_CORNER_RADIUS,      // min_radius
    MOTION_MAX_ROUNDED_TURN,       // max_rounded_turn
    MOTION_PATH_TOLERANCE,         // path_tolerance
    MOTION_MAX_TRACK_HEADING_ERROR,// max_heading_error
    MOTION_HEADING_GAIN,           // heading_gain
    MOTION_APPROACH_DISTANCE,      // approach_distance
};

static dds_thread_context_t thread_context;

static double x;
static double y;
static float yaw;
static float linear_vel;
static float rotation_vel;

static int mode = WAITING; // WAITING, MOVING, MOVING_REVERSE, MOTION_PAUSE

// Speed cap for the queued path (docking approach): 0 = the path's own profile
static float speed_cap = 0.0f;

// Reverse line follower (MOVING_REVERSE, docking): the axle backs along the line rev_a -> rev_b.
// Closed loop on the fused pose: slip and bumps bend any "straight" drive, so the heading command is
// the line heading plus a pure-pursuit style cross-track correction, never an open heading hold.
static double rev_ax, rev_ay, rev_ux, rev_uy, rev_len;
static float  rev_speed;
static bool   rev_gyro_heading = false;   // heading = rev_yaw0 + gyro turn since the start
static float  rev_yaw0;
static double rev_gyro0;

static bool     teleport_active   = false;
static uint32_t teleport_deadline = 0;
static double   teleport_target_x = 0;
static double   teleport_target_y = 0;

static NavPath&   path = *new NavPath();   // on the heap: static DRAM is full
#if MOTION_USE_MPC
// On-path steering by TinyMPC instead of NavTracker (A/B build, ARCHITECTURE §6)
static const NavMpcWeights mpc_weights = {
    MOTION_MPC_Q_LATERAL, MOTION_MPC_Q_HEADING, MOTION_MPC_Q_CURVATURE, MOTION_MPC_R_CURVATURE_RATE,
};
static NavMpcTracker tracker;
static float      last_curvature = 0.0f;   // what is being commanded: the MPC's third state
#else
static NavTracker tracker;
#endif
static bool       turning = false;      // on the spot: path start, a sharp corner, or far off heading
static float      last_speed = 0.0f;
static uint8_t    applied_tool = NAV_TOOL_KEEP;

static uint32_t last_pose_ms  = 0;
static bool     pose_stale    = false;

// Pose arrival gaps while driving, reported every MOTION_POSE_GAP_REPORT_MS
static int      gap_count = 0, gap_long = 0;
static uint32_t gap_max_ms = 0, gap_report_ms = 0;

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
#if MOTION_USE_MPC
    last_curvature = 0.0f;
#endif
}

// Stop the wheels and drop the path; also drops queued messages (a real stop)
static void motion_stop() {
    mode = WAITING;
    speed_cap = 0.0f;
    path.clear();
    point_requested = false;
    publish_motor(MOTOR_STOP, 0.0f, 0.0f);
    xQueueReset(thread_context.queue); // Clear any pending motion commands to prevent them from executing after a stop command
}

static void motion_add_point(double px, double py, uint8_t tool, int8_t edge) {
    point_requested = false;
    if (!imu_ready()) {   // the controller and console check this too; never drive without gyro
        SerialDebug.printf("[MOTION] Gyro not calibrated, not driving\r\n");
        return;
    }
    bool starting = !path.active();
    double ax = x + GNSS_ANTENNA_BEHIND_AXLE * cos(yaw);
    double ay = y + GNSS_ANTENNA_BEHIND_AXLE * sin(yaw);
    NavWaypoint wp = { px, py, tool, edge };
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

static void reverse_start(const motion_command_t* c) {
    if (!imu_ready()) {
        SerialDebug.printf("[MOTION] Gyro not calibrated, not driving\r\n");
        return;
    }
    path.clear();
    point_requested = false;
    speed_cap = 0.0f;
    rev_ax = c->start_x;
    rev_ay = c->start_y;
    double dx = c->end_x - c->start_x, dy = c->end_y - c->start_y;
    rev_len = hypot(dx, dy);
    if (rev_len < 0.01) {
        signal_controller(SIGNAL_MOTION_DONE);
        return;
    }
    rev_ux = dx / rev_len;
    rev_uy = dy / rev_len;
    rev_speed = c->speed > 0.0f ? fminf(c->speed, MOTION_REVERSE_SPEED_MAX) : MOTION_REVERSE_SPEED_MAX;
    rev_gyro_heading = !isnan(c->start_yaw) && imu_yaw_now(&rev_gyro0);
    rev_yaw0 = c->start_yaw;
    mode = MOVING_REVERSE;
    stall_anchor_set = false;
    last_pose_ms = millis();
    last_speed = 0.0f;
    SerialDebug.printf("[MOTION] Reverse %.2f m along (%.2f, %.2f) -> (%.2f, %.2f)\r\n",
                       rev_len, c->start_x, c->start_y, c->end_x, c->end_y);
}

// One step of the reverse line follower (axle point: stable backing up as well as forward)
static void reverse_update(uint32_t now_ms) {
    float h = yaw;
    double g;
    if (rev_gyro_heading && imu_yaw_now(&g)) h = normalize_angle(rev_yaw0 + (float)(g - rev_gyro0));
    double ax = x + GNSS_ANTENNA_BEHIND_AXLE * cos(h);
    double ay = y + GNSS_ANTENNA_BEHIND_AXLE * sin(h);
    double px = ax - rev_ax, py = ay - rev_ay;
    double s = px * rev_ux + py * rev_uy;           // progress along the line
    double e = -px * rev_uy + py * rev_ux;          // axle left of the line
    if (s >= rev_len) {
        SerialDebug.printf("[MOTION] Reverse end at (%.2f, %.2f), %.1f cm off the line\r\n", x, y, e * 100.0);
        mode = WAITING;
        publish_motor(MOTOR_STOP, 0.0f, 0.0f);
        signal_controller(SIGNAL_MOTION_DONE);
        return;
    }
    // Backing up, the axle travels along yaw + pi: aim that direction back at the line
    float line = atan2f((float)rev_uy, (float)rev_ux);
    float travel = line - atanf((float)e / MOTION_REVERSE_LOOKAHEAD);
    float want_yaw = normalize_angle(travel + (float)M_PI);
    float err = normalize_angle(want_yaw - h);
    float w = fmaxf(-MAX_ANGULAR_VEL, fminf(MAX_ANGULAR_VEL, MOTION_ROTATE_GAIN * err));
    float v = 0.0f;
    if (fabsf(err) < MOTION_REVERSE_MAX_HEADING_ERROR) {   // far off: turn on the spot first
        float speed = fminf(rev_speed, last_speed + nav_config.accel / MOTION_UPDATE_FREQUENCY);
        speed = fminf(speed, fmaxf((float)(rev_len - s) * 2.0f, nav_config.v_min));   // ease into the end
        v = -speed;
    }
    last_speed = -v;

    // Stall: backing up but the fused position does not move (a wall behind, wheels spinning)
    if (!stall_anchor_set || v == 0.0f ||
        DistanceBetweenPoints(x, y, stall_anchor_x, stall_anchor_y) > MOTION_STALL_DIST) {
        stall_anchor_set = true;
        stall_anchor_x = x;
        stall_anchor_y = y;
        stall_anchor_ms = now_ms;
    }
    if (now_ms - stall_anchor_ms > MOTION_STALL_TIME_MS) {
        SerialDebug.printf("[MOTION] Stalled reversing at (%.2f, %.2f), stopping\r\n", x, y);
        motion_stop();
        signal_controller(SIGNAL_OBSTACLE);
        return;
    }
    publish_motor(MOTOR_MOVE, v, w);
}

// The poses come 0.1 s apart only on average; the gap report shows how often one is late
// (HIL 2026-10-29: a third of them, ARCHITECTURE item 26).
static void pose_updated_callback(dds_callback_context_t* context) {
    fused_pose_data_t* data = (fused_pose_data_t*)context->message_data.data;

    uint32_t now_ms = millis();
    if (mode == MOVING || mode == MOVING_REVERSE) {
        uint32_t gap = now_ms - last_pose_ms;
        gap_count++;
        if (gap > MOTION_POSE_GAP_LONG_MS) gap_long++;
        if (gap > gap_max_ms) gap_max_ms = gap;
        if (now_ms - gap_report_ms > MOTION_POSE_GAP_REPORT_MS) {
            SerialDebug.printf("[MOTION] Poses %d, %d late (> %d ms), longest gap %u ms\r\n",
                               gap_count, gap_long, MOTION_POSE_GAP_LONG_MS, gap_max_ms);
            gap_count = gap_long = 0;
            gap_max_ms = 0;
            gap_report_ms = now_ms;
        }
    }

    x = data->x;
    y = data->y;
    yaw = data->yaw;
    linear_vel = data->vx;
    rotation_vel = data->omega;
    last_pose_ms = now_ms;
    if (pose_stale) {
        pose_stale = false;
        stall_anchor_set = false;   // the wheels were stopped, don't count that as a stall
        SerialDebug.printf("[MOTION] Pose resumed\r\n");
    }

    if (mode == MOVING_REVERSE) {
        reverse_update(now_ms);
        return;
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
        speed_cap = 0.0f;
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
#if MOTION_USE_MPC
        float curvature = tracker.solve(ref, last_curvature);
        last_curvature = curvature;
#else
        float curvature = tracker.solve(ref, fmaxf(last_speed, nav_config.v_min));
#endif
        // Planned speed, ramped up no faster than the path acceleration (the wheel ramps then
        // keep the commanded curvature), never below creep speed so the end still closes
        float v_ref = speed_cap > 0.0f ? fminf(ref.speed, speed_cap) : ref.speed;
        linear_cmd = fmaxf(nav_config.v_min,
                           fminf(v_ref, last_speed + nav_config.accel / MOTION_UPDATE_FREQUENCY));
        rotation_cmd = linear_cmd * curvature;
        if (fabsf(rotation_cmd) > MAX_ANGULAR_VEL) {   // keep the curvature, slow down instead
            linear_cmd = MAX_ANGULAR_VEL / fabsf(curvature);
            rotation_cmd = copysignf(MAX_ANGULAR_VEL, rotation_cmd);
        }
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

        if (!path.active()) speed_cap = data->speed;   // a cap holds for the path it starts
        motion_add_point(data->end_x, data->end_y, data->tool, data->edge);
    }
    else if (data->mode == MOVING_REVERSE) {
        reverse_start(data);
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
#if MOTION_USE_MPC
    SerialDebug.printf("[MOTION] Tracker: TinyMPC (MOTION_USE_MPC)\r\n");
    if (!tracker.begin(mpc_weights)) {
#else
    if (!tracker.begin(nav_config)) {
#endif
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
            if ((mode == MOVING || mode == MOVING_REVERSE) && !pose_stale && (millis() - last_pose_ms) > MOTION_POSE_TIMEOUT_MS) {
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
