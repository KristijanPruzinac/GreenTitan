#include "controller_task.h"

static int robot_state = ROBOT_STATE_IDLE;

// External globals from algorithm
extern String algorithmTarget;
extern String algorithmMode;
extern bool algorithmAbortFull;

// GPS cache (updated by gps_callback)
static gps_data_t latest_gps_ctrl = {0, 0, 0, 999};
static bool       gps_valid_ctrl  = false;

// Datum capture state
static uint32_t datum_capture_start_ms  = 0;
static double   datum_capture_start_lat = 0;
static double   datum_capture_start_lon = 0;

// -------------------------------------------------------- HELPERS ---------------------------------------------------------------

static void go_to_base() {

}

static void go_to_base_exit() {
    
}

static void stop_motors() {
    motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", stop);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send motor command: %d\r\n", result);
    }
}

static void motor_main_on() {
    motor_data_t msg = { MOTOR_MAIN_ON, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", msg);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send motor command: %d\r\n", result);
    }
}

static void motor_main_off() {
    motor_data_t msg = { MOTOR_MAIN_OFF, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", msg);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send motor command: %d\r\n", result);
    }
}

static void motion_stop() {
    motion_command_t cmd = { WAITING, 0, 0, 0, 0 };
    dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send motion command: %d\r\n", result);
    }
}

static void go_to_next_algorithm_point() {
    std::vector<double> next = AlgorithmNextPoint();
    if (next.size() < 2) {
        SerialDebug.printf("[CTRL] Mowing complete, returning to IDLE\r\n");
        stop_motors();
        motor_main_off();
        robot_state = ROBOT_STATE_IDLE;
        return;
    }

    double next_x = next.at(0);
    double next_y = next.at(1);

    AlgorithmMotionSetTargetPoint(next_x, next_y);
}

static void enter_manual_mode() {
    if (robot_state != ROBOT_STATE_IDLE) {
        SerialDebug.printf("[CTRL] Manual ON rejected: must be IDLE (state=%d)\r\n", robot_state);
        return;
    }

    SerialDebug.println("[CTRL] Manual mode ON");

    // Defensive: halt motion_task and motors before handing control to manual input
    motion_stop();
    stop_motors();

    robot_state = ROBOT_STATE_MANUAL;
    MANUAL_MODE_ACTIVE = true;
}

static void exit_manual_mode() {
    if (robot_state != ROBOT_STATE_MANUAL) {
        SerialDebug.printf("[CTRL] Manual OFF rejected: not in MANUAL (state=%d)\r\n", robot_state);
        return;
    }

    SerialDebug.println("[CTRL] Manual mode OFF");
    stop_motors();
    MANUAL_MOTORS_STOPPED = true;

    MANUAL_MODE_ACTIVE = false;
    robot_state = ROBOT_STATE_IDLE;
}

// -------------------------------------------------------- DATUM CAPTURE ---------------------------------------------------------------

static void finish_datum_capture(int result) {
    stop_motors();
    DATUM_CAPTURE_ACTIVE       = false;
    LAST_DATUM_CAPTURE_RESULT  = result;
    robot_state                = ROBOT_STATE_IDLE;
    SerialDebug.printf("[CTRL] Datum capture finished: result=%d\r\n", result);
}

static void start_datum_capture() {
    if (robot_state != ROBOT_STATE_IDLE) {
        SerialDebug.printf("[CTRL] Datum capture rejected: must be IDLE (state=%d)\r\n", robot_state);
        LAST_DATUM_CAPTURE_RESULT = DATUM_RESULT_FAIL_STATE;
        return;
    }
    if (!gps_valid_ctrl || (latest_gps_ctrl.accuracy * 1000.0f) > (float)GPS_ACC_THRESHOLD) {
        SerialDebug.printf("[CTRL] Datum capture rejected: GPS accuracy bad (acc=%.4f m)\r\n",
                           latest_gps_ctrl.accuracy);
        LAST_DATUM_CAPTURE_RESULT = DATUM_RESULT_FAIL_ACCURACY;
        return;
    }

    SerialDebug.printf("[CTRL] Datum capture: starting (initial lat=%.7f lon=%.7f)\r\n",
                       latest_gps_ctrl.latitude, latest_gps_ctrl.longitude);

    datum_capture_start_lat   = latest_gps_ctrl.latitude;
    datum_capture_start_lon   = latest_gps_ctrl.longitude;
    datum_capture_start_ms    = millis();
    LAST_DATUM_CAPTURE_RESULT = DATUM_RESULT_NONE;
    DATUM_CAPTURE_ACTIVE      = true;
    robot_state               = ROBOT_STATE_CAPTURING_DATUM;

    // Drive forward; we publish /motor MOVE directly, motion_task is not involved.
    motor_data_t cmd = { MOTOR_MOVE, DATUM_CAPTURE_DRIVE_SPEED, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to start datum drive: %d\r\n", result);
    }
}

static void tick_datum_capture() {
    if (robot_state != ROBOT_STATE_CAPTURING_DATUM) return;
    if ((int32_t)(millis() - datum_capture_start_ms) < DATUM_CAPTURE_DRIVE_DURATION_MS) return;

    // Drive complete -- stop first, then evaluate.
    stop_motors();

    if (!gps_valid_ctrl || (latest_gps_ctrl.accuracy * 1000.0f) > (float)GPS_ACC_THRESHOLD) {
        SerialDebug.printf("[CTRL] Datum capture FAIL: accuracy degraded (acc=%.4f m)\r\n",
                           latest_gps_ctrl.accuracy);
        finish_datum_capture(DATUM_RESULT_FAIL_ACCURACY);
        return;
    }

    // Convert GPS displacement to local meters in ENU.
    double dlat     = latest_gps_ctrl.latitude  - datum_capture_start_lat;
    double dlon     = latest_gps_ctrl.longitude - datum_capture_start_lon;
    double dy_north = dlat * 111111.0;
    double dx_east  = dlon * 111111.0 * cos(datum_capture_start_lat * M_PI / 180.0);
    double displacement = sqrt(dx_east * dx_east + dy_north * dy_north);

    if (displacement < DATUM_CAPTURE_MIN_DISPLACEMENT_M) {
        SerialDebug.printf("[CTRL] Datum capture FAIL: displacement too small (%.3f m)\r\n", displacement);
        finish_datum_capture(DATUM_RESULT_FAIL_DISPLACEMENT);
        return;
    }

    // Yaw in ENU: 0 = +east, π/2 = +north.
    float yaw = (float)atan2(dy_north, dx_east);

    // Commit datum.
    BASE_LAT = datum_capture_start_lat;
    BASE_LON = datum_capture_start_lon;
    BASE_YAW = yaw;

    // Fixed-distance base exit point in the heading direction.
    BASE_EXIT_X_CM = (int64_t)round((double)DATUM_BASE_EXIT_DISTANCE_CM * cos(yaw));
    BASE_EXIT_Y_CM = (int64_t)round((double)DATUM_BASE_EXIT_DISTANCE_CM * sin(yaw));

    CONFIG_DATUM = true;
    CONFIG_PATH  = false;   // any prior path is invalid against the new datum
    SaveConfiguration();

    // Publish into the internal DDS topic; main loop forwards to ROS2 via /esp/datum.
    datum_data_t datum = { BASE_LAT, BASE_LON, BASE_YAW };
    dds_result_t pub_result = DDS_PUBLISH("/datum/set", datum);
    if (pub_result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to publish datum: %d\r\n", pub_result);
    }

    SerialDebug.printf("[CTRL] Datum captured: lat=%.7f lon=%.7f yaw=%.4f rad (disp=%.3f m, exit=%lld,%lld cm)\r\n",
                       BASE_LAT, BASE_LON, BASE_YAW, displacement,
                       (long long)BASE_EXIT_X_CM, (long long)BASE_EXIT_Y_CM);

    finish_datum_capture(DATUM_RESULT_OK);
}

// -------------------------------------------------------- STATE MACHINE ---------------------------------------------------------------

static void start_mowing() {
    if (robot_state != ROBOT_STATE_IDLE && robot_state != ROBOT_STATE_CHARGING) {
        SerialDebug.printf("[CTRL] Cannot start mowing from state: %d\r\n", robot_state);
        return;
    }
    if (!CONFIG_DATUM) {
        SerialDebug.printf("[CTRL] Cannot start mowing: datum not set\r\n");
        return;
    }
    if (!CONFIG_PATH) {
        SerialDebug.printf("[CTRL] Cannot start mowing: path not set\r\n");
        return;
    }

    SerialDebug.printf("[CTRL] Starting mowing\r\n");
    robot_state = ROBOT_STATE_MOWING;
    AlgorithmSeedStartPosition();
    go_to_next_algorithm_point();
}

static void pause_mowing() {
    if (robot_state != ROBOT_STATE_MOWING) {
        SerialDebug.printf("[CTRL] Cannot pause from state: %d\r\n", robot_state);
        return;
    }
    
    SerialDebug.printf("[CTRL] Pausing mowing\r\n");
    stop_motors();
    motion_stop();
    robot_state = ROBOT_STATE_PAUSED;
}

static void resume_mowing() {
    if (robot_state != ROBOT_STATE_PAUSED) {
        SerialDebug.printf("[CTRL] Cannot resume from state: %d\r\n", robot_state);
        return;
    }
    
    SerialDebug.printf("[CTRL] Resuming mowing\r\n");
    robot_state = ROBOT_STATE_MOWING;
    go_to_next_algorithm_point();
}

static void return_to_base() {
    if (robot_state == ROBOT_STATE_RETURNING_TO_BASE) {
        return; // Already returning
    }
    
    SerialDebug.printf("[CTRL] Returning to base\r\n");
    stop_motors();
    robot_state = ROBOT_STATE_RETURNING_TO_BASE;
    go_to_base();
}

static void handle_charging_complete() {
    SerialDebug.printf("[CTRL] Charging complete, leaving charging station\r\n");
    robot_state = ROBOT_STATE_LEAVING_CHARGING_STATION;
    go_to_base_exit();
}

static void handle_docking_complete() {
    SerialDebug.printf("[CTRL] Docked at charging station, starting charge\r\n");
    robot_state = ROBOT_STATE_CHARGING;
    // In a real system, you'd monitor battery level and call handle_charging_complete() when done
}

// -------------------------------------------------------- EVENT HANDLERS ---------------------------------------------------------------

static void handle_motion_done() {
    switch (robot_state) {
        case ROBOT_STATE_MOWING:
            go_to_next_algorithm_point();
            break;

        case ROBOT_STATE_RETURNING_TO_BASE:
            SerialDebug.printf("[CTRL] Reached base, entering charging station\r\n");
            robot_state = ROBOT_STATE_ENTERING_CHARGING_STATION;
            handle_docking_complete();
            break;

        case ROBOT_STATE_LEAVING_CHARGING_STATION:
            SerialDebug.printf("[CTRL] Left base, resuming mowing\r\n");
            robot_state = ROBOT_STATE_MOWING;
            go_to_next_algorithm_point();
            break;

        default:
            break;
    }
}

static void handle_obstacle() {
    if (robot_state != ROBOT_STATE_MOWING) return;

    SerialDebug.printf("[CTRL] Obstacle hit — skipping line\r\n");
    stop_motors();
    AlgorithmAbort(false);
    go_to_next_algorithm_point();    // continue from next point
}

static void handle_rain() {
    if (robot_state != ROBOT_STATE_MOWING) return;

    SerialDebug.printf("[CTRL] Rain detected — returning to base\r\n");
    return_to_base();
}

static void handle_low_battery() {
    if (robot_state != ROBOT_STATE_MOWING) return;

    SerialDebug.printf("[CTRL] Low battery — returning to base\r\n");
    return_to_base();
}

static void handle_battery_charged() {
    if (robot_state == ROBOT_STATE_CHARGING) {
        handle_charging_complete();
    }
}

// -------------------------------------------------------- DDS CALLBACKS ---------------------------------------------------------------

static void controller_signal_callback(dds_callback_context_t* context) {
    controller_signal_t* signal = (controller_signal_t*)context->message_data.data;

    switch (signal->signal) {
        case SIGNAL_MOTION_DONE:
            handle_motion_done();
            break;
        case SIGNAL_LOW_BATTERY:
            handle_low_battery();
            break;
        case SIGNAL_RAIN:
            handle_rain();
            break;
        case SIGNAL_OBSTACLE:
            handle_obstacle();
            break;
        case SIGNAL_BATTERY_CHARGED:
            handle_battery_charged();
            break;
        case SIGNAL_MAIN_CHARGING_START:
            SerialDebug.printf("[CTRL] Main charging started signal received\r\n");
            break;
        case SIGNAL_START_MOWING:
            start_mowing();
            break;
        case SIGNAL_MANUAL_ON:
            enter_manual_mode();
            break;
        case SIGNAL_MANUAL_OFF:
            exit_manual_mode();
            break;
        case SIGNAL_START_DATUM_CAPTURE:
            start_datum_capture();
            break;
        default:
            SerialDebug.printf("[CTRL] Unknown signal: %d\r\n", signal->signal);
    }
}

static void gps_callback(dds_callback_context_t* context) {
    gps_data_t* data = (gps_data_t*)context->message_data.data;
    latest_gps_ctrl = *data;
    gps_valid_ctrl  = (data->accuracy > 0 && data->accuracy < 999);
}

// -------------------------------------------------------- TASK ---------------------------------------------------------------

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }

void controller_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 200 * 1000); // 200 ms

    // ------- THREAD SETUP CODE START -------

    dds_result_t result;
    result = DDS_SUBSCRIBE("/controller/signal", controller_signal_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    result = DDS_SUBSCRIBE("/gps", gps_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("GPS topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    // ------- THREAD SETUP CODE END -------

    vTaskDelay(1500); // Allow some time for system stabilization before starting main loop

    robot_state = ROBOT_STATE_IDLE;

    if (!CONFIG_DATUM) {
        SerialDebug.printf("[CTRL] Datum not set. Waiting for Bluetooth setup.\r\n");
    }
    if (!CONFIG_PATH) {
        SerialDebug.printf("[CTRL] Path not set. Waiting for path setup.\r\n");
    }
    if (CONFIG_DATUM && CONFIG_PATH) {
        SerialDebug.printf("[CTRL] Datum + path confirmed, ready for MOWER/START.\r\n");
    }
    
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

            tick_datum_capture();
            
            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}