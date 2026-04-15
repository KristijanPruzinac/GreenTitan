#include "controller_task.h"

static int robot_state = ROBOT_STATE_IDLE;

// -------------------------------------------------------- HELPERS ---------------------------------------------------------------

static void go_to_base() {
    motion_command_t cmd = { MOVING, BASE_LON, BASE_LAT };
    dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send command: %d\r\n", result);
    }
}

static void go_to_base_exit() {
    motion_command_t cmd = { MOVING, BASE_EXIT_LON, BASE_EXIT_LAT };
    dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send command: %d\r\n", result);
    }
}

static void go_to_next_algorithm_point() {
    point_t next = AlgorithmNextPoint();
    if (next.x == 0 && next.y == 0) {
        // Algorithm finished — mowing complete, return to base
        SerialDebug.printf("[CTRL] Mowing complete, returning to base\r\n");
        robot_state = ROBOT_STATE_RETURNING_TO_BASE;
        go_to_base();
        return;
    }
    motion_command_t cmd = { MOVING, next.x, next.y };
    dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send command: %d\r\n", result);
    }
    else {
        SerialDebug.printf("[CTRL] Moving to next point: %f, %f\r\n", next.x, next.y);
    }
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

// -------------------------------------------------------- EVENT HANDLERS ---------------------------------------------------------------

static void handle_motion_done() {
    switch (robot_state) {
        case ROBOT_STATE_MOWING:
            go_to_next_algorithm_point();
            break;

        case ROBOT_STATE_RETURNING_TO_BASE:
            SerialDebug.printf("[CTRL] Reached base, entering charging station\r\n");
            robot_state = ROBOT_STATE_ENTERING_CHARGING_STATION;
            // TODO: trigger docking sequence here
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
    AlgorithmAbort(false);           // skip current line only
    go_to_next_algorithm_point();    // continue from next point
}

static void handle_rain() {
    if (robot_state != ROBOT_STATE_MOWING) return;

    SerialDebug.printf("[CTRL] Rain detected — returning to base\r\n");
    stop_motors();
    robot_state = ROBOT_STATE_RETURNING_TO_BASE;
    go_to_base();
}

static void handle_low_battery() {
    if (robot_state != ROBOT_STATE_MOWING) return;

    SerialDebug.printf("[CTRL] Low battery — returning to base\r\n");
    stop_motors();
    robot_state = ROBOT_STATE_RETURNING_TO_BASE;
    go_to_base();
}

static void handle_command(controller_command_t* cmd) {
    switch (cmd->command) {
        case CMD_START_MOWING:
            if (robot_state != ROBOT_STATE_IDLE) break;
            SerialDebug.printf("[CTRL] Starting mowing\r\n");
            robot_state = ROBOT_STATE_MOWING;
            go_to_next_algorithm_point();
            break;

        case CMD_PAUSE:
            if (robot_state != ROBOT_STATE_MOWING) break;
            SerialDebug.printf("[CTRL] Paused\r\n");
            stop_motors();
            robot_state = ROBOT_STATE_PAUSED;
            break;

        case CMD_RESUME:
            if (robot_state != ROBOT_STATE_PAUSED) break;
            SerialDebug.printf("[CTRL] Resuming\r\n");
            robot_state = ROBOT_STATE_MOWING;
            go_to_next_algorithm_point();
            break;

        case CMD_RETURN_TO_BASE:
            SerialDebug.printf("[CTRL] Manual return to base\r\n");
            stop_motors();
            robot_state = ROBOT_STATE_RETURNING_TO_BASE;
            go_to_base();
            break;

        case CMD_START_RECORDING:
            break;

        case CMD_CAPTURE_POINT:
            break;

        case CMD_END_RECORDING:
            break;

        case CMD_ABORT:
            break;

        default:
            SerialDebug.printf("[CTRL] Unknown command: %d\r\n", cmd->command);
            break;
    }
}

// -------------------------------------------------------- DDS CALLBACKS ---------------------------------------------------------------

static void controller_topic_callback(dds_callback_context_t* context) {
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
        default:
            SerialDebug.printf("[CTRL] Unknown signal: %d\r\n", signal->signal);
    }
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
    result = DDS_SUBSCRIBE("/controller/signal", controller_topic_callback, &thread_context);
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

            if (robot_state == ROBOT_STATE_IDLE) {
                controller_command_t cmd = { CMD_START_MOWING };
                handle_command(&cmd);
            }

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}