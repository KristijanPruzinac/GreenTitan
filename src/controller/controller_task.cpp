#include "controller_task.h"
#include "../algorithm/algorithm.h"

static int robot_state = ROBOT_STATE_IDLE;

// External globals from algorithm
extern String algorithmTarget;
extern String algorithmMode;
extern bool algorithmAbortFull;

// -------------------------------------------------------- HELPERS ---------------------------------------------------------------

static void go_to_base() {
    motion_command_t cmd = { MOVING, 0, 0, BASE_LON, BASE_LAT };
    dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send command: %d\r\n", result);
    }
}

static void go_to_base_exit() {
    motion_command_t cmd = { MOVING, 0, 0, BASE_EXIT_LON, BASE_EXIT_LAT };
    dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send command: %d\r\n", result);
    }
}

static void go_to_next_algorithm_point() {
    std::vector<double> next = AlgorithmNextPoint();
    if (next.size() < 2) {
        SerialDebug.printf("[CTRL] Algorithm returned invalid point\r\n");
        return;
    }
    
    double next_x = next.at(0);
    double next_y = next.at(1);

    AlgorithmMotionSetTargetPoint(next_x, next_y);
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

// -------------------------------------------------------- STATE MACHINE ---------------------------------------------------------------

static void start_mowing() {
    if (robot_state != ROBOT_STATE_IDLE && robot_state != ROBOT_STATE_CHARGING) {
        SerialDebug.printf("[CTRL] Cannot start mowing from state: %d\r\n", robot_state);
        return;
    }
    
    SerialDebug.printf("[CTRL] Starting mowing\r\n");
    robot_state = ROBOT_STATE_MOWING;
    
    // Start from base exit point if defined
    if (BASE_EXIT_LON != 0 || BASE_EXIT_LAT != 0) {
        motion_command_t cmd = { MOVING, 0, 0, BASE_EXIT_LON, BASE_EXIT_LAT };
        dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
        if (result != DDS_SUCCESS) {
            SerialDebug.printf("[CTRL] Failed to send command: %d\r\n", result);
        }
    } else {
        go_to_next_algorithm_point();
    }
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
    algorithm_command_t cmd = { CMD_ALGO_ABORT, false };
    dds_result_t result = DDS_PUBLISH("/algorithm/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[CTRL] Failed to send algorithm command: %d\r\n", result);
    }
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
    result = DDS_SUBSCRIBE("/controller/signal", controller_signal_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("Topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    // ------- THREAD SETUP CODE END -------

    vTaskDelay(500);
    
    // Auto-start mowing if outlines are loaded? Or wait for condition?
    // For now, we'll start mowing automatically after a short delay
    // You can change this behavior based on your needs
    vTaskDelay(2000); // Wait for system to stabilize
    start_mowing();
    
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
            
            // You can add periodic checks here if needed
            // For example: check battery level periodically
            
            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}