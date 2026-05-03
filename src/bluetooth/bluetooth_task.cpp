#include "bluetooth_task.h"
#include "../configuration/configuration.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

// Latest GPS fix cached from DDS
static gps_data_t latest_gps = {0, 0, 0, 999};
static bool gps_valid = false;
static int64_t current_x_cm = 0;
static int64_t current_y_cm = 0;
static bool fused_pose_received = false;

// Manual mode — when true, BT motor commands are honored
static bool manual_mode = false;

// Declared in main.cpp
extern void publish_datum();
extern bool CONFIG_DATUM;

bool InitBluetooth() {
    if (!SerialBT.begin("GreenTitan")) return false;
    SerialBT.setTimeout(COMMUNICATION_TIMEOUT);
    return true;
}

static void bt_send(String msg) {
    SerialBT.print(msg);
}

static bool capture_preconditions_ok(const String& fail_prefix) {
    if (!CONFIG_DATUM) {
        bt_send(fail_prefix + "/FAIL/NO_DATUM");
        return false;
    }
    if (!fused_pose_received) {
        bt_send(fail_prefix + "/FAIL/NOT_READY");
        return false;
    }
    return true;
}

static void handle_message(String MessageGroup, String ActionGroup, String DataGroup) {
    if (MessageGroup == "GPS") {
        if (ActionGroup == "GET") {
            if (DataGroup == "ACCURACY") {
                bt_send("GPS/ACCURACY/" + String(latest_gps.accuracy));
            }
            else if (DataGroup == "POS") {
                bt_send("GPS/POS/" + String(latest_gps.latitude, 7) + "," + String(latest_gps.longitude, 7));
            }
        }
        else if (ActionGroup == "CAPTURE") {
            if (DataGroup == "BASE") {
                // Allow re-pairing only before initial pairing or while in manual mode
                if (CONFIG_DATUM && !manual_mode) {
                    bt_send("GPS/CAPTURE/BASE/FAIL/BUSY");
                    return;
                }
                if (!gps_valid || latest_gps.accuracy > GPS_ACC_THRESHOLD) {
                    bt_send("GPS/CAPTURE/BASE/FAIL/ACCURACY");
                    return;
                }
                BASE_LAT = latest_gps.latitude;
                BASE_LON = latest_gps.longitude;
                CONFIG_DATUM = true;
                CONFIG_PATH = false;  // re-pairing invalidates path
                SaveConfiguration();

                datum_data_t datum = { BASE_LAT, BASE_LON };
                dds_result_t result = DDS_PUBLISH("/datum/set", datum);
                if (result != DDS_SUCCESS) {
                    SerialDebug.printf("[BT] Failed to publish datum: %d\r\n", result);
                    bt_send("GPS/CAPTURE/BASE/FAIL/PUBLISH");
                }
                else {
                    bt_send("GPS/CAPTURE/BASE/OK");
                    SerialDebug.printf("Base captured: lat=%.7f lon=%.7f\r\n", BASE_LAT, BASE_LON);
                }
            }
        }
    }
    else if (MessageGroup == "MOWER") {
        if (ActionGroup == "START") {
            controller_signal_t signal = { SIGNAL_START_MOWING };
            dds_result_t result = DDS_PUBLISH("/controller/signal", signal);
            if (result != DDS_SUCCESS) {
                bt_send("MOWER/START/FAIL");
            } else {
                bt_send("MOWER/START/OK");
            }
        }
        else if (ActionGroup == "MANUAL") {
            if (DataGroup == "ON") {
                manual_mode = true;
                controller_signal_t signal = { SIGNAL_MANUAL_ON };
                DDS_PUBLISH("/controller/signal", signal);
                bt_send("MOWER/MANUAL/ON/OK");
            }
            else if (DataGroup == "OFF") {
                manual_mode = false;
                controller_signal_t signal = { SIGNAL_MANUAL_OFF };
                DDS_PUBLISH("/controller/signal", signal);
                // Stop motors on exit from manual mode
                motor_data_t stop = { MOTOR_STOP, 0.0f, 0.0f };
                DDS_PUBLISH("/motor", stop);
                bt_send("MOWER/MANUAL/OFF/OK");
            }
        }
        else if (ActionGroup == "MOVE") {
            // Format: MOWER/MOVE/<lin>,<ang>  e.g. MOWER/MOVE/0.3,0.0
            if (!manual_mode) {
                bt_send("MOWER/MOVE/FAIL/NOT_MANUAL");
                return;
            }
            int comma = DataGroup.indexOf(',');
            if (comma < 0) {
                bt_send("MOWER/MOVE/FAIL/PARSE");
                return;
            }
            float lin = DataGroup.substring(0, comma).toFloat();
            float ang = DataGroup.substring(comma + 1).toFloat();
            motor_data_t cmd = { MOTOR_MOVE, lin, ang };
            dds_result_t result = DDS_PUBLISH("/motor", cmd);
            bt_send(result == DDS_SUCCESS ? "MOWER/MOVE/OK" : "MOWER/MOVE/FAIL");
        }
        else if (ActionGroup == "CAPTURE") {
            if (DataGroup == "EXIT") {
                if (!capture_preconditions_ok("MOWER/CAPTURE/EXIT")) return;
                AlgorithmCaptureBaseExitPoint(current_x_cm, current_y_cm);
                SaveConfiguration();
                bt_send("MOWER/CAPTURE/EXIT/OK/" + String((int32_t)current_x_cm) + "," + String((int32_t)current_y_cm));
            }
            else if (DataGroup == "POINT") {
                if (!capture_preconditions_ok("MOWER/CAPTURE/POINT")) return;
                AlgorithmCaptureNewPoint();
                bt_send("MOWER/CAPTURE/POINT/OK/" + String((int32_t)current_x_cm) + "," + String((int32_t)current_y_cm));
            }
            else if (DataGroup == "OUTLINE") {
                // Start a new outline (no preconditions — purely structural)
                AlgorithmCaptureNewOutline();
                bt_send("MOWER/CAPTURE/OUTLINE/OK");
            }
            else if (DataGroup == "START") {
                // Begin a new path recording session (clears previous)
                AlgorithmCaptureStart();
                CONFIG_PATH = false;
                SaveConfiguration();
                bt_send("MOWER/CAPTURE/START/OK");
            }
            else if (DataGroup == "END") {
                if (AlgorithmCaptureEnd()) {
                    CONFIG_PATH = true;
                    SaveConfiguration();
                    bt_send("MOWER/CAPTURE/END/OK");
                } else {
                    bt_send("MOWER/CAPTURE/END/FAIL");
                }
            }
        }
    }
}

static void parse_bluetooth() {
    while (SerialBT.available()) {
        String message = SerialBT.readStringUntil('<');
        if (message.length() == 0) break;

        // Expect format: MessageGroup/ActionGroup/DataGroup
        String parts[3] = {"", "", ""};
        int part = 0;
        for (int i = 0; i < message.length() && part < 3; i++) {
            if (message[i] == '/') { part++; continue; }
            parts[part] += message[i];
        }

        handle_message(parts[0], parts[1], parts[2]);
    }
}

static void gps_callback(dds_callback_context_t* context) {
    gps_data_t* data = (gps_data_t*)context->message_data.data;
    latest_gps = *data;
    gps_valid = (data->accuracy > 0 && data->accuracy < 999);
}

static void fused_pose_callback(dds_callback_context_t* context) {
    fused_pose_data_t* data = (fused_pose_data_t*)context->message_data.data;
    current_x_cm = (long long)(data->x * 100.0);
    current_y_cm = (long long)(data->y * 100.0);
    fused_pose_received = true;
    AlgorithmSetCurrentPosition(current_x_cm, current_y_cm);
}

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }

void bluetooth_task(void* pvParameters) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();

    esp_timer_create_args_t timer_args = { .callback = &thread_timer_callback, .arg = NULL };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 100 * 1000); // 100ms

    dds_result_t result = DDS_SUBSCRIBE("/gps", gps_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[BT] GPS subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }

    result = DDS_SUBSCRIBE("/fused_pose", fused_pose_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[BT] Fused pose subscribe failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }

    vTaskDelay(500);

    while (1) {
        uint32_t notification_value;
        xTaskNotifyWait(0x00, 0xFF, &notification_value, portMAX_DELAY);

        if (notification_value & DDS_NOTIFY_BIT) {
            DDS_TAKE_MUTEX(&thread_context);
            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
        if (notification_value & THREAD_NOTIFY_BIT) {
            DDS_TAKE_MUTEX(&thread_context);
            parse_bluetooth();
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}