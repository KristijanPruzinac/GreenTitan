#include "bluetooth_task.h"
#include "../configuration/configuration.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

// Latest GPS fix cached from DDS
static gps_data_t latest_gps = {0, 0, 0, 999};
static bool gps_valid = false;

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
                if (!gps_valid || latest_gps.accuracy > GPS_ACC_THRESHOLD) {
                    bt_send("GPS/CAPTURE/BASE/FAIL/ACCURACY");
                    return;
                }
                BASE_LAT = latest_gps.latitude;
                BASE_LON = latest_gps.longitude;
                CONFIG_DATUM = true;
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