#include "gps_task.h"

static HardwareSerial SerialGPS(2);

//GPS is outputing UBX NAV POSLLH messages
static const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

static NAV_POSLLH posllh;

bool init_gps(){
  if (SIMULATION_ENABLED) return true;

  SerialGPS.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialGPS.setTimeout(COMMUNICATION_TIMEOUT);

  //TODO: Implement check for GPS initializing properly
  return true;
}

static void calc_checksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

static void gps_read() {
  if (SIMULATION_ENABLED) {
    const double METERS_PER_LAT = 111111.0;

    gps_data_t gpsData = {
        BASE_LAT + (odom_y / METERS_PER_LAT) + gaussian_noise(0.0000001),  // ~3cm
        BASE_LON + (odom_x / (METERS_PER_LAT * cosf(BASE_LAT * M_PI / 180.0))) + gaussian_noise(0.0000001),  // ~3cm
        0.0,
        0.014 + gaussian_noise(0.005),
    };
    dds_result_t result = DDS_PUBLISH("/gps", gpsData);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("GPS Topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }
    return;
  }

  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( SerialGPS.available() ) {
    byte c = SerialGPS.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&posllh))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calc_checksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          //returnVal = true;
          break;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }

  //Publish GPS data if accuracy is valid
  if (posllh.hAcc > 0) {
    gps_data_t gpsData = {
        posllh.lat / 1.0e7,
        posllh.lon / 1.0e7,
        posllh.height / 1000.0,
        posllh.hAcc / 1000.0,   // horizontal accuracy in meters
    };

    dds_result_t result = DDS_PUBLISH("/gps", gpsData);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("GPS Topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }
  }
}

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void gps_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 1000 * 1000 / GPS_UPDATE_FREQUENCY);

    // ------- THREAD SETUP CODE START -------

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

            gps_read();

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}