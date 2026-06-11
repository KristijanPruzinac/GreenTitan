#include "gps_task.h"

static HardwareSerial SerialGPS(2);

//GPS is outputing UBX NAV PVT messages
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)
  
  unsigned short year;         // Year (UTC) 
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char minute;        // Minute of hour, range 0..59 (UTC)
  unsigned char second;        // Seconds of minute, range 0..60 (UTC)
  char valid;                  // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  char flags;                  // Fix Status Flags
  unsigned char reserved1;     // reserved
  unsigned char numSV;         // Number of satellites used in Nav Solution
  
  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)
  
  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long heading;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headingAcc;    // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision
  short reserved2;             // Reserved
  unsigned long reserved3;     // Reserved
};

NAV_PVT pvt;

static void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

static void gps_read() {
  if (SIMULATION_ENABLED) {
    // Simulated GPS – generate realistic heading and speed based on simulated odometry
    const double METERS_PER_LAT = 111111.0;
    double simulated_heading = atan2(odom_y, odom_x);  // rad, 0 = east
    double simulated_speed = sqrt(odom_x*odom_x + odom_y*odom_y) / 0.1; // rough

    gps_data_t gpsData = {
        .latitude  = BASE_LAT + (odom_y / METERS_PER_LAT) + gaussian_noise(0.0000001),
        .longitude = BASE_LON + (odom_x / (METERS_PER_LAT * cos(BASE_LAT * M_PI / 180.0))) + gaussian_noise(0.0000001),
        .altitude  = 0.0,
        .accuracy  = 0.014 + gaussian_noise(0.005),
        .heading   = normalize_angle(simulated_heading + gaussian_noise(0.02)),
        .heading_accuracy = 0.05,   // ~3 deg
        .speed     = fabs(simulated_speed) + gaussian_noise(0.05),
        .speed_accuracy = 0.1,
    };

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    gpsData.timestamp_sec = ts.tv_sec;
    gpsData.timestamp_nsec = ts.tv_nsec;

    dds_result_t result = DDS_PUBLISH("/gps", gpsData);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("GPS Topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
    }
    return;
  }

  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

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
        ((unsigned char*)(&pvt))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          break; // returnVal = true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }

  //Publish GPS data if accuracy is valid
  if (pvt.hAcc > 0 && pvt.fixType >= 3) { // 3 = 3D fix, 4 = GNSS+DeadReckoning
    double heading_deg = (double)pvt.heading / 1e5;
    double heading_rad = heading_deg * M_PI / 180.0;

    // UBX heading is NED (0=N, 90=E, CW). Convert to ENU (0=E, 90=N, CCW).
    heading_rad = M_PI / 2.0 - heading_rad;
    heading_rad = normalize_angle(heading_rad);

    gps_data_t gpsData = {
        .latitude  = (double)pvt.lat / 1e7,
        .longitude = (double)pvt.lon / 1e7,
        .altitude  = (double)pvt.hMSL / 1000.0,
        .accuracy  = (double)pvt.hAcc / 1000.0,
        .heading   = heading_rad,
        .heading_accuracy = (double)pvt.headingAcc / 1e5 * M_PI / 180.0,
        .speed     = (double)pvt.gSpeed / 1000.0,
        .speed_accuracy = (double)pvt.sAcc / 1000.0,
        .vel_north = (double)pvt.velN / 1000.0,
        .vel_east  = (double)pvt.velE / 1000.0,
        .vel_accuracy = (double)pvt.sAcc / 1000.0,   // same as speed accuracy
    };

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    gpsData.timestamp_sec = ts.tv_sec;
    gpsData.timestamp_nsec = ts.tv_nsec;

    if (TASKS_STARTED) {
        dds_result_t result = DDS_PUBLISH("/gps", gpsData);
        if (result != DDS_SUCCESS) {
            SerialDebug.printf("GPS Topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
        }

        SerialDebug.printf("GPS: lat=%.7f lon=%.7f alt=%.2f acc=%.2f heading=%.1f° speed=%.2f m/s\r\n",
                       gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.accuracy,
                       heading_deg, gpsData.speed);
    }
  }
}

bool init_gps(){
  if (SIMULATION_ENABLED) return true;

  SerialGPS.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialGPS.setTimeout(COMMUNICATION_TIMEOUT);

  // Clear so a stale value from before can't fool us
  memset(&pvt, 0, sizeof(pvt));

  // Let the parser pump for ~1 second
  unsigned long start = millis();
  while (millis() - start < 1000) {
      gps_read();

      if (pvt.hAcc > 0) {
          return true;
      }

      delay(10);
  }

  return false;
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