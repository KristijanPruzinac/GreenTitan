#include "gps_task.h"
#include "../hil/hil.h"

static HardwareSerial SerialGPS(2);

// Receiver byte source: the UART, or the simulator stream in HIL builds
static int gps_available() { return HIL_ENABLED ? hil_gnss_available() : SerialGPS.available(); }
static int gps_read_byte() { return HIL_ENABLED ? hil_gnss_read() : SerialGPS.read(); }

//GPS is outputing UBX NAV PVT messages
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };
#define UBX_CLASS_NAV  0x01
#define UBX_ID_NAV_PVT 0x07
#define UBX_MAX_PAYLOAD 100

// NAV-PVT payload, first 84 bytes (same layout on M8/M9/F9, which send 92)
struct NAV_PVT {
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
static_assert(sizeof(NAV_PVT) == 84, "NAV_PVT layout");

NAV_PVT pvt;

// Feeds one byte to the UBX frame parser. Returns true when a checksum-valid NAV-PVT
// frame has completed; only then is pvt overwritten.
static bool ubx_parse(unsigned char c) {
  static unsigned char frame[4 + UBX_MAX_PAYLOAD];   // cls, id, len (LE), payload
  static int fpos = 0;
  static unsigned short len = 0;
  static unsigned char ck_a = 0, ck_b = 0;

  if (fpos < 2) {
    fpos = (c == UBX_HEADER[fpos]) ? fpos + 1 : (c == UBX_HEADER[0] ? 1 : 0);
    ck_a = ck_b = 0;
    return false;
  }
  int i = fpos - 2;                // index into frame[]
  if (i < 4 || i < 4 + len) {      // header + payload: checksummed
    frame[i] = c;
    ck_a += c; ck_b += ck_a;
    fpos++;
    if (i == 3) {
      len = frame[2] | (frame[3] << 8);
      if (len > UBX_MAX_PAYLOAD) fpos = 0;   // too long for us (or garbage)
    }
    return false;
  }
  if (i == 4 + len) {              // CK_A
    if (c != ck_a) fpos = 0; else fpos++;
    return false;
  }
  fpos = 0;                        // CK_B
  if (c != ck_b) return false;
  if (frame[0] != UBX_CLASS_NAV || frame[1] != UBX_ID_NAV_PVT || len < sizeof(NAV_PVT)) return false;
  memcpy(&pvt, &frame[4], sizeof(NAV_PVT));
  return true;
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

  bool new_frame = false;
  while ( gps_available() ) {
    if ( ubx_parse(gps_read_byte()) ) new_frame = true;
  }

  //Publish GPS data once per new valid frame
  if (new_frame && pvt.hAcc > 0 && pvt.fixType >= 3) { // 3 = 3D fix, 4 = GNSS+DeadReckoning
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
  if (HIL_ENABLED) { hil_init(); return true; }  // stream only fills once the executor spins

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