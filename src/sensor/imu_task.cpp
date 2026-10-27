#include "imu_task.h"
#include "../hil/hil.h"

//Calibration
static double gyro_x_offset = 0;
static double gyro_y_offset = 0;
static double gyro_z_offset = 0;

static double acc_x_offset = 0;
static double acc_y_offset = 0;
static double acc_z_offset = 0;

static bool calibration_completed = false;

//Data
static sensors_event_t imu_acc, imu_gyro, imu_temp;

static Adafruit_MPU6050 mpu;

bool init_imu(){
  if (SIMULATION_ENABLED) {
    calibration_completed = true;
    return true;
  }
  if (HIL_ENABLED) return true;

  if (!mpu.begin()) {
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  return true;
}

// Gyro bias from the first samples at rest. Runs on the same imu_read() stream in every
// build (real, HIL), so the firmware sees identical behaviour. Accel is not offset: that
// would also remove gravity and tilt.
#define IMU_CALIBRATION_SAMPLES  (3 * IMU_UPDATE_FREQUENCY)   // 3 s
#define IMU_CALIBRATION_MAX_STD  0.01                         // rad/s; above this the mower moved

static bool imu_calibrate_step(){
  static int n = 0;
  static double sum[3], sum_sq[3];
  double g[3] = { imu_gyro.gyro.x, imu_gyro.gyro.y, imu_gyro.gyro.z };
  for (int i = 0; i < 3; i++) { sum[i] += g[i]; sum_sq[i] += g[i] * g[i]; }
  if (++n < IMU_CALIBRATION_SAMPLES) return false;

  double mean[3], max_std = 0;
  for (int i = 0; i < 3; i++) {
    mean[i] = sum[i] / n;
    max_std = fmax(max_std, sqrt(fmax(sum_sq[i] / n - mean[i] * mean[i], 0.0)));
    sum[i] = sum_sq[i] = 0;
  }
  n = 0;
  if (max_std > IMU_CALIBRATION_MAX_STD) {
    SerialDebug.printf("[IMU] Moving during gyro calibration (std %.3f rad/s), retrying\r\n", max_std);
    return false;
  }
  gyro_x_offset = mean[0];
  gyro_y_offset = mean[1];
  gyro_z_offset = mean[2];
  SerialDebug.printf("[IMU] Gyro bias %.4f %.4f %.4f rad/s\r\n", mean[0], mean[1], mean[2]);
  return true;
}

void imu_calibrate(){
  calibration_completed = false;
}

static bool imu_read(){
  if (SIMULATION_ENABLED) {
      imu_acc.acceleration.x = gaussian_noise(0.05f);
      imu_acc.acceleration.y = gaussian_noise(0.05f);
      imu_acc.acceleration.z = 9.81f + gaussian_noise(0.05f);
      imu_gyro.gyro.x = gaussian_noise(0.005f);
      imu_gyro.gyro.y = gaussian_noise(0.005f);
      imu_gyro.gyro.z = odom_angular_velocity + gaussian_noise(0.01f);
      return true;
  }

  if (HIL_ENABLED) {
      hil_imu_sample_t s;
      if (!hil_imu_get(&s)) return false;
      imu_acc.acceleration.x = s.acc_x;
      imu_acc.acceleration.y = s.acc_y;
      imu_acc.acceleration.z = s.acc_z;
      imu_gyro.gyro.x = s.gyro_x;
      imu_gyro.gyro.y = s.gyro_y;
      imu_gyro.gyro.z = s.gyro_z;
      return true;
  }

  mpu.getEvent(&imu_acc, &imu_gyro, &imu_temp);

  return true;
}

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void imu_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 1000 * 1000 / IMU_UPDATE_FREQUENCY);

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

            bool have_sample = imu_read();
            if (have_sample && !calibration_completed) {
              calibration_completed = imu_calibrate_step();
            }
            else if (have_sample){
              double gz;
              if (IMU_INVERT){
                gz = -(imu_gyro.gyro.z - gyro_z_offset);
              }
              else {
                gz = imu_gyro.gyro.z - gyro_z_offset;
              }
              if (fabs(gz) < GYRO_DEADBAND) gz = 0.0f;

              if (fabs(imu_acc.acceleration.x - acc_x_offset) < GYRO_DEADBAND) {
                  imu_acc.acceleration.x = acc_x_offset;
              }
              if (fabs(imu_acc.acceleration.y - acc_y_offset) < GYRO_DEADBAND) {
                  imu_acc.acceleration.y = acc_y_offset;
              }

              IMU_data_t IMU_data = {imu_acc.acceleration.x - acc_x_offset, imu_acc.acceleration.y - acc_y_offset, imu_acc.acceleration.z - acc_z_offset,
                                    imu_gyro.gyro.x - gyro_x_offset, imu_gyro.gyro.y - gyro_y_offset, gz};

              struct timespec ts;
              clock_gettime(CLOCK_REALTIME, &ts);
              IMU_data.timestamp_sec = ts.tv_sec;
              IMU_data.timestamp_nsec = ts.tv_nsec;

              if (TASKS_STARTED) {
                dds_result_t result = DDS_PUBLISH("/imu", IMU_data);
                if (result != DDS_SUCCESS) {
                    SerialDebug.printf("IMU Topic publish failed: %s\r\n", DDS_RESULT_TO_STRING(result));
                }

                /*
                SerialDebug.printf("IMU: acc=(%.2f, %.2f, %.2f) gyro=(%.2f, %.2f, %.2f)\r\n",
                                   IMU_data.acc_x, IMU_data.acc_y, IMU_data.acc_z,
                                   IMU_data.gyro_x, IMU_data.gyro_y, IMU_data.gyro_z);
                                   */
                                   
              }
            }

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}