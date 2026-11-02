#include "imu_task.h"
#include "../hil/hil.h"
#include "../motor/motor_task.h"

//Calibration
static double gyro_x_offset = 0;
static double gyro_y_offset = 0;
static double gyro_z_offset = 0;

static volatile bool calibration_completed = false;

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
#define IMU_CALIBRATION_SAMPLES  (3 * IMU_SAMPLE_FREQUENCY)   // 3 s
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

bool imu_ready(){ return calibration_completed; }

// Latest integrated yaw for imu_yaw_now(), written after every read (other tasks read it)
static portMUX_TYPE yaw_mux = portMUX_INITIALIZER_UNLOCKED;
static double yaw_latest = 0.0, yaw_latest_t = 0.0;   // rad, esp_timer s
static float gz_latest = 0.0f;                        // rad/s, bias removed

bool imu_yaw_now(double* yaw){
  if (!calibration_completed) return false;
  portENTER_CRITICAL(&yaw_mux);
  double y = yaw_latest, t = yaw_latest_t;
  float gz = gz_latest;
  portEXIT_CRITICAL(&yaw_mux);
  if (t == 0.0) return false;
  // The read itself is IMU_SAMPLE_DELAY old when taken; the rate is held up to 0.1 s past it
  double ahead = fmin(esp_timer_get_time() * 1e-6 - t + IMU_SAMPLE_DELAY, 0.1);
  *yaw = y + gz * ahead;
  return true;
}

// GPS ground speed for the standstill test, from the bridge's /gps callback
static volatile float gps_speed = 1e6f;
static volatile uint32_t gps_speed_ms = 0;
void imu_set_ground_speed(float speed){
  gps_speed = speed;
  gps_speed_ms = millis();
}

// Standstill gyro bias (2026-11-02). The boot calibration alone left the parked heading drifting
// with the bias: 45 deg in 16 min parked in HIL (user drive 2026-11-02; there mostly the sim bias
// scaled by the varying sim speed, on the robot the MPU6050's drift), and nothing corrects the
// heading while the mower stands. While the
// mower is clearly still the gyro z mean is the bias: wheels stopped for IMU_STILL_SETTLE (a
// command hint only, never odometry), GPS ground speed below IMU_STILL_GPS_SPEED, and per block
// of IMU_STILL_BLOCK reads a low spread and a mean within IMU_STILL_RATE of the current bias. A
// still block's yaw is taken back (the yaw holds), and every IMU_BIAS_WINDOW of still blocks
// replaces the bias with their mean (HIL replay: a slower blend, 10-60 s, left more drift on the
// drive after each stop, because the sim bias moves with the sim speed). A real turn faster than
// IMU_STILL_RATE fails the block and is integrated as always; one slower than that while the wheels stand is
// held out and, after IMU_BIAS_WINDOW, learnt as bias: 0.17 deg/s, below what a push would do.
// After IMU_STILL_PARKED with the wheels stopped up to IMU_STILL_PARKED_RATE counts as bias.
static double still_wheels_t = 0.0;                        // last time a wheel turned
static double blk_t0 = 0.0, blk_yaw0 = 0.0, blk_sum = 0.0, blk_sum_sq = 0.0;
static int blk_n = 0;
static double win_sum = 0.0, win_t = 0.0;
static int win_n = 0;

// Returns the yaw to keep: blk_yaw0 when a still block ends, else yaw unchanged
static double imu_still_step(double now, double prev_t, double raw_z, double yaw_before, double yaw){
  float wl, wr;
  motor_get_step_rates(&wl, &wr);
  if (wl != 0.0f || wr != 0.0f || still_wheels_t == 0.0) still_wheels_t = now;
  bool gate = now - still_wheels_t >= IMU_STILL_SETTLE &&
              millis() - gps_speed_ms < 500 && gps_speed < IMU_STILL_GPS_SPEED;
  if (!gate) {
    blk_n = 0;
    win_n = 0; win_sum = win_t = 0.0;
    return yaw;
  }
  if (blk_n == 0) { blk_t0 = prev_t; blk_yaw0 = yaw_before; blk_sum = blk_sum_sq = 0.0; }
  blk_sum += raw_z;
  blk_sum_sq += raw_z * raw_z;
  blk_n++;
  if (now - blk_t0 < IMU_STILL_BLOCK) return yaw;

  double mean = blk_sum / blk_n;
  double sd = sqrt(fmax(blk_sum_sq / blk_n - mean * mean, 0.0));
  // Parked (wheels stopped IMU_STILL_PARKED) a larger offset is a bias step, not a turn: HIL
  // 2026-11-02, a boot calibration 0.26 deg/s off was not relearnt for 3 min under IMU_STILL_RATE
  double max_rate = now - still_wheels_t >= IMU_STILL_PARKED ? IMU_STILL_PARKED_RATE : IMU_STILL_RATE;
  bool still = fabs(mean - gyro_z_offset) < max_rate && sd < IMU_CALIBRATION_MAX_STD;
  double keep = yaw;
  if (still) {
    keep = blk_yaw0;   // the yaw holds over the block
    win_sum += blk_sum; win_n += blk_n; win_t += now - blk_t0;
    if (win_t >= IMU_BIAS_WINDOW) {
      static double printed_bias = 0.0;
      static uint32_t printed_ms = 0;
      gyro_z_offset = win_sum / win_n;
      win_n = 0; win_sum = win_t = 0.0;
      if (millis() - printed_ms > 60000 || fabs(gyro_z_offset - printed_bias) > 2e-4) {
        SerialDebug.printf("[IMU] Still: gyro z bias %.5f rad/s (%+.2f deg/min since last report)\r\n",
                           gyro_z_offset, (gyro_z_offset - printed_bias) * 60.0 * 180.0 / M_PI);
        printed_bias = gyro_z_offset;
        printed_ms = millis();
      }
    }
  } else {
    win_n = 0; win_sum = win_t = 0.0;
  }
  blk_n = 0;
  return keep;
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
    esp_timer_start_periodic(thread_context.timer, 1000 * 1000 / IMU_SAMPLE_FREQUENCY);

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
              // Sensor -> body axes (x forward, y left, z up). Mounted upside down = turned 180 deg
              // about x (IMU_INVERT): y and z flip, for the gyro and the accelerometer alike.
              double flip = IMU_INVERT ? -1.0 : 1.0;
              double gx = imu_gyro.gyro.x - gyro_x_offset;
              double gy = flip * (imu_gyro.gyro.y - gyro_y_offset);
              double gz = flip * (imu_gyro.gyro.z - gyro_z_offset);
              double ax = imu_acc.acceleration.x, ay = flip * imu_acc.acceleration.y, az = flip * imu_acc.acceleration.z;
              // No deadband on gz: the boot calibration removes the bias, and a deadband dropped the
              // slow steering corrections on a row from the heading (2026-10-29, ARCHITECTURE item 26)

              // Every read is integrated (trapezoid) into the gyro yaw; /imu goes out at
              // IMU_UPDATE_FREQUENCY with the averages since the last message, so a turn that
              // starts or stops between messages is still counted in full.
              static double yaw = 0.0, last_t = 0.0, last_gz = 0.0, pub_t = 0.0, pub_yaw = 0.0;
              static double roll = 0.0, pitch = 0.0;
              static double sum[5];
              static int n = 0;
              double now = esp_timer_get_time() * 1e-6;
              // Roll and pitch: the gyro carries them (Euler rates, so a turn on a slope stays
              // right), the accelerometer's gravity direction pulls them in over IMU_TILT_TAU. The
              // pull is slow because driving and turning add their own acceleration; the part a
              // spin puts on the IMU, IMU_X_BEHIND_AXLE ahead of the centre, is taken out first.
              double ax_g = ax - gz * gz * IMU_X_BEHIND_AXLE;
              double roll_acc = atan2(ay, az), pitch_acc = atan2(-ax_g, sqrt(ay * ay + az * az));
              if (last_t > 0.0) {
                double dt = now - last_t;
                double yaw_before = yaw;
                yaw += 0.5 * (gz + last_gz) * dt;
                yaw = imu_still_step(now, last_t, imu_gyro.gyro.z, yaw_before, yaw);
                double cr = cos(roll), sr = sin(roll), tp = tan(pitch);
                roll += (gx + (sr * gy + cr * gz) * tp) * dt;
                pitch += (cr * gy - sr * gz) * dt;
                double k = fmin(1.0, dt / IMU_TILT_TAU);
                roll += k * remainder(roll_acc - roll, 2.0 * M_PI);
                pitch += k * (pitch_acc - pitch);
              } else {
                roll = roll_acc;
                pitch = pitch_acc;
              }
              last_t = now;
              last_gz = gz;
              portENTER_CRITICAL(&yaw_mux);
              yaw_latest = yaw;
              yaw_latest_t = now;
              gz_latest = (float)gz;
              portEXIT_CRITICAL(&yaw_mux);
              double s_now[5] = { ax, ay, az, gx, gy };
              for (int i = 0; i < 5; i++) sum[i] += s_now[i];
              n++;
              if (now - pub_t < (1.0 - 0.5 * IMU_UPDATE_FREQUENCY / IMU_SAMPLE_FREQUENCY) / IMU_UPDATE_FREQUENCY) {
                DDS_PROCESS_THREAD_MESSAGES(&thread_context);
                DDS_GIVE_MUTEX(&thread_context);
                continue;
              }
              double gz_avg = pub_t > 0.0 ? (yaw - pub_yaw) / (now - pub_t) : gz;
              IMU_data_t IMU_data = {(float)(sum[0] / n), (float)(sum[1] / n), (float)(sum[2] / n),
                                    (float)(sum[3] / n), (float)(sum[4] / n), (float)gz_avg, yaw,
                                    (float)roll, (float)pitch};
              for (int i = 0; i < 5; i++) sum[i] = 0.0;
              n = 0;
              pub_t = now;
              pub_yaw = yaw;

              // Stamped when the gyro measured it, not when it is sent: the angle is the one at the
              // last read, IMU_SAMPLE_DELAY old by then (sensor filter, HIL link). Stamped at publish
              // time (until 2026-10-30) the gyro ran ~0.1 s behind the GPS in HIL, and at every curve exit
              // the EKF heading went ~5 deg past the turn (ARCHITECTURE item 26).
              struct timespec ts;
              clock_gettime(CLOCK_REALTIME, &ts);
              int64_t stamp_ns = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec
                                 - (int64_t)((esp_timer_get_time() * 1e-6 - now + IMU_SAMPLE_DELAY) * 1e9);
              IMU_data.timestamp_sec = stamp_ns / 1000000000LL;
              IMU_data.timestamp_nsec = stamp_ns % 1000000000LL;

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