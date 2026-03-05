#include "imu_task.h"

//Calibration
static float gyro_x_offset = 0;
static float gyro_y_offset = 0;
static float gyro_z_offset = 0;

static float acc_x_offset = 0;
static float acc_y_offset = 0;
static float acc_z_offset = 0;

static bool calibration_completed = false;

//Data
static sensors_event_t imu_acc, imu_gyro, imu_temp;

static Adafruit_MPU6050 mpu;

bool init_imu(){
  if (!mpu.begin()) {
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  return true;
}

void imu_calibrate(){
  //Init temp values
  float gyro_x = 0;
  float gyro_y = 0;
  float gyro_z = 0;

  float acc_x = 0;
  float acc_y = 0;
  float acc_z = 0;

  //Read calibration data for 2s
  int sample_size = 40;
  int sample_delay = 50;
  for (int i = 0; i < sample_size; i++){
    mpu.getEvent(&imu_acc, &imu_gyro, &imu_temp);

    gyro_x += imu_gyro.gyro.x;
    gyro_y += imu_gyro.gyro.y;
    gyro_z += imu_gyro.gyro.z;

    acc_x += imu_acc.acceleration.x;
    acc_y += imu_acc.acceleration.y;
    acc_z += imu_acc.acceleration.z;

    delay(sample_delay);
  }

  //Divide to get average value
  gyro_x_offset = gyro_x / sample_size;
  gyro_y_offset = gyro_y / sample_size;
  gyro_z_offset = gyro_z / sample_size;

  acc_x_offset = acc_x / sample_size;
  acc_y_offset = acc_y / sample_size;
  acc_z_offset = acc_z / sample_size;

  //TODO: Check Mower is not moving during this time
  calibration_completed = true;
}

static bool imu_read(){
  if (!calibration_completed){
    return false;
  }

  mpu.getEvent(&imu_acc, &imu_gyro, &imu_temp);

  return true;
}

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void imu_task(void* parameter) {

    if (!ENABLE_IMU) {
        vTaskDelete(NULL);
        return;
    }

    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 100 * 1000); // 100 ms

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

            if (!imu_read()){
              continue;
            }

            float gz;
            if (IMU_INVERT){
              gz = -(imu_gyro.gyro.z - gyro_z_offset);
            }
            else {
              gz = imu_gyro.gyro.z - gyro_z_offset;
            }

            IMU_data_t IMU_data = {imu_acc.acceleration.x - acc_x_offset, imu_acc.acceleration.y - acc_y_offset, imu_acc.acceleration.z - acc_z_offset,
                                   imu_gyro.gyro.x - gyro_x_offset, imu_gyro.gyro.y - gyro_y_offset, gz};

            DDS_PUBLISH("/imu", IMU_data);

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}