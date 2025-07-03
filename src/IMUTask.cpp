#include "IMUTask.h"

//Globals
const int IMUDataMaxSampleSize = 9;
float IMU_RotSpeedData[IMUDataMaxSampleSize];
float IMU_RotAccData[IMUDataMaxSampleSize];
int IMUDataSampleSize = 3;

float IMU_RotSpeedRaw = 0;
float IMU_RotSpeed = 0;
float IMU_RotAcc = 0;
float IMU_HeadingChange = 0;

//Calibration
float IMUGyroXOffset = 0;
float IMUGyroYOffset = 0;
float IMUGyroZOffset = 0;

//Data
sensors_event_t IMU_acc, IMU_gyro, IMU_temp;

/* Assign a unique ID to this sensor at the same time */
Adafruit_MPU6050 mpu;

bool InitIMU(){
  if (!mpu.begin()) {
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  return true;
}

void IMUCalibrate(){
  //Init temp values
  float ogx = 0;
  float ogy = 0;
  float ogz = 0;

  //Read calibration data for 2s
  int sample_size = 40;
  int sample_delay = 50;
  for (int i = 0; i < sample_size; i++){
    mpu.getEvent(&IMU_acc, &IMU_gyro, &IMU_temp);

    ogx += IMU_gyro.gyro.x;
    ogy += IMU_gyro.gyro.y;
    ogz += IMU_gyro.gyro.z;

    delay(sample_delay);
  }

  //Divide to get average value
  IMUGyroXOffset = ogx / sample_size;
  IMUGyroYOffset = ogy / sample_size;
  IMUGyroZOffset = ogz / sample_size;
}

void IMURead(){
  mpu.getEvent(&IMU_acc, &IMU_gyro, &IMU_temp);

  xSemaphoreTake(IMUMutex, portMAX_DELAY);
  //Calculate angular speed and acceleration
  IMU_RotSpeedRaw = IMUGetGyroZ();

  //Shift data for averaging
  for (int i = 0; i < IMUDataMaxSampleSize - 1; i++){
    IMU_RotSpeedData[i] = IMU_RotSpeedData[i + 1];
  }
  for (int i = 0; i < IMUDataMaxSampleSize - 1; i++){
    IMU_RotAccData[i] = IMU_RotAccData[i + 1];
  }

  //Add speed reading
  IMU_RotSpeedData[IMUDataMaxSampleSize - 1] = IMU_RotSpeedRaw;

  //Average speed readings
  IMU_RotSpeed = 0;
  for (int i = IMUDataMaxSampleSize - 1; i > IMUDataMaxSampleSize - 1 - IMUDataSampleSize; i--){
    IMU_RotSpeed += IMU_RotSpeedData[i];
  }
  IMU_RotSpeed /= (float) IMUDataSampleSize;

  //Average acceleration readings
  IMU_RotAcc = 0;
  for (int i = IMUDataMaxSampleSize - 1; i > IMUDataMaxSampleSize - 1 - IMUDataSampleSize; i--){
    IMU_RotAcc += (IMU_RotSpeedData[i] - IMU_RotSpeedData[i - 1]) / (1.0 / IMU_UPDATE_FREQUENCY);
  }
  IMU_RotAcc /= (float) IMUDataSampleSize;

  //Add current heading
  IMU_HeadingChange = NormalizeAngle(IMU_HeadingChange + IMU_RotSpeedRaw / IMU_UPDATE_FREQUENCY);

  xSemaphoreGive(IMUMutex);
}

//Accelerometer
float IMUGetAccX(){
  return IMU_acc.acceleration.x;
}

float IMUGetAccY(){
  return IMU_acc.acceleration.y;
}

float IMUGetAccZ(){
  return IMU_acc.acceleration.z;
}

//Gyro
float IMUGetGyroX(){
  return RadDeg(IMU_gyro.gyro.x - IMUGyroXOffset);
}

float IMUGetGyroY(){
  return RadDeg(IMU_gyro.gyro.y - IMUGyroYOffset);
}

float IMUGetGyroZ(){
  if (IMU_INVERT){
    return -RadDeg(IMU_gyro.gyro.z - IMUGyroZOffset);
  }
  else {
    return RadDeg(IMU_gyro.gyro.z - IMUGyroZOffset);
  }
}

//Temp
float IMUGetTemp(){
  return IMU_temp.temperature;
}

void IMUTask(void* pvParameters){
  IMUCalibrate();
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / IMU_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    IMURead();

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}