#include "IMU.h"

//Globals
const int IMUDataMaxSampleSize = 9;
float IMURotSpeedData[IMUDataMaxSampleSize];
float IMURotAccData[IMUDataMaxSampleSize];
int IMUDataSampleSize = 3;

float IMURotSpeed = 0;
float IMURotAcc = 0;
float IMUHeading = 0;

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
  float newRotSpeed = IMUGetGyroZ();

  //Shift data for averaging
  for (int i = 0; i < IMUDataMaxSampleSize - 1; i++){
    IMURotSpeedData[i] = IMURotSpeedData[i + 1];
  }
  for (int i = 0; i < IMUDataMaxSampleSize - 1; i++){
    IMURotAccData[i] = IMURotAccData[i + 1];
  }

  //Add speed reading
  IMURotSpeedData[IMUDataMaxSampleSize - 1] = newRotSpeed;

  //Average speed readings
  IMURotSpeed = 0;
  for (int i = IMUDataMaxSampleSize - 1; i > IMUDataMaxSampleSize - 1 - IMUDataSampleSize; i--){
    IMURotSpeed += IMURotSpeedData[i];
  }
  IMURotSpeed /= (float) IMUDataSampleSize;

  //Average acceleration readings
  IMURotAcc = 0;
  for (int i = IMUDataMaxSampleSize - 1; i > IMUDataMaxSampleSize - 1 - IMUDataSampleSize; i--){
    IMURotAcc += (IMURotSpeedData[i] - IMURotSpeedData[i - 1]) / (1.0 / IMU_UPDATE_FREQUENCY);
  }
  IMURotAcc /= (float) IMUDataSampleSize;

  //Add current heading
  IMUHeading = NormalizeAngle(IMUHeading + newRotSpeed / IMU_UPDATE_FREQUENCY);

  xSemaphoreGive(IMUMutex);

  //Printout TODO: REMOVE
  /*
  Serial.print("IMURotSpeed: ");
  Serial.print(IMURotSpeed);
  Serial.print(", IMURotAcc: ");
  */
  //Serial.println(IMURotAcc);
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
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / IMU_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    IMURead();

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}