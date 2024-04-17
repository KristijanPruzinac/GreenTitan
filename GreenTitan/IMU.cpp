#include "IMU.h"

float IMUCurrentAzimuth = 0;

//Globals
bool imuFirstMeasurement = true;

float IMURotSpeed = 0;
float IMURotAcc = 0;

float IMURotPrevSpeed = 0;
float IMURotPrevAcc = 0;

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
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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
  if (imuFirstMeasurement){
    imuFirstMeasurement = false;
  }
  else {
    IMURotPrevAcc = IMURotAcc;
    IMURotAcc = (newRotSpeed - IMURotSpeed) / (1.0 / SENSORS_SAMPLING_RATE); //Acceleration is derivation of speed with respect to time
  }
  IMURotPrevSpeed = IMURotSpeed;
  IMURotSpeed = newRotSpeed;
  xSemaphoreGive(IMUMutex);

  //Printout TODO: REMOVE
  /*
  Serial.print("IMURotSpeed: ");
  Serial.print(IMURotSpeed);
  Serial.print(", IMURotAcc: ");
  */
  //Serial.println(IMURotAcc);
}

float IMUGetAzimuth(){
  return IMUCurrentAzimuth;
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