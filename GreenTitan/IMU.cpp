#include "IMU.h"

sensors_event_t IMU_acc, IMU_gyro, IMU_temp;

Adafruit_MPU6050 mpu;

float IMUCurrentAzimuth;

bool InitIMU(){
  if (!mpu.begin()) {
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  IMUCurrentAzimuth = 0;

  return true;
}

void IMURead(){
 //a.acceleration.x y z (m/s2),  g.gyro.x y z (rad/s),  temp.temperature (degC)
  mpu.getEvent(&IMU_acc, &IMU_gyro, &IMU_temp);

  //Update azimuth
  IMUCurrentAzimuth -= IMUGetGyroZ() * (1.0 / SENSORS_SAMPLING_RATE);
  IMUCurrentAzimuth = NormalizeAngle(IMUCurrentAzimuth);
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
  return RadDeg(IMU_gyro.gyro.x);
}

float IMUGetGyroY(){
  return RadDeg(IMU_gyro.gyro.y);
}

float IMUGetGyroZ(){
  return RadDeg(IMU_gyro.gyro.z);
}

//Temp
float IMUGetTemp(){
  return IMU_temp.temperature;
}