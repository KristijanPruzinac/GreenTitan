#include "IMU.h"

float IMUCurrentAzimuth;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

bool InitIMU(){
  if (!mag.begin()) {
    return false;
  }

  IMUCurrentAzimuth = 0;

  return true;
}

void IMURead(){
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  heading += DegRad(MagDeclinationAngle);
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = NormalizeAngle(RadDeg(heading)); 

  //Invert if compass is upside down
  if (InvertCompassAzimuth){
    headingDegrees = 360.0 - headingDegrees;
  }

  //Update azimuth
  xSemaphoreTake(AzimuthMutex, portMAX_DELAY);
  IMUCurrentAzimuth = NormalizeAngle(headingDegrees - MagOffsetAngle);
  xSemaphoreGive(AzimuthMutex);
}

float IMUGetAzimuth(){
  return IMUCurrentAzimuth;
}

/*
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
*/