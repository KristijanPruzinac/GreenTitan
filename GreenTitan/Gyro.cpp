#include "Gyro.h"

// (c) Michael Schoeffler 2017, http://www.mschoeffler.de
const int MPU_ADDR = 0x68;
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature_;
int16_t temperature;

double pitch, roll;

int16_t xErr, yErr, zErr;
bool errFirst = true;

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void getAngle(int Ax,int Ay,int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}

void InitGyro(){
  Wire.begin(26, 25, 100000); // sda, scl, clock speed
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void GyroRead(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, 1); // request a total of 7*2=14 registers //Change 1 to true if error
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  temperature = temperature_/340.00+36.53;

  if (errFirst){
    xErr = gyro_x;
    yErr = gyro_y;
    zErr = gyro_z;

    errFirst = false;
  }

  getAngle(gyro_x,gyro_y,gyro_z);

  //printing values to serial port
  Serial.print("Angle: ");
  Serial.print("Pitch = "); Serial.print(pitch);
  Serial.print(" Roll = "); Serial.println(roll);
}