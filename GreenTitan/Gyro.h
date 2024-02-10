//TODO: Add yaw

#ifndef GYRO_H
#define GYRO_H

#include "Arduino.h"
#include "Defines.h"

#include "Wire.h"

char* convert_int16_to_str(int16_t i);
void getAngle(int Ax,int Ay,int Az);
void InitGyro();
void GyroRead();

#endif