//FUNCTIONAL

#ifndef GPS_H
#define GPS_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"

#include <math.h>
#include <HardwareSerial.h>

extern int GPS_ACC_THRESHOLD;
extern int GPS_STABILITY_CHECK_DURATION;

extern int GPS_ACCURACY_STABLE;

extern float IMUCurrentAzimuth;

extern SemaphoreHandle_t AzimuthMutex;

void calcChecksum(unsigned char* CK);
bool GPSRead();
void GPSCheck();
void InitGPS();
int GpsGetLon();
int GpsGetLat();
int GpsGetAcc();

#endif