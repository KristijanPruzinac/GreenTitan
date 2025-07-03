//FUNCTIONAL

#ifndef GPS_TASK_H
#define GPS_TASK_H

#include "Arduino.h"
#include "Defines.h"
#include "Functions.h"

#include <math.h>
#include <HardwareSerial.h>

extern SemaphoreHandle_t GPSMutex;

extern QueueHandle_t GPS_SensorInterface_Queue;

void calcChecksum(unsigned char* CK);
void GPSRead();
void GPSCheck();
bool InitGPS();
long GPS_GetLon();
long GPS_GetLat();
long GPS_GetAcc();
void GPSTask(void* pvParameters);

#endif