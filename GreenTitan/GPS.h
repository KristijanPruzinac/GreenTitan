//FUNCTIONAL

#ifndef GPS_H
#define GPS_H

#include "Arduino.h"
#include "Defines.h"

#include <HardwareSerial.h>

void calcChecksum(unsigned char* CK);
bool GPSRead();
void InitGPS();

#endif