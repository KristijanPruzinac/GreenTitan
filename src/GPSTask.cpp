#include "GPSTask.h"

HardwareSerial SerialGPS(2);

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

NAV_POSLLH posllh;

long prevLon = 1;
long prevLat = 1;
long prevAcc = -2;

GPS_SensorInterface_Unit GPS_gpsData;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

void GPSRead() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  xSemaphoreTake(GPSMutex, portMAX_DELAY);
  while ( SerialGPS.available() ) {
    byte c = SerialGPS.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&posllh))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          //returnVal = true;
          break;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }

  //Calculate heading if position changed
  if (prevLon != posllh.lon || prevLat != posllh.lat){
    GPS_gpsData.heading = AngleBetweenPoints(prevLon, prevLat, posllh.lon, posllh.lat);
    GPS_gpsData.distance = Distance(prevLon, prevLat, posllh.lon, posllh.lat);
    GPS_gpsData.accuracy = posllh.hAcc;

    xQueueSend(GPS_SensorInterface_Queue, &GPS_gpsData, 0);

    prevLon = posllh.lon;
    prevLat = posllh.lat;
    prevAcc = posllh.hAcc;
  }

  xSemaphoreGive(GPSMutex);
}

long GPS_GetLon(){
  xSemaphoreTake(GPSMutex, portMAX_DELAY);
  long valueToReturn = prevLon;
  xSemaphoreGive(GPSMutex);
  return valueToReturn;
}
long GPS_GetLat(){
  xSemaphoreTake(GPSMutex, portMAX_DELAY);
  long valueToReturn = prevLat;
  xSemaphoreGive(GPSMutex);
  return valueToReturn;
}
long GPS_GetAcc(){
  xSemaphoreTake(GPSMutex, portMAX_DELAY);
  long valueToReturn = prevAcc;
  xSemaphoreGive(GPSMutex);
  return valueToReturn;
}

bool InitGPS(){
  SerialGPS.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialGPS.setTimeout(COMMUNICATION_TIMEOUT);

  //Dont touch magic numbers
  posllh.lon = 1;
  posllh.lat = 1;
  prevAcc = -1;

  //TODO: Implement check for GPS initializing properly
  return true;
}

void GPSTask(void* pvParameters){
  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / GPS_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    GPSRead();

    //TODO: Implement rain sensor

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}