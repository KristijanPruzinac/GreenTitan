#include "GPS.h"

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

float GPS_Heading = 0;
float GPS_Dist = 0;

int prevLon = 0;
int prevLat = 0;
int prevAcc = 0;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool GPSRead() {
  bool returnVal = false;

  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

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
          returnVal = true;
          break;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }

  //Calculate heading if changed
  if (prevLon != posllh.lon || prevLat != posllh.lat){
    GPS_Heading = AngleBetweenPoints(prevLon, prevLat, posllh.lon, posllh.lat);
    GPS_Dist = Distance(prevLon, prevLat, posllh.lon, posllh.lat);

    //Correct azimuth
    IMUCurrentAzimuth -= ShortestRotation(GPS_Heading, IMUCurrentAzimuth) * GPS_AZIMUTH_CORRECTION_FACTOR * ( abs(GPS_Dist) / (1 + abs(GPS_Dist)) ); //Sensor fuse gps heading to correct IMU azimuth with sigmoid function
    IMUCurrentAzimuth = NormalizeAngle(IMUCurrentAzimuth);

    prevLon = (int) posllh.lon;
    prevLat = (int) posllh.lat;
  }

  prevAcc = (int) posllh.hAcc;

  return returnVal;
}

unsigned long TimerGPS = -1;
bool TimerGPSActive = false;

void GPSCheck(){
  int gpsAccuracy = GpsGetAcc();
    if (gpsAccuracy <= GPS_ACC_THRESHOLD && gpsAccuracy >= 0){ //Check to see if accuracy is within threshold, and if so try to check if it is stable
      if (!GPS_ACCURACY_STABLE){
        if (TimerGPSActive){
          if ((millis() - TimerGPS) / MILLIS_PER_SECOND > (unsigned long) GPS_STABILITY_CHECK_DURATION){ //If accuracy is stable for long enough, set GPS_ACCURACY_STABLE to true
            TimerGPSActive = false;
            GPS_ACCURACY_STABLE = true;
          }
        }
        else {  //Start timing
          TimerGPSActive = true;
          TimerGPS = millis();
        }
      }
    }
    else {  //Accuracy loss, immediately determines GPS_ACCURACY_STABLE = false
      GPS_ACCURACY_STABLE = false;
      TimerGPSActive = false;
    }
}

int GpsGetLon(){
  return prevLon;
}

int GpsGetLat(){
  return prevLat;
}

int GpsGetAcc(){
  return prevAcc;
}

void InitGPS(){
  SerialGPS.begin(19200);
  SerialGPS.setTimeout(50);

  //Dont remove
  posllh.lon = 1;
  posllh.lat = 1;
}