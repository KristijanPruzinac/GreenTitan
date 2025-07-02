#include "SensorInterfaceTask.h"

float AzimuthHeading = 0;

float GPS_Heading[2] = {0}; //Previous, current
float IMU_Heading[2] = {0};

float GPS_TargetHeading = 0;

GPS_SensorInterface_Unit gpsData;

// How many degrees heading changes per second due to GPS correction
float HeadingReliabillity = 360;
float HeadingCorrectionData[10] = {360};

unsigned long GPS_TimerStabillity = -1;
bool GPS_TimerStabillityActive = false;

int GPS_AccuracyStable = false;

void SensorInterface_CorrectHeading(){
  SensorInterface_IMU_UpdateHeading();
  while ((unsigned int) uxQueueMessagesWaiting(GPS_SensorInterface_Queue) > 0){
      SensorInterface_GPS_UpdateHeading();

      //Set GPS target heading if readings are valid
      if (gpsData.distance >= GPS_MIN_DIST_BETWEEN_READINGS && !(isnan(gpsData.heading) || isnan(gpsData.distance))){

          //Avoid possible division by zero error
          if (ShortestRotation(IMU_Heading[1], IMU_Heading[0]) < 0.1){
              IMU_Heading[1] = NormalizeAngle(IMU_Heading[0] + 0.1);
          }
          if (ShortestRotation(GPS_Heading[1], GPS_Heading[0]) < 0.1){
              GPS_Heading[1] = NormalizeAngle(GPS_Heading[0] + 0.1);
          }

          //Set target heading whenever valid readings arrive
          GPS_TargetHeading = GPS_Heading[1];

          //Correct azimuth with GPS target heading
          if (fabs(ShortestRotation(IMU_Heading[1], IMU_Heading[0])) <= GPS_HEADING_CORRECTION_ANGLE &&
              fabs(ShortestRotation(IMU_Heading[1], IMU_Heading[0]) - ShortestRotation(GPS_Heading[1], GPS_Heading[0])) <= GPS_HEADING_CORRECTION_ANGLE){
              //If GPS is enabled, do corrections, otherwise use IMU only mode
              if (ENABLE_GPS){
                  //TODO: Implement GPS speed measurement, and use threshold for correction
                  if (MowerIsInMotion() && fabs(GetHeading() - GPS_TargetHeading) > 0.1){
                      float HeadingAdjustment = -(ShortestRotation(GPS_TargetHeading, IMU_Heading[1]) * GPS_HEADING_CORRECTION_FACTOR - pow(ShortestRotation(GPS_TargetHeading, IMU_Heading[1]), 3) * GPS_HEADING_CORRECTION_FACTOR / 25000.0);
                      
                      //Update heading reliabillity
                      for (int i = 9; i > 0; i--){
                          HeadingCorrectionData[i] = HeadingCorrectionData[i - 1];
                      }
                      //Implement proper error measurement
                      HeadingCorrectionData[0] = fabs(HeadingAdjustment);

                      HeadingReliabillity = 0;
                      for (int i = 0; i < 10; i++){
                          HeadingReliabillity += HeadingCorrectionData[i];
                      }
                      
                      //Correct heading with GPS data
                      xSemaphoreTake(SensorInterfaceMutex, portMAX_DELAY);
                      IMU_Heading[1] = NormalizeAngle(IMU_Heading[1] + HeadingAdjustment);
                      xSemaphoreGive(SensorInterfaceMutex);
                  }
              }
          }
      }
  }
  
  xSemaphoreTake(SensorInterfaceMutex, portMAX_DELAY);
  AzimuthHeading = IMU_Heading[1];
  xSemaphoreGive(SensorInterfaceMutex);
}

void SensorInterface_GPS_UpdateHeading(){
    xQueueReceive(GPS_SensorInterface_Queue, &gpsData, 0);
    xSemaphoreTake(SensorInterfaceMutex, portMAX_DELAY);
    GPS_Heading[0] = GPS_Heading[1];
    GPS_Heading[1] = gpsData.heading;
    xSemaphoreGive(SensorInterfaceMutex);
}
void SensorInterface_IMU_UpdateHeading(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    IMU_Heading[0] = IMU_Heading[1];
    IMU_Heading[1] = NormalizeAngle(IMU_Heading[1] + IMU_HeadingChange);
    IMU_HeadingChange = 0; //Reset heading change after update
    xSemaphoreGive(IMUMutex);
}

void GPS_CheckStabillity(){
  int gpsAccuracy = GPS_GetAcc();
    if (gpsAccuracy <= GPS_ACC_THRESHOLD && gpsAccuracy > 0){ //Check to see if accuracy is within threshold, and if so try to check if it is stable
      if (!GPS_AccuracyStable){
        if (GPS_TimerStabillityActive){
          if ((millis() - GPS_TimerStabillity) / MILLIS_PER_SECOND > (unsigned long) GPS_STABILITY_CHECK_DURATION){ //If accuracy is stable for long enough, set GPS_ACCURACY_STABLE to true
            GPS_TimerStabillityActive = false;

            xSemaphoreTake(SensorInterfaceMutex, portMAX_DELAY);
            GPS_AccuracyStable = true;
            xSemaphoreGive(SensorInterfaceMutex);
          }
        }
        else {  //Start timing
          GPS_TimerStabillityActive = true;
          GPS_TimerStabillity = millis();
        }
      }
    }
    else {  //Accuracy loss, immediately determines GPS_ACCURACY_STABLE = false
        xSemaphoreTake(SensorInterfaceMutex, portMAX_DELAY);
        GPS_AccuracyStable = false;
        xSemaphoreGive(SensorInterfaceMutex);

        GPS_TimerStabillityActive = false;
    }
}

// --- INTERFACE ---

//IMU
float GetRotationSpeedRaw(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    float rotSpeed = IMU_RotSpeedRaw;
    xSemaphoreGive(IMUMutex);
    return rotSpeed;
}

float GetRotationSpeed(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    float rotSpeed = IMU_RotSpeed;
    xSemaphoreGive(IMUMutex);
    return rotSpeed;
}
float GetRotationAcceleration(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    float rotAcc = IMU_RotAcc;
    xSemaphoreGive(IMUMutex);
    return rotAcc;
}

//GPS
long GetLon(){
  return GPS_GetLon();
}

long GetLat(){
  return GPS_GetLat();
}

long GetGpsAcc(){
  return GPS_GetAcc();
}

//Interface
float GetHeading(){
    xSemaphoreTake(SensorInterfaceMutex, portMAX_DELAY);
    float returnVal = AzimuthHeading;
    xSemaphoreGive(SensorInterfaceMutex);
    return returnVal;
}
bool IsGpsStable(){
    xSemaphoreTake(SensorInterfaceMutex, portMAX_DELAY);
    bool returnVal = GPS_AccuracyStable;
    xSemaphoreGive(SensorInterfaceMutex);
    return returnVal;
}

//DEBUG
void PlotIMUData(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    Serial.print(">");
    Serial.print("IMU_RotSpeed:");
    Serial.print(GetRotationSpeed());
    Serial.print(",IMU_RotAcc:");
    Serial.print(GetRotationAcceleration());
    Serial.print(",Heading:");
    Serial.print(GetHeading());
    Serial.print("\r\n");
    xSemaphoreGive(IMUMutex);
}

void SensorInterfaceTask(void* pvParameters){
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / SENSORS_INTERFACE_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    GPS_CheckStabillity();
    SensorInterface_CorrectHeading();

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}