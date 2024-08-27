#include "MotionTask.h"

float MowerRotAngle = 0;
float MowerRotSpeed = 0;
float MowerRotAcc = 0;
float MowerIMUHeading = 0;

float MowerGPSHeading = 0;

float MowerHeading = 0;

int MotionPrevLon;
int MotionPrevLat;

int MotionTargetLon;
int MotionTargetLat;

int MotionMode = WAITING;

void MotionSetMode(int mode){
  if (mode == WAITING){
    MotorStop();
  }

  MotionMode = mode;
}

void MotionUpdateSensorData(){
  //Grab IMU data
  xSemaphoreTake(IMUMutex, portMAX_DELAY);
  MowerRotSpeed = IMURotSpeed;
  MowerRotAcc = IMURotAcc;

  //Grab GPS data
  xSemaphoreTake(GPSMutex, portMAX_DELAY);

  //In MOVING mode, If GPS heading changed adjust IMU heading
  if (MotionMode == MOVING && fabs(MowerGPSHeading - GPS_Heading) > 0.1){
    IMUHeading = NormalizeAngle(IMUHeading - ShortestRotation(GPS_Heading, IMUHeading) * GPS_HEADING_CORRECTION_FACTOR - pow(ShortestRotation(GPS_Heading, IMUHeading), 3) * GPS_HEADING_CORRECTION_FACTOR / 25000.0);
  }

  //MowerGPSHeading = GPS_Heading;
  MowerHeading = IMUHeading;

  xSemaphoreGive(GPSMutex);
  xSemaphoreGive(IMUMutex);

  //Serial.print("0 360 ");
  //Serial.print(MowerHeading); Serial.print(" "); Serial.println(MowerGPSHeading);
}

void MotionSetTarget(int tLon, int tLat){
  MotionPrevLon = GpsGetLon();
  MotionPrevLat = GpsGetLat();

  MotionTargetLon = tLon;
  MotionTargetLat = tLat;

  MotionMode = ROTATING;
  //MotionSetMode(MOVING);
  //MotionSetMode(TEST);
}

void MotionTask(void* pvParameters){
  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / MOTION_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    //Sensor data
    MotionUpdateSensorData();

    //Calculate needed values for navigating
    float PrevTargetAngle = AngleBetweenPoints(MotionPrevLon, MotionPrevLat, MotionTargetLon, MotionTargetLat); //Angle from line start point to end point
    float MowerTargetAngle = AngleBetweenPoints(GpsGetLon(), GpsGetLat(), MotionTargetLon, MotionTargetLat); //Angle from current position to end point

    float AngleDiff = ShortestRotation(PrevTargetAngle, MowerTargetAngle);
    float MowerTargetDist = sqrt(pow(GpsGetLon() - MotionTargetLon, 2) + pow(GpsGetLat() - MotionTargetLat, 2));
    
    float DistAint = MowerTargetDist * cos(radians(abs(AngleDiff))); //Distance on line remaining to target
    float DistOffset = MowerTargetDist * sin(radians(abs(AngleDiff))); //Distance from current position to line 

    //Check for nan values
    if (isnan(DistAint)) DistAint = 0;
    if (isnan(DistOffset)) DistOffset = 0;

    if (MotionMode == ROTATING){
      if (fabs(ShortestRotation(MowerHeading, MowerTargetAngle)) <= MOTION_ACCEPTED_ROTATION_TO_POINT){
        MotionSetMode(MOVING);

        //TODO: Remove
        BluetoothWrite("Ended rotating.");
      }
      else {
        MotorRotate(ShortestRotation(MowerTargetAngle, MowerHeading) / 180);
      }
    }
    else if (MotionMode == MOVING){
      /*
      //STRAYED FROM PATH
      if (DistOffset > MAX_DEVIATION){
        //MotorStop();
        //Error("Mower strayed from path!");
        
        //TODO: Remove
        //BluetoothWrite("Mower strayed from path!");
      }
*/
      if (counter % 2 == 0){//TODO: Remove
        //toSend += String((ShortestRotation(MowerTargetAngle, PrevTargetAngle) / fabs(ShortestRotation(MowerTargetAngle, PrevTargetAngle))) * DistOffset) + "\n";
        //toSend += String(MowerHeading) + " " + String(PrevTargetAngle) + " " + String(CurrentTargetAngle) + " " + "\n";
      }
      counter++;

      if (counter >= 5){
        counter = 0;

        BluetoothWrite(toSend);

        toSend = "";
      }
      
      
      if (DistAint < MOTION_ACCEPTED_DIST_TO_POINT){
        MotorStop();
        MotionSetMode(WAITING);

        //TODO: Remove
        BluetoothWrite("Ended moving.");

      }
      else {
        //float AngleToAdd = (ShortestRotation(MowerTargetAngle, PrevTargetAngle) / fabs(ShortestRotation(MowerTargetAngle, PrevTargetAngle))) * DistOffset*DistOffset*DistOffset/80.0;
        float AngleToAdd = (ShortestRotation(MowerTargetAngle, PrevTargetAngle) / fabs(ShortestRotation(MowerTargetAngle, PrevTargetAngle))) * DistOffset / 3.0;
        AngleToAdd = constrain(AngleToAdd, -30, 30);
        float CurrentTargetAngle = NormalizeAngle(PrevTargetAngle - AngleToAdd);

        MotorDriveAngle(ShortestRotation(CurrentTargetAngle, MowerHeading) * 3, FORWARD, 1.0);
        //MotorDriveAngle(ShortestRotation(0, MowerHeading) * 3, FORWARD, 1.0);
      }
    }
    /*
    else if (MotionMode == TEST){
      //TODO: Remove
      if (startMillis == 0){
        startMillis = millis();
      }

      if ((millis() - startMillis) / 1000 == 0){
        AccelerationController->SetTarget(MOTION_CORRECTION_ACCELERATION);
      }
      else if ((millis() - startMillis) / 1000 == 2){
        AccelerationController->SetTarget(0);
      }
      else if ((millis() - startMillis) / 1000 == 4){
        AccelerationController->SetTarget(-MOTION_CORRECTION_ACCELERATION);
      }
      else if ((millis() - startMillis) / 1000 == 6){
        AccelerationController->SetTarget(0);
      }
      else if ((millis() - startMillis) / 1000 == 8){
        MotionSetMode(WAITING);
        startMillis = 0;
      }

      AccelerationController->Update(MowerRotAcc, 1.0 / MOTION_UPDATE_FREQUENCY);

      float ControllerDriveAngle = AccelerationController->GetVariable();
      if (ControllerDriveAngle <= 0)
        MotorRotate(LEFT, fabs(constrain(ControllerDriveAngle, -1.0, 1.0)), 1);
      else
        MotorRotate(RIGHT, fabs(constrain(ControllerDriveAngle, -1.0, 1.0)), 1);
      //MotorDriveAngle(constrain(ControllerDriveAngle, MOTOR_ANGLE_MIN, MOTOR_ANGLE_MAX), FORWARD, 1.0);
      //MotorDriveAngle(0, FORWARD, 1.0);

      if (counter % 2 == 0){//TODO: Remove
        //toSend += String(DistOffset * (AngleDiff / fabs(AngleDiff))) + "\n";
        toSend += String(MowerRotAcc) + "\n";
      }
      counter++;

      if (counter >= 10){
        counter = 0;

        BluetoothWrite(toSend);

        toSend = "";
      }
    }
    */

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


//TODO: Implement functionality
//String Mode = "POWER_ON";
/* CHARGING STOP PAUSE START RUNNING POWER_ON SETUP*/

/*
void MainCharging(){}
void MainChargingStart(){
  Serial.println("Charging start");
}
void MainChargingStop(){}
void MainStop(){Serial.println("Strayed from path (STOP)");}
void MainPause(){}
void MainStart(){}
void MainRunning(){}
void MainPowerOn(){
  //LoadConfiguration();
  //GenerateGcode();
}
void MainSetup(){}
*/
/*
char QueueBluetoothMainReceive(){
  char returnChar = NULL;
  xQueueReceive(BluetoothMainQueue, &returnChar, 50);

  return returnChar;
}

void QueueMainBluetoothSend(char receivedChar){
  xQueueSend(BluetoothMainQueue, &receivedChar, 50);
}
*/