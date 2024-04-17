#include "MotionTask.h"

float MowerRotAngle = 0;
float MowerRotSpeed = 0;
float MowerRotAcc = 0;

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

void MotionUpdateAzimuth(){
  //Grab IMU data and average over 2 samples
  xSemaphoreTake(IMUMutex, portMAX_DELAY);
  MowerRotSpeed = (IMURotPrevSpeed + IMURotSpeed) / 2;
  MowerRotAcc =(IMURotPrevAcc +  IMURotAcc) / 2;
  xSemaphoreGive(IMUMutex);

  //Grab GPS data
  xSemaphoreTake(GPSMutex, portMAX_DELAY);
  MowerHeading = GPS_Heading;
  xSemaphoreGive(GPSMutex);

  //Serial.println(MowerRotSpeed);
}

void MotionSetTarget(int tLon, int tLat){
  MotionPrevLon = GpsGetLon();
  MotionPrevLat = GpsGetLat();

  MotionTargetLon = tLon;
  MotionTargetLat = tLat;

  //MotionMode = ROTATING; TODO: Uncomment
  MotionSetMode(MOVING);
  //MotionSetMode(TEST);
}

/*
int pointsCount = 0;
int gcodeIndex = 0;
int pointsX[50];
int pointsY[50];

//Parse bluetooth messages
void MainParseBluetooth(){
  while (true){
    String message = BluetoothRead(); if (message.length() == 0 || message.equals("")){ break; }

    //Count / characters
    int co = 0;
    for (int i = 0; i < message.length(); i++) {
      if (message.charAt(i) == '/') {
        co++;
      }
    }
    if (co < 2){break;} //Invalid message

    String MessageGroup = message.substring(0, message.indexOf('/'));
    message = message.substring(message.indexOf('/') + 1); // Update message to remove the first part

    String ActionGroup = message.substring(0, message.indexOf('/'));
    message = message.substring(message.indexOf('/') + 1); // Update message to remove the second part

    String DataGroup = message;

    //STATUS ------------------------------------------
    if (MessageGroup == "STATUS") {
      if (ActionGroup == "GET") {
        Respond("STATUS", MowerStatus);
      }
      else if (ActionGroup == "SET") {
        if (DataGroup == "RUNNING") {
          MowerResume();
        }
        else if (DataGroup == "PAUSED") {
          MowerPause();
        }

        Respond("STATUS", MowerStatus);
      }
    }
    else if (MessageGroup == "MOTOR") {
      if (ActionGroup == "SET") {
        if (DataGroup == "LEFTFORWARD") {}
        else if (DataGroup == "LEFTBACKWARD") {}
        else if (DataGroup == "LEFTSTOP") {}
        else if (DataGroup == "RIGHTFORWARD") {}
        else if (DataGroup == "RIGHTBACKWARD") {}
        else if (DataGroup == "RIGHTSTOP") {}
        else if (DataGroup == "MAINSTART") {
          MotorMainOn();
        }
        else if (DataGroup == "MAINSTOP") {
          MotorMainOff();
        }
      }
      else if (ActionGroup == "MOVE") {
        if (DataGroup == "FORWARD") {
          MotorForward();
        }
        else if (DataGroup == "BACKWARD") {
          MotorBackward();
        }
        else if (DataGroup == "ROTATELEFT") {
          MotorTurnLeft();
        }
        else if (DataGroup == "ROTATERIGHT") {
          MotorTurnRight();
        }
        else if (DataGroup == "STOP"){
          MotorStop();
        }
      }
    }
    else if (MessageGroup == "GPS"){
      if (ActionGroup == "GET") {
        if (DataGroup == "ACCURACY") {
          Respond("GPS", String(posllh.hAcc));
        }
      }
    }
    else if (MessageGroup == "GCODE"){
      if (ActionGroup == "ADD") {
        if (DataGroup == "POINT") { //ADD NEW POINT
          pointsX[pointsCount] = posllh.lon;
          pointsY[pointsCount] = posllh.lat;

          pointsCount++;
        }
      }
      else if (ActionGroup == "RESET") {
        pointsCount = 0;
      }
      else if (ActionGroup == "START") {
        MowerExecutingPath = true;
        gcodeIndex = 0;
      }
    }
  }

  return;
}
*/

int angleTesting = 0;
float currentMotVal = 0;

void MotionTask(void* pvParameters){
  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / GPS_SAMPLING_RATE);

    xLastWakeTime = xTaskGetTickCount();

    MotionUpdateAzimuth();
/*
    if (MotionMode == ROTATING){

      float RotationAngle = ShortestRotation(MotionCurrentAzimuth, AngleBetweenPoints(GpsGetLon(), GpsGetLat(), MotionTargetLon, MotionTargetLat));
      
      if (abs(RotationAngle) <= MOTION_ACCEPTED_ROTATION_TO_POINT){ //|| abs(RotationAngle) > MotionPrevRotationAngle + MOTION_ACCEPTED_ROTATION_TO_POINT){
        MotionSetMode(MOVING);
        //MotorStop();

        //TODO: Uncomment top
        
        //MotorDriveAngle(0, FORWARD, 1.0);
        //delay(2000);
        //MotorStop();
        

        //TODO: Remove
        BluetoothWrite("Switched to moving.");
      }
      else {
        if (RotationAngle < 0){
          MotorRotate(LEFT, 1.0);
        }
        else {
          MotorRotate(RIGHT, 1.0);
        }
      }

      if (abs(RotationAngle) < MotionPrevRotationAngle)
        MotionPrevRotationAngle = abs(RotationAngle);
    }
    else */if (MotionMode == MOVING){
      float PrevTargetAngle = AngleBetweenPoints(MotionPrevLon, MotionPrevLat, MotionTargetLon, MotionTargetLat);
      float MowerTargetAngle = AngleBetweenPoints(GpsGetLon(), GpsGetLat(), MotionTargetLon, MotionTargetLat);
      float AngleDiff = ShortestRotation(PrevTargetAngle, MowerTargetAngle);
      
      float MowerTargetDist = sqrt(pow(GpsGetLon() - MotionTargetLon, 2) + pow(GpsGetLat() - MotionTargetLat, 2));
      
      float DistAint = MowerTargetDist * cos(radians(abs(AngleDiff))); //Distance left to target
      float DistOffset = MowerTargetDist * sin(radians(abs(AngleDiff))); //Distance from expected line to 


/*
      float SlopeFactor = 0.15;
      float MaxSlopeAngle = 20.0;
      */
      
      /*
      float CorrectionAngle = NormalizeAngle(PrevTargetAngle + (AngleDiff / fabs(AngleDiff)) * ( (DistOffset * SlopeFactor) / (1 + fabs(DistOffset * SlopeFactor)) ) * MaxSlopeAngle);
      if (DistOffset <= 4){
        CorrectionAngle = PrevTargetAngle;
      }
      */
      /*
      float CorrectionAngleSlope = pow(1.2, DistOffset) + 0.5 * DistOffset - 1.0;
      */
      
      /*
      float CorrectionAngleSlope = DistOffset * 1.4;
      if (isnan(CorrectionAngleSlope) || isnan(AngleDiff) || isnan(DistOffset)){
        CorrectionAngleSlope = 0;
      }
      */
      
      
      /*
      float CorrectionAngleSlope;
      if (DistOffset <= 2.5){
        CorrectionAngleSlope = 0;
      }
      else {
        CorrectionAngleSlope = 15;
      }
      */

/*
      if (CorrectionAngleSlope > MaxSlopeAngle){
        CorrectionAngleSlope = MaxSlopeAngle;
      }
      else if (CorrectionAngleSlope < 0){
        CorrectionAngleSlope = 0;
      }
      */
      /*
      float CorrectionAngle = NormalizeAngle(PrevTargetAngle + (AngleDiff / fabs(AngleDiff)) * CorrectionAngleSlope);

      float CurrentGpsCorrectionAngle = ShortestRotation(MowerHeading, CorrectionAngle);
      */

      //Update GPS azimuth
      //IMUCurrentAzimuth = NormalizeAngle(IMUCurrentAzimuth + ShortestRotation(IMUCurrentAzimuth, MowerHeading) * );
      
      //STRAYED FROM PATH
      if (DistOffset > MAX_DEVIATION){
        //MotorStop();
        //Error("Mower strayed from path!");
        
        //TODO: Remove
        //BluetoothWrite("Mower strayed from path!");
      }
      
      //float RotationAngle = ShortestRotation(MowerTargetAngle, MotionCurrentAzimuth);
      //PID 0.8 0 1 Overall ok, need stability or eliminate sse

      //Update PID with DIST FROM LINE
      
      /*
      if (AngleDiff < 0)
        PID_Input = -DistOffset;
      else
         PID_Input = DistOffset;
      */

      if (counter % 2 == 0){//TODO: Remove
        toSend += String(DistOffset * (AngleDiff / fabs(AngleDiff))) + "\n";
      }
      counter++;

      if (counter >= 5){
        counter = 0;

        BluetoothWrite(toSend);

        toSend = "";
      }//

      //Update PID with ANGLE TO TARGET from GPS
      //PID_Input = CurrentGpsCorrectionAngle;

      //MotionPID.Compute();
      
      //float DistanceToTarget = sqrt(pow(MotionTargetLon - GpsGetLon(), 2) + pow(MotionTargetLat - GpsGetLat(), 2));
      
      if (DistAint < MOTION_ACCEPTED_DIST_TO_POINT || abs(AngleDiff) > 90.0){
        MotorStop();
        MotionSetMode(WAITING);

        //TODO: Remove
        BluetoothWrite("Ended motion.");

      }
      else {
        /*
        float RotFactor = DistOffset / 5.0; if (RotFactor > 1){RotFactor = 1;}
        if (RotationAngle < 0){
          MotorDriveAngle((-90)*(1 - RotFactor * 0.7), true, 1.0);
        }
        else {
          MotorDriveAngle((90)*(1 - RotFactor * 0.7), true, 1.0);
        }
        */
        //MotorDriveAngle(-CurrentGpsCorrectionAngle * 0.7, FORWARD, 1.0); //Inverted direction

        //MotorDriveAngle(0 - (MowerRotAcc / MOTION_ACC_FACTOR) * 90.0, FORWARD, 1.0);
        
        /*
        float TargetRotSpeed = -AngleDiff;
        currentMotVal += (TargetRotSpeed - MowerRotSpeed) * MOTION_ACC_FACTOR;
        currentMotVal = constrain(currentMotVal, -90, 90);
        MotorDriveAngle(currentMotVal, FORWARD, 1.0);
        */
      }
    }
    else if (MotionMode == TEST){
      MotorDriveAngle(0, FORWARD, 1.0);
      delay(5000);
      MotorRotate(LEFT, 1.0);
      delay(800);
      MotorDriveAngle(0, FORWARD, 1.0);
      delay(5000);
    }

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