#include "MotionTask.h"

float MotionCurrentAzimuth = 0;
float MotionTargetAzimuth;

int MotionPrevLon;
int MotionPrevLat;

int MotionTargetLon;
int MotionTargetLat;

//PID variables
double PID_Setpoint = 0, PID_Input = 0, PID_Output = 0;
PID MotionPID(&PID_Input, &PID_Output, &PID_Setpoint, PID_Kp, PID_Ki, PID_Kd, DIRECT, REVERSE);

float MotionPrevRotationAngle = 999;

int MotionMode = WAITING;

void InitializeMotionPID() {
  MotionPID.SetTunings(PID_Kp, PID_Ki, PID_Kd);
  MotionPID.SetSampleTime(MILLIS_PER_SECOND / GPS_SAMPLING_RATE); //TODO: Change if using compass for PID
  MotionPID.SetOutputLimits(-90, 90);
}

void MotionUpdatePIDParameters(double Kp, double Ki, double Kd){
  xSemaphoreTake(PID_Mutex, portMAX_DELAY);
  PID_Kp = Kp;
  PID_Ki = Ki;
  PID_Kd = Kd;
  MotionPID.SetTunings(PID_Kp, PID_Ki, PID_Kd);
  xSemaphoreGive(PID_Mutex);
}

void MotionSetMode(int mode){
  //TODO: Maybe add for rotating as well
  if (mode == MOVING)
    MotionPID.SetMode(AUTOMATIC);
  if (mode == WAITING)
    MotionPID.SetMode(MANUAL);

  MotionMode = mode;
}

void MotionUpdateAzimuth(){
  xSemaphoreTake(AzimuthMutex, portMAX_DELAY);
  MotionCurrentAzimuth = IMUCurrentAzimuth;
  xSemaphoreGive(AzimuthMutex);
}

void MotionSetTarget(int tLon, int tLat){
  MotionPrevLon = GpsGetLon();
  MotionPrevLat = GpsGetLat();

  MotionTargetLon = tLon;
  MotionTargetLat = tLat;

  //TODO: REMOVE
  delay(5000);

  MotionPrevRotationAngle = 999;
  //MotionMode = ROTATING; TODO: Uncomment
  MotionSetMode(MOVING);
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
void MotionTask(void* pvParameters){
  InitializeMotionPID();

  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / SENSORS_SAMPLING_RATE);

    xLastWakeTime = xTaskGetTickCount();

    MotionUpdateAzimuth();

    if (MotionMode == ROTATING){

      float RotationAngle = ShortestRotation(MotionCurrentAzimuth, AngleBetweenPoints(GpsGetLon(), GpsGetLat(), MotionTargetLon, MotionTargetLat));

      if (counter % 2 == 0){//TODO: Remove
        toSend += String(RotationAngle) + " " + String(abs(RotationAngle) / 180.0) + "\n";
      }
      counter++;

      if (counter >= 10){
        counter = 0;

        BluetoothWrite(toSend);

        toSend = "";
      }//
      
      if (abs(RotationAngle) <= MOTION_ACCEPTED_ROTATION_TO_POINT){ //|| abs(RotationAngle) > MotionPrevRotationAngle + MOTION_ACCEPTED_ROTATION_TO_POINT){
        MotionSetMode(MOVING);
        //MotorStop();

        //TODO: Uncomment top
        /*
        MotorDriveAngle(0, FORWARD, 1.0);
        delay(2000);
        MotorStop();
        */

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
    else if (MotionMode == MOVING){
      float PrevTargetAngle = AngleBetweenPoints(MotionPrevLon, MotionPrevLat, MotionTargetLon, MotionTargetLat);
      float MowerTargetAngle = AngleBetweenPoints(GpsGetLon(), GpsGetLat(), MotionTargetLon, MotionTargetLat);
      float AngleDiff = ShortestRotation(PrevTargetAngle, MowerTargetAngle);
      
      float MowerTargetDist = sqrt(pow(GpsGetLon() - MotionTargetLon, 2) + pow(GpsGetLat() - MotionTargetLat, 2));
      
      float DistAint = MowerTargetDist * cos(radians(abs(AngleDiff))); //Distance left to target
      float DistOffset = MowerTargetDist * sin(radians(abs(AngleDiff))); //Distance from expected line to target
      
      //STRAYED FROM PATH
      if (DistOffset > MAX_DEVIATION){
        MotorStop();
        //Error("Mower strayed from path!");
        
        //TODO: Remove
        BluetoothWrite("Mower strayed from path!");
      }
      
      float RotationAngle = ShortestRotation(MowerTargetAngle, MotionCurrentAzimuth);

      //Update PID with DIST FROM LINE
      
      if (AngleDiff < 0)
        PID_Input = -DistOffset;
      else
         PID_Input = DistOffset;
      

      //Update PID with ANGLE TO TARGET
      /*
      PID_Input = RotationAngle;
      */

      MotionPID.Compute();
      
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
        MotorDriveAngle(-1 * PID_Output, FORWARD, 1.0); //Inverted direction
      }
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