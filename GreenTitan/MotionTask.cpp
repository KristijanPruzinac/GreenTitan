#include "MotionTask.h"

float MotionCurrentAzimuth = 0;
float MotionTargetAzimuth;

int MotionPrevLon;
int MotionPrevLat;

int MotionTargetLon;
int MotionTargetLat;

void MotionUpdateMovingAzimuth(){
  //MotionCurrentAzimuth += ;
}

void MotionMoveToTarget(){
  float PrevTargetAngle = angleBetweenPoints(MotionPrevLon, MotionPrevLat, MotionTargetLon, MotionTargetLat);
  float MowerTargetAngle = angleBetweenPoints(GpsGetLon(), GpsGetLat(), MotionTargetLon, MotionTargetLat);
  float AngleDiff = ShortestRotation(PrevTargetAngle, MowerTargetAngle);
  
  float MowerTargetDist = sqrt(pow(GpsGetLon() - MotionTargetLon, 2) + pow(GpsGetLat() - MotionTargetLat, 2));
  
  float DistAint = MowerTargetDist * cos(radians(abs(AngleDiff)));
  float DistOffset = MowerTargetDist * sin(radians(abs(AngleDiff)));
  
  //STRAYED FROM PATH
  if (DistOffset > MAX_DEVIATION){MainStop();}
  
  float RotationAngle = ShortestRotation(MotionCurrentAzimuth, MowerTargetAngle);
  
  //float DistanceToTarget = sqrt(pow(MotionTargetLon - GpsGetLon(), 2) + pow(MotionTargetLat - GpsGetLat(), 2));
  
  /* TODO: Implement proper motion
  if (DistAint < 3 || AngleDiff > 90.0){
    std::vector<int> targetPoint = AlgorithmNextPoint();
    MotionTargetLon = targetPoint.at(0);
    MotionTargetLat = targetPoint.at(1);
    
    MotionRotateToTarget();
  }
  else {
    float RotFactor = DistOffset / 5.0; if (RotFactor > 1){RotFactor = 1;}
    if (RotationAngle < 0){
      MotorGradualLeft(1 - RotFactor * 0.7);
    }
    else {
      MotorGradualRight(1 - RotFactor * 0.7);
    }
  }
  */
}

void MotionRotateToTarget(){
  float RotationAngle = ShortestRotation(MotionCurrentAzimuth, angleBetweenPoints(GpsGetLon(), GpsGetLat(), MotionTargetLon, MotionTargetLat));
  
  if (abs(RotationAngle) < 5){
    MotionMoveToTarget();
  }
  else {
    if (RotationAngle < 0){
      MotorRotate(LEFT, 1.0);
    }
    else {
      MotorRotate(RIGHT, 1.0);
    }
  }
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
  while (1){
    //If write queue is not empty send data
    /*
    char sendChar = QueueMainBluetoothReceive();
    if (sendChar != NULL){
      BluetoothWrite(sendChar);
    }

    //Read bluetooth and route all data to queue
    char receivedChar = BluetoothRead();
    if (receivedChar != NULL){
      QueueBluetoothMainSend(receivedChar);
    }
    */

    delay(10);
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