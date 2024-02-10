#include "MainTask.h"

/*
int pointsCount = 0;
int gcodeIndex = 0;
long pointsX[50];
long pointsY[50];

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

//TODO: Implement functionality
String Mode = "POWER_ON";
/* CHARGING STOP PAUSE START RUNNING POWER_ON SETUP*/

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

char QueueBluetoothMainReceive(){
  char returnChar = NULL;
  xQueueReceive(BluetoothMainQueue, &returnChar, 50);

  return returnChar;
}

void QueueMainBluetoothSend(char receivedChar){
  xQueueSend(BluetoothMainQueue, &receivedChar, 50);
}

void MainTask(void* pvParameters){
  while (1){
    Serial.println("Main task");
    delay(10);
  }
}