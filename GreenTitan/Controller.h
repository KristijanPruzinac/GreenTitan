//Timers
unsigned long UpdateInterval = 200; //Delay in ms before the MCU reads new sensor data
unsigned long PreviousTime = 0;
unsigned long CurrentTime = 0;

int pointsCount = 0;
int gcodeIndex = 0;
long pointsX[50];
long pointsY[50];

//Check if controller should read new sensor data
bool ControllerTickReady(){
  CurrentTime = millis();

  //If overflow happened (every 50 days), tick
  if (CurrentTime < PreviousTime){
    PreviousTime = millis();
    CurrentTime = millis();
    return true;
  }
  //If update interval time passed, tick
  else if (CurrentTime >= PreviousTime + UpdateInterval){
    PreviousTime = millis();
    CurrentTime = millis();
    return true;
  }

  return false;
}

//Parse bluetooth messages
void ControllerParseBluetooth(){
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

void ControllerParseGPS(){
  processGPS();
}

void ControllerParseGyro(){
  GyroRead();
}

void InitVoltageSensor(){
  pinMode(BATTERY_voltage_pin, INPUT);
}

void ControllerInit(){
  InitGyro();
  InitGPS();
  InitBluetooth();
  InitVoltageSensor();
}

//Update
void ControllerUpdate(){
  //Sensor tick
  if (ControllerTickReady()){
    ControllerParseBluetooth();
    ControllerParseGPS();
    //ControllerParseGyro();
    
    //GCODE
    if (MowerExecutingPath && MowerStatus == "RUNNING"){
      //End of path
      if (gcodeIndex >= pointsCount){
        MowerExecutingPath = false;
        return;
      }

      //Current on point (+- 5cm)
      if (absVal(posllh.lon - pointsX[gcodeIndex]) < 5 && absVal(posllh.lat - pointsY[gcodeIndex]) < 5){
        //Increment index
        gcodeIndex++;
        return;
      }

      //Adjust angle
      float currentAngle = posllhHeading;
      float targetAngle = angleBetweenPoints(posllh.lon, posllh.lat, pointsX[gcodeIndex], pointsY[gcodeIndex]);

      //Shortest rotation
      String rotationDirection = ShortestRotation(currentAngle, targetAngle);

      if (rotationDirection == "STRAIGHT"){
        MotorForward();
      }
      else if (rotationDirection == "CW"){
        MotorPivotRight();
      }
      else if (rotationDirection == "CCW"){
        MotorPivotLeft();
      }
    }
  }
}
