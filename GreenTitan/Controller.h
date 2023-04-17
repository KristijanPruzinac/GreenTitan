//Timers
unsigned long UpdateInterval = 20; //Delay in ms before the MCU reads new sensor data
unsigned long PreviousTime = 0;
unsigned long CurrentTime = 0;

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
    String message = BluetoothRead();
    
    //No new message
    if (message.length() == 0 || message.equals("")){ return; }

    Serial.println(message + " " + message.length());

/*
    if (message.equals("left")){
      digitalWrite(MOTOR_left, HIGH);
    }
    else if (message.equals("right")){
      digitalWrite(MOTOR_right, HIGH);
    }
    else if (message.equals("main")){
      //digitalWrite(MOTOR_main, HIGH);

      digitalWrite(MOTOR_left, LOW);
      digitalWrite(MOTOR_right, LOW);
      digitalWrite(MOTOR_main, LOW);
    }
    else if (message.equals("stop")){
      digitalWrite(MOTOR_left, LOW);
      digitalWrite(MOTOR_right, LOW);
      digitalWrite(MOTOR_main, LOW);
    }
    */
  }
}

void ControllerParseGPS(){
  //processGPS();

  if ( processGPS() ) {
    Serial.print("iTOW:");      Serial.print(posllh.iTOW);
    Serial.print(" lat/lon: "); Serial.print(posllh.lat/10000000.0f); Serial.print(","); Serial.print(posllh.lon/10000000.0f);
    Serial.print(" height: ");  Serial.print(posllh.height/1000.0f);
    Serial.print(" hMSL: ");    Serial.print(posllh.hMSL/1000.0f);
    Serial.print(" hAcc: ");    Serial.print(posllh.hAcc/1000.0f);
    Serial.print(" vAcc: ");    Serial.print(posllh.vAcc/1000.0f);
    Serial.println();
  }
}

//Update
void ControllerUpdate(){
  //Sensor tick
  if (ControllerTickReady()){
    ControllerParseBluetooth();
    ControllerParseGPS();
  }
}