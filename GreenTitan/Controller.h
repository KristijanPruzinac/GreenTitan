//Timers
unsigned long UpdateInterval = 20; //Delay in ms before the MCU reads new sensor data
unsigned long PreviousTime = 0;
unsigned long CurrentTime = 0;

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

void ControllerParseBluetooth(){
  String message = BluetoothRead();

  //No new message
  if (message.length() == 0){ return; }

  Serial.println(message + " " + message.length());

  if (message.equals("left")){
    digitalWrite(MOTOR_left, HIGH);
  }
  else if (message.equals("right")){
    digitalWrite(MOTOR_right, HIGH);
  }
  else if (message.equals("main")){
    digitalWrite(MOTOR_main, HIGH);
  }
  else if (message.equals("stop")){
    digitalWrite(MOTOR_left, LOW);
    digitalWrite(MOTOR_right, LOW);
    digitalWrite(MOTOR_main, LOW);
  }
}

void ControllerUpdate(){
  //Sensor tick
  if (ControllerTickReady()){
    ControllerParseBluetooth();
  }
}