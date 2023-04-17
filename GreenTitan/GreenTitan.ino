// -------------------------------------------------------- PIN DEFINITIONS ---------------------------------------------------------------

//Relay
#define MOTOR_left_A 23
#define MOTOR_left_B 22
#define MOTOR_right_A 21
#define MOTOR_right_B 19

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

#include "Sensors.h";
#include "Bluetooth.h";
#include "GPS.h";
#include "Controller.h";

// -------------------------------------------------------- INITIALIZATION ---------------------------------------------------------------

// -------------------------------------------------------- OBJECT DEFINITIONS ---------------------------------------------------------------

// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void InitPins(){
  pinMode(MOTOR_left_A, OUTPUT);
  pinMode(MOTOR_left_B, OUTPUT);
  pinMode(MOTOR_right_A, OUTPUT);
  pinMode(MOTOR_right_B, OUTPUT);

  digitalWrite(MOTOR_left_A, LOW);
  digitalWrite(MOTOR_left_B, LOW);
  digitalWrite(MOTOR_right_A, LOW);
  digitalWrite(MOTOR_right_B, LOW);

  Serial.begin(9600);
}

// -------------------------------------------------------- USER FUNCTIONS ---------------------------------------------------------------


// -------------------------------------------------------- MAIN PROGRAM ---------------------------------------------------------------
void setup() {
  InitPins();
  InitGyro();
  InitGPS();
  InitBluetooth();
}

void loop() {
  ControllerUpdate();
  /*
  char data = SerialGPS.read();
  if (data == 0xB5){
    Serial.print("\n");
  }
  else {
    Serial.print(String(int(data)) + " ");
  }
  */
}