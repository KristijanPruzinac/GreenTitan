// -------------------------------------------------------- PIN DEFINITIONS ---------------------------------------------------------------

//Relay
#define MOTOR_left 25
#define MOTOR_right 26
#define MOTOR_main 27
#define RELAY_PIN4_UNUSED 14

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

#include "Bluetooth.h";
#include "GPS.h";
#include "Controller.h";

// -------------------------------------------------------- INITIALIZATION ---------------------------------------------------------------

// -------------------------------------------------------- OBJECT DEFINITIONS ---------------------------------------------------------------

// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void InitPins(){
  pinMode(MOTOR_left, OUTPUT);
  pinMode(MOTOR_right, OUTPUT);
  pinMode(MOTOR_main, OUTPUT);

  digitalWrite(MOTOR_left, LOW);
  digitalWrite(MOTOR_right, LOW);
  digitalWrite(MOTOR_main, LOW);
}

void InitGPS(){
  Serial2.begin(9600, SERIAL_8N1, 22, 21);
}

void InitBluetooth(){
  bluetooth.begin("GreenTitan");
}

// -------------------------------------------------------- USER FUNCTIONS ---------------------------------------------------------------


// -------------------------------------------------------- MAIN PROGRAM ---------------------------------------------------------------
void setup() {
  InitPins();
  InitGPS();
  InitBluetooth();
}

void loop() {
  ControllerUpdate();
}