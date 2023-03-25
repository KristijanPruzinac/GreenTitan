// -------------------------------------------------------- PIN DEFINITIONS ---------------------------------------------------------------

//Relay
#define MOTOR_left 25
#define MOTOR_right 26
#define MOTOR_main 27
#define RELAY_PIN4_UNUSED 14

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------

#include "Bluetooth.h";
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

void InitBluetooth(){
  bluetooth.begin("GreenTitan");
}

// -------------------------------------------------------- USER FUNCTIONS ---------------------------------------------------------------


// -------------------------------------------------------- MAIN PROGRAM ---------------------------------------------------------------
void setup() {
  InitPins();
  InitBluetooth();
}

void loop() {
  ControllerUpdate();
}