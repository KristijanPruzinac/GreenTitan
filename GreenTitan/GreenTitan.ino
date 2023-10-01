// -------------------------------------------------------- USER VARIABLES ---------------------------------------------------------------

String MowerStatus = "PAUSED";
bool MowerExecutingPath = false;

// -------------------------------------------------------- PIN DEFINITIONS ---------------------------------------------------------------

//Relay
#define MOTOR_left_A 23
#define MOTOR_left_B 22
#define MOTOR_right_A 21
#define MOTOR_right_B 19
#define MOTOR_main 18

//Battery voltage pin
#define BATTERY_voltage_pin 32

// -------------------------------------------------------- DEPENDENCIES ---------------------------------------------------------------
#include "Algorithm.h";

#include "Bluetooth.h";

#include "Sensors.h";
#include "GPS.h";

#include "Motors.h";
#include "Functions.h";

#include "Controller.h";
// -------------------------------------------------------- INIT FUNCTIONS ---------------------------------------------------------------

void InitPins(){
  pinMode(MOTOR_left_A, OUTPUT);
  pinMode(MOTOR_left_B, OUTPUT);
  pinMode(MOTOR_right_A, OUTPUT);
  pinMode(MOTOR_right_B, OUTPUT);
  pinMode(MOTOR_main, OUTPUT);

  digitalWrite(MOTOR_left_A, LOW);
  digitalWrite(MOTOR_left_B, LOW);
  digitalWrite(MOTOR_right_A, LOW);
  digitalWrite(MOTOR_right_B, LOW);
  digitalWrite(MOTOR_main, LOW);

  Serial.begin(9600);
}



// -------------------------------------------------------- MAIN TASK ---------------------------------------------------------------
void setup() {
  InitPins();
  ControllerInit();
}

void loop() {
  ControllerUpdate();
}

