#include "MotionTask.h"

float MowerRotAngle = 0;
float MowerRotSpeed = 0;
float MowerRotAcc = 0;
float MowerIMUHeading = 0;

float MowerGPSHeading = 0;

float MowerHeading = 0;

int MotionPrevLon;
int MotionPrevLat;

int MotionTargetLon;
int MotionTargetLat;

int MotionMode = WAITING;

void MotionSetMode(int mode){
  if (mode == WAITING){
    MotorStop();
  }

  MotionMode = mode;
}

void MotionUpdateSensorData(){
  //Grab IMU data
  xSemaphoreTake(IMUMutex, portMAX_DELAY);
  MowerRotSpeed = IMURotSpeed;
  MowerRotAcc = IMURotAcc;

  //Grab GPS data
  xSemaphoreTake(GPSMutex, portMAX_DELAY);

  //If GPS heading changed adjust IMU heading
  if (fabs(MowerGPSHeading - GPS_Heading) > 0.1){
    IMUHeading = NormalizeAngle(IMUHeading - ShortestRotation(GPS_Heading, IMUHeading) * GPS_HEADING_CORRECTION_FACTOR);
    Serial.println("Updated heading");
  }

  //MowerGPSHeading = GPS_Heading;
  MowerHeading = IMUHeading;

  xSemaphoreGive(GPSMutex);
  xSemaphoreGive(IMUMutex);

  //Serial.print("0 360 ");
  //Serial.print(MowerHeading); Serial.print(" "); Serial.println(MowerGPSHeading);
}

void MotionSetTarget(int tLon, int tLat){
  MotionPrevLon = GpsGetLon();
  MotionPrevLat = GpsGetLat();

  MotionTargetLon = tLon;
  MotionTargetLat = tLat;

  //MotionMode = ROTATING;// TODO: Uncomment
  MotionSetMode(MOVING);
  //MotionSetMode(TEST);
}


class SV_A_Controller {
private:
  float Position;
  float Target;

  float Speed;
  float MaxSpeed;

  float Acceleration;
  float MaxAcceleration;
  float NormalAccelerationFactor;

  float Reaction;
  
public:
  SV_A_Controller(float Position, float Target, float MaxSpeed, float MaxAcceleration, float NormalAccelerationFactor, float Reaction) {
    this->Position = Position;
    this->Target = Target;
    
    this->MaxSpeed = MaxSpeed;
    
    this->MaxAcceleration = MaxAcceleration;
    this->NormalAccelerationFactor = NormalAccelerationFactor;
    
    this->Reaction = Reaction;
    
    this->Acceleration = 0;
  }

  void SetTarget(float Target){
    this->Target = Target;
  }
  
  // Calculations
  void Update(float Position, float Speed, float TimeElapsed) {
    this->Position = Position;
    this->Acceleration = (Speed - this->Speed) / TimeElapsed;
    this->Speed = Speed;
    
    // Minimum path needed to stop
    float StoppingAcceleration = MaxAcceleration * NormalAccelerationFactor;
    float StoppingSpeed = MaxSpeed;
    float StoppingPath = pow(StoppingSpeed, 2) / (2 * StoppingAcceleration);
    
    // If in range of stopping path, adjust speed linearly
    float TargetSpeed;
    float DistanceToTarget = Target - Position;
    if (abs(DistanceToTarget) < StoppingPath) {
      TargetSpeed = ((Target - Position) / StoppingPath) * MaxSpeed;
    }
    // Else speed up to MaxSpeed
    else {
      TargetSpeed = (Target - Position) / abs(Target - Position) * MaxSpeed;
    }
    
    // Match speed
    this->Acceleration = constrain(((TargetSpeed - Speed) / MaxSpeed) * MaxAcceleration * Reaction, -MaxAcceleration, MaxAcceleration);
  }
  
  float GetAcceleration() {
    return this->Acceleration;
  }
};

SV_A_Controller* MotionController;

/*
class S_x_Controller {
private:
  float Variable;
  
  float TargetPosition;
  float MaxPosition;
  
  float Reaction;

public:
  S_x_Controller(float Variable, float TargetPosition, float MaxPosition, float Reaction){
    this->Variable = Variable;
    
    this->TargetPosition = TargetPosition;
    this->MaxPosition = MaxPosition;
    
    this->Reaction = Reaction;
  }
  
  //Calculations
  void Update(float Position, float TimeElapsed){
    this->Variable += ((TargetPosition - Position) / MaxPosition) * Reaction;
  }

  void SetTarget(float TargetPosition){
    this->TargetPosition = TargetPosition;
  }
  
  float GetVariable(){
    return this->Variable;
  }
};

S_x_Controller* AccelerationController;
*/

void MotionTask(void* pvParameters){

  //MotionController = new SV_A_Controller(0, 0, MOTION_CORRECTION_SPEED, MOTION_CORRECTION_ACCELERATION, MOTION_CORRECTION_ACCELERATION_FACTOR, MOTION_CORRECTION_REACTION);
  //AccelerationController = new S_x_Controller(0, 0, MOTION_CORRECTION_ACCELERATION, ACCELERATION_CORRECTION_REACTION);

  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / MOTION_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    //Sensor data
    MotionUpdateSensorData();

    if (MotionMode == ROTATING){
      MotorRotate(ShortestRotation(0, MowerHeading) / 180);
    }
    else if (MotionMode == MOVING){
      float PrevTargetAngle = AngleBetweenPoints(MotionPrevLon, MotionPrevLat, MotionTargetLon, MotionTargetLat);
      float MowerTargetAngle = AngleBetweenPoints(GpsGetLon(), GpsGetLat(), MotionTargetLon, MotionTargetLat);
      float AngleDiff = ShortestRotation(PrevTargetAngle, MowerTargetAngle);
      
      float MowerTargetDist = sqrt(pow(GpsGetLon() - MotionTargetLon, 2) + pow(GpsGetLat() - MotionTargetLat, 2));
      
      float DistAint = MowerTargetDist * cos(radians(abs(AngleDiff))); //Distance left to target
      float DistOffset = MowerTargetDist * sin(radians(abs(AngleDiff))); //Distance from expected line to 
      /*
      //STRAYED FROM PATH
      if (DistOffset > MAX_DEVIATION){
        //MotorStop();
        //Error("Mower strayed from path!");
        
        //TODO: Remove
        //BluetoothWrite("Mower strayed from path!");
      }

      if (counter % 2 == 0){//TODO: Remove
        //toSend += String(DistOffset * (AngleDiff / fabs(AngleDiff))) + "\n";
        toSend += String(MowerRotAcc) + "\n";
      }
      counter++;

      if (counter >= 5){
        counter = 0;

        BluetoothWrite(toSend);

        toSend = "";
      }
      */
      
      if (DistAint < MOTION_ACCEPTED_DIST_TO_POINT || abs(AngleDiff) > 90.0){
        MotorStop();
        MotionSetMode(WAITING);

        //TODO: Remove
        BluetoothWrite("Ended motion.");

      }
      else {
        //float AngleToAdd = (ShortestRotation(MowerTargetAngle, PrevTargetAngle) / fabs(ShortestRotation(MowerTargetAngle, PrevTargetAngle))) * DistOffset*DistOffset*DistOffset/80.0;
        float AngleToAdd = (ShortestRotation(MowerTargetAngle, PrevTargetAngle) / fabs(ShortestRotation(MowerTargetAngle, PrevTargetAngle))) * DistOffset * 3;
        AngleToAdd = constrain(AngleToAdd, -90, 90);
        float CurrentTargetAngle = NormalizeAngle(PrevTargetAngle + AngleToAdd);

        //MotorDriveAngle(ShortestRotation(MowerTargetAngle, MowerHeading) * 3, FORWARD, 1.0);
        MotorDriveAngle(ShortestRotation(0, MowerHeading) * 3, FORWARD, 1.0);
      }
    }
    /*
    else if (MotionMode == TEST){
      //TODO: Remove
      if (startMillis == 0){
        startMillis = millis();
      }

      if ((millis() - startMillis) / 1000 == 0){
        AccelerationController->SetTarget(MOTION_CORRECTION_ACCELERATION);
      }
      else if ((millis() - startMillis) / 1000 == 2){
        AccelerationController->SetTarget(0);
      }
      else if ((millis() - startMillis) / 1000 == 4){
        AccelerationController->SetTarget(-MOTION_CORRECTION_ACCELERATION);
      }
      else if ((millis() - startMillis) / 1000 == 6){
        AccelerationController->SetTarget(0);
      }
      else if ((millis() - startMillis) / 1000 == 8){
        MotionSetMode(WAITING);
        startMillis = 0;
      }

      AccelerationController->Update(MowerRotAcc, 1.0 / MOTION_UPDATE_FREQUENCY);

      float ControllerDriveAngle = AccelerationController->GetVariable();
      if (ControllerDriveAngle <= 0)
        MotorRotate(LEFT, fabs(constrain(ControllerDriveAngle, -1.0, 1.0)), 1);
      else
        MotorRotate(RIGHT, fabs(constrain(ControllerDriveAngle, -1.0, 1.0)), 1);
      //MotorDriveAngle(constrain(ControllerDriveAngle, MOTOR_ANGLE_MIN, MOTOR_ANGLE_MAX), FORWARD, 1.0);
      //MotorDriveAngle(0, FORWARD, 1.0);

      if (counter % 2 == 0){//TODO: Remove
        //toSend += String(DistOffset * (AngleDiff / fabs(AngleDiff))) + "\n";
        toSend += String(MowerRotAcc) + "\n";
      }
      counter++;

      if (counter >= 10){
        counter = 0;

        BluetoothWrite(toSend);

        toSend = "";
      }
    }
    */

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