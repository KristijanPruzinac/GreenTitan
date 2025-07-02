#include "MotionTask.h"

//Start and end point
long MotionPrevLon;
long MotionPrevLat;

long MotionTargetLon;
long MotionTargetLat;

//Current mode of motion
int MotionMode = WAITING;
int MotionMoveAfterRotation = 0;

void MotionSetMode(int mode){
  xSemaphoreTake(MotionMutex, portMAX_DELAY);
  if (mode == WAITING){
    MotorStop();
  }

  MotionMode = mode;
  xSemaphoreGive(MotionMutex);
}

void MotionSetTargetPoint(long tLon, long tLat){
  MotionPrevLon = GetLon();
  MotionPrevLat = GetLat();

  MotionTargetLon = tLon;
  MotionTargetLat = tLat;

  MotionSetMode(ROTATING);
  //MotionSetMode(MOVING);
  //MotionSetMode(TEST);
}

void MotionSetTargetPointRotation(float azimuthDegrees){
  MotionPrevLon = GetLon();
  MotionPrevLat = GetLat();

  MotionTargetLon = MotionPrevLon + 100 * sin(radians(azimuthDegrees));
  MotionTargetLat = MotionPrevLat + 100 * cos(radians(azimuthDegrees)); //Calculate target based on azimuth

  MotionMoveAfterRotation = 0; //Don't move after rotation
}

bool MowerIsInMotion(){
  bool returnValue = false;
  xSemaphoreTake(MotionMutex, portMAX_DELAY);
  if (MotionMode != WAITING){
    returnValue = true;
  }
  xSemaphoreGive(MotionMutex);

  return returnValue;
}

void MotionTask(void* pvParameters){
  String toSend = "";
  int counter = 0;
  while (1){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MILLIS_PER_SECOND / MOTION_UPDATE_FREQUENCY);

    xLastWakeTime = xTaskGetTickCount();

    //Calculate needed values for navigating
    float PrevTargetAngle = AngleBetweenPoints(MotionPrevLon, MotionPrevLat, MotionTargetLon, MotionTargetLat); //Angle from line start point to end point
    float MowerTargetAngle = AngleBetweenPoints(GetLon(), GetLat(), MotionTargetLon, MotionTargetLat); //Angle from current position to end point

    float AngleDiff = ShortestRotation(PrevTargetAngle, MowerTargetAngle);
    float MowerTargetDist = sqrt(pow(GetLon() - MotionTargetLon, 2) + pow(GetLat() - MotionTargetLat, 2));
    
    float DistAint = MowerTargetDist * cos(radians(abs(AngleDiff))); //Distance on line remaining to target
    float DistOffset = MowerTargetDist * sin(radians(abs(AngleDiff))); //Distance from current position to line 

    if (MotionMode == ROTATING){
      if (fabs(ShortestRotation(GetHeading(), MowerTargetAngle)) <= MOTION_ACCEPTED_ROTATION_TO_POINT){
        if (MotionMoveAfterRotation){
          MotionSetMode(MOVING);
        }
        else {
          MotorStop();
          MotionSetMode(WAITING);
        }
      }
      else {
        MotorRotate(ShortestRotation(MowerTargetAngle, GetHeading()) / 180);
      }
    }
    else if (MotionMode == MOVING){
      /*
      //STRAYED FROM PATH
      //TODO: Implement
      if (DistOffset > MAX_DEVIATION){
        //MotorStop();
        //Error("Mower strayed from path!");
        
        //TODO: Remove
        //BluetoothWrite("Mower strayed from path!");
      }
*/
      if (counter % 6 == 0){//TODO: Remove
        float DistanceFromLineToSend = (ShortestRotation(MowerTargetAngle, PrevTargetAngle) / fabs(ShortestRotation(MowerTargetAngle, PrevTargetAngle))) * DistOffset; if (isnan(DistanceFromLineToSend)) DistanceFromLineToSend = 0;
        toSend += String(DistanceFromLineToSend) + "\n";
      }
      counter++;

      if (counter >= 15){
        counter = 0;

        BluetoothWrite(toSend);

        toSend = "";
      }
      
      
      if (DistAint < MOTION_ACCEPTED_DIST_TO_POINT){
        MotorStop();
        MotionSetMode(WAITING);

      }
      else {
        //float AngleToAdd = (ShortestRotation(MowerTargetAngle, PrevTargetAngle) / fabs(ShortestRotation(MowerTargetAngle, PrevTargetAngle))) * DistOffset*DistOffset*DistOffset/80.0;
        float AngleToAdd = (ShortestRotation(MowerTargetAngle, PrevTargetAngle) / fabs(ShortestRotation(MowerTargetAngle, PrevTargetAngle))) * DistOffset / 3.0;
        AngleToAdd = constrain(AngleToAdd, -30, 30);
        float CurrentTargetAngle = NormalizeAngle(PrevTargetAngle - AngleToAdd);

        MotorDriveAngle(ShortestRotation(CurrentTargetAngle, GetHeading()) * 5, FORWARD, 1.0);
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