//TODO: Implement functionality
void InterruptsTask(void* pvParameters){
  while (1){
    Serial.println("Interrupts task");
    delay(10);
  }
}

void InterruptsSetup(){}
 
void InterruptsRainSensor(){}
void InterruptsLowPower(){}
void InterruptsMowingTimeFrame(){}
void InterruptsObstacle(){}
void InterruptsGPSAccLoss(){}
void InterruptsMowerLifted(){}

void ReadSensors(){}
 
void QueueInterruptsMain(){}