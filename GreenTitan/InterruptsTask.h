//FUNCTION DECLARATIONS
void InterruptsSetup();
void InterruptsRainSensor();
void InterruptsLowPower();
void InterruptsMowingTimeFrame();
void InterruptsObstacle();
void InterruptsGPSAccuracyLoss();
void InterruptsMowerLifted();
void QueueInterruptsMain();
void InterruptsTask(void* pvParameters);

int INTERRUPTS_SAMPLING_RATE = 10;  //GLOBAL

void InterruptsSetup(){}
 
void InterruptsRainSensor(){}
void InterruptsLowPower(){}
void InterruptsMowingTimeFrame(){}
void InterruptsObstacle(){}
void InterruptsGPSAccuracyLoss(){}
void InterruptsMowerLifted(){}

void QueueInterruptsMain(){}

void InterruptsTask(void* pvParameters){
  while (1){
    //Read sensors
    BatteryUpdate();

    //Interrupts
    if (STATUS_BATTERY_LOW) InterruptsLowPower();

    delay(INTERRUPTS_SAMPLING_RATE);
  }
}