//TODO: Port to wifi server

//FUNCTION DECLARATIONS
void InitBluetooth();
char BluetoothRead();
void BluetoothWrite(char msgChar);
void QueueBluetoothMainSend(char receivedChar);
char QueueMainBluetoothReceive();
void BluetoothTask(void* pvParameters);

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void InitBluetooth() {
  SerialBT.begin("GreenTitan"); //Bluetooth device name
  Serial.println("Bluetooth started");

  SerialBT.setTimeout(50);
}

char BluetoothRead(){
  if (SerialBT.available()){
    return SerialBT.read();
  }
  else {
    return NULL;
  }
}

void BluetoothWrite(char msgChar){
  SerialBT.print(msgChar);
}

//TODO: Move functionality to main task
/*
void BluetoothWrite(String message){
  SerialBT.print(message + "<");
}
void BluetoothRespond(String MessageGroup, String ActionGroup){
  BluetoothWrite(MessageGroup + "/" + ActionGroup);
}
void BluetoothRespond(String MessageGroup, String ActionGroup, String DataGroup){
  BluetoothWrite(MessageGroup + "/" + ActionGroup + "/" + DataGroup);
}
*/

//FreeRTOS
void QueueBluetoothMainSend(char receivedChar){
  xQueueSend(BluetoothMainQueue, &receivedChar, 50);
}

char QueueMainBluetoothReceive(){
  char returnChar = NULL;
  xQueueReceive(MainBluetoothQueue, &returnChar, 50);

  return returnChar;
}

void BluetoothTask(void* pvParameters){
  while (1){
    //If write queue is not empty send data
    char sendChar = QueueMainBluetoothReceive();
    if (sendChar != NULL){
      BluetoothWrite(sendChar);
    }

    //Read bluetooth and route all data to queue
    char receivedChar = BluetoothRead();
    if (receivedChar != NULL){
      QueueBluetoothMainSend(receivedChar);
    }
  }
}