#include "BluetoothTask.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

bool InitBluetooth() {
  bool BeginSuccess = SerialBT.begin("GreenTitan"); //Bluetooth device name

  if (!BeginSuccess){
    return false;
  }

  SerialBT.setTimeout(COMMUNICATION_TIMEOUT);

  return true;
}

String BluetoothRead(){
  if (SerialBT.available()){
    return SerialBT.readStringUntil('<');
  }
  else {
    return "";
  }
}

void BluetoothWrite(String message){
  SerialBT.print(message);
}
void BluetoothRespond(String MessageGroup, String ActionGroup){
  BluetoothWrite(MessageGroup + "/" + ActionGroup);
}
void BluetoothRespond(String MessageGroup, String ActionGroup, String DataGroup){
  BluetoothWrite(MessageGroup + "/" + ActionGroup + "/" + DataGroup);
}

void BluetoothTask(void* pvParameters){
  while (1){

    delay(10);
  }
}