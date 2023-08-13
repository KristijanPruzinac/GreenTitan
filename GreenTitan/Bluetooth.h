#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void InitBluetooth() {
  SerialBT.begin("GreenTitan"); //Bluetooth device name
  Serial.println("Bluetooth started");
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
  SerialBT.print(message + "<");
}
void Respond(String MessageGroup, String ActionGroup){
  BluetoothWrite(MessageGroup + "/" + ActionGroup);
}
void Respond(String MessageGroup, String ActionGroup, String DataGroup){
  BluetoothWrite(MessageGroup + "/" + ActionGroup + "/" + DataGroup);
}
