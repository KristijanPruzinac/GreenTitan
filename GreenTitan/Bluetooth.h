#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial bluetooth;

String BluetoothRead(){
  return bluetooth.readString();
}

void BluetoothSend(String message){
  bluetooth.println(message);
}