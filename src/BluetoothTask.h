//TODO: Port to wifi server

#ifndef BLUETOOTH_TASK_H
#define BLUETOOTH_TASK_H

#include "Arduino.h"
#include "Defines.h"
#include "BluetoothSerial.h"

bool InitBluetooth();
String BluetoothRead();
void BluetoothWrite(String message);
void BluetoothTask(void* pvParameters);

#endif