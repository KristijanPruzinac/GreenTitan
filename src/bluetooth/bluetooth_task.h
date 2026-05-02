//TODO: Port to wifi server

#ifndef BLUETOOTH_TASK_H
#define BLUETOOTH_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"
 
#include "BluetoothSerial.h"

extern HardwareSerial SerialDebug;

bool InitBluetooth();
String BluetoothRead();
void BluetoothWrite(String message);
void bluetooth_task(void* pvParameters);

#endif