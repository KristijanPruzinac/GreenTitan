//TODO: Port to wifi server

#ifndef BLUETOOTH_TASK_H
#define BLUETOOTH_TASK_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"

#include "esp_dds.h"
#include "esp_timer.h"
 
#include "BluetoothSerial.h"

#include "../algorithm/algorithm.h"

extern HardwareSerial SerialDebug;

extern bool pending_path_publish;

bool InitBluetooth();
String BluetoothRead();
void BluetoothWrite(String message);
void bluetooth_task(void* pvParameters);

#endif