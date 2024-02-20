//TODO: Port to wifi server

#ifndef BLUETOOTH_TASK_H
#define BLUETOOTH_TASK_H

#include "Arduino.h"
#include "Defines.h"
#include "BluetoothSerial.h"
/*
extern QueueHandle_t BluetoothMainQueue;
extern QueueHandle_t MainBluetoothQueue;
*/
void InitBluetooth();
char BluetoothRead();
void BluetoothWrite(char msgChar);
void QueueBluetoothMainSend(char receivedChar);
char QueueMainBluetoothReceive();
void BluetoothTask(void* pvParameters);

#endif