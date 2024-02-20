#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "Arduino.h"
#include "Defines.h"

extern QueueHandle_t BluetoothMainQueue;
extern QueueHandle_t MainBluetoothQueue;

extern QueueHandle_t SensorsMainQueue;

void MainCharging();
void MainChargingStart();
void MainChargingStop();
void MainStop();
void MainPause();
void MainStart();
void MainRunning();
void MainPowerOn();
void MainSetup();
char QueueBluetoothMainReceive();
void QueueMainBluetoothSend(char receivedChar);
void MainTask(void* pvParameters);

#endif