#pragma once
#include "FreeRTOS.h"

SemaphoreHandle_t xSemaphoreCreateMutex();
SemaphoreHandle_t xSemaphoreCreateBinary();
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);
#define xSemaphoreCreateRecursiveMutex() xSemaphoreCreateMutex()
#define xSemaphoreTakeRecursive(s, ticks) xSemaphoreTake(s, ticks)
#define xSemaphoreGiveRecursive(s) xSemaphoreGive(s)
#define vSemaphoreDelete(s) vQueueDelete(s)
