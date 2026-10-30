#pragma once
#include "FreeRTOS.h"

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
BaseType_t xQueueSend(QueueHandle_t q, const void* item, TickType_t ticks);
BaseType_t xQueueReceive(QueueHandle_t q, void* item, TickType_t ticks);
BaseType_t xQueuePeek(QueueHandle_t q, void* item, TickType_t ticks);
BaseType_t xQueueOverwrite(QueueHandle_t q, const void* item);
BaseType_t xQueueReset(QueueHandle_t q);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q);
void vQueueDelete(QueueHandle_t q);
#define xQueueSendToBack(q, item, ticks) xQueueSend(q, item, ticks)
#define xQueueSendFromISR(q, item, woken) xQueueSend(q, item, 0)
