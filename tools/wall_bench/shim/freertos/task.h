#pragma once
#include "FreeRTOS.h"

typedef enum { eNoAction, eSetBits, eIncrement, eSetValueWithOverwrite, eSetValueWithoutOverwrite } eNotifyAction;

BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action);
BaseType_t xTaskNotifyWait(uint32_t clear_on_entry, uint32_t clear_on_exit, uint32_t* value, TickType_t ticks);
#define xTaskNotifyFromISR(task, value, action, woken) xTaskNotify(task, value, action)
TaskHandle_t xTaskGetCurrentTaskHandle();
void vTaskDelay(TickType_t ticks);
TickType_t xTaskGetTickCount();
BaseType_t xTaskCreate(TaskFunction_t fn, const char* name, uint32_t stack, void* param, UBaseType_t prio, TaskHandle_t* out);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name, uint32_t stack, void* param, UBaseType_t prio,
                                   TaskHandle_t* out, BaseType_t core);
