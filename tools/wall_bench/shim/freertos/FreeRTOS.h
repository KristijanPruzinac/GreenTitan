// Wall bench: FreeRTOS on a simulated clock (bench.cpp). Tasks are threads run in lockstep: only
// one of them or the bench runs at a time, so critical sections and mutexes never contend.
#pragma once
#include <stdint.h>
#include <stddef.h>

typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdFAIL 0
#define portMAX_DELAY 0xFFFFFFFFu
#define configTICK_RATE_HZ 1000
#define portTICK_PERIOD_MS 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

typedef struct { int unused; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED { 0 }
#define portENTER_CRITICAL(mux) ((void)(mux))
#define portEXIT_CRITICAL(mux) ((void)(mux))
#define portENTER_CRITICAL_ISR(mux) ((void)(mux))
#define portEXIT_CRITICAL_ISR(mux) ((void)(mux))
#define taskENTER_CRITICAL(mux) ((void)(mux))
#define taskEXIT_CRITICAL(mux) ((void)(mux))

typedef struct sim_task* TaskHandle_t;
typedef struct sim_queue* QueueHandle_t;
typedef struct sim_queue* SemaphoreHandle_t;
typedef struct sim_stream* StreamBufferHandle_t;
typedef void (*TaskFunction_t)(void*);

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "../esp_timer.h"   // esp_dds.h needs it and includes only FreeRTOS
