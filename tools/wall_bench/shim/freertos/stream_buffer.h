#pragma once
#include "FreeRTOS.h"

StreamBufferHandle_t xStreamBufferCreate(size_t size, size_t trigger);
size_t xStreamBufferSend(StreamBufferHandle_t s, const void* data, size_t len, TickType_t ticks);
size_t xStreamBufferReceive(StreamBufferHandle_t s, void* data, size_t len, TickType_t ticks);
size_t xStreamBufferBytesAvailable(StreamBufferHandle_t s);
