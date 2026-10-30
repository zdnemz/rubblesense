#pragma once
#include "FreeRTOS.h"
typedef void* SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateBinary();
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks);
