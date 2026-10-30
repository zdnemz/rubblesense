// Minimal FreeRTOS declarations so device-only code paths can be syntax-checked on a PC.
#pragma once
#include <stdint.h>
typedef int  portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY 0xFFFFFFFFu
