#ifndef APPS_DEBUG_CONSOLE_H
#define APPS_DEBUG_CONSOLE_H

#define BUFFER_CONSOLE_LEN_MAX (255)

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void uart2_send(const char *buffer);
void console_debug(const char *format, ...);

QueueHandle_t xQueueConsoleDebug;
void vTaskConsoleDebug(void *pvParameters);

#endif /* APPS_DEBUG_CONSOLE_H */
