#ifndef APPS_COMM_RN4871_H
#define APPS_COMM_RN4871_H

#define BUFFER_UART_LEN_MAX (255)

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

QueueHandle_t xQueueCommUartRx;
QueueHandle_t xQueueCommUartTx;

EventGroupHandle_t xEventsCommRn4871;

int8_t rn4871_send_data(const char *buffer, const int buffer_size);
void vTaskCommRn4871(void *pvParameters);

#endif /* APPS_COMM_RN4871_H */
