#ifndef APPS_COMM_RN4871_H
#define APPS_COMM_RN4871_H

#define BUFFER_UART_LEN_MAX (255)

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

enum msg_type_ble_e {
  MSG_TYPE_INFOS = 0x00,
  MSG_TYPE_TIME = 0x01,
  MSG_TYPE_BME680 = 0x02,
  MSG_TYPE_LPS33W = 0x03,
  MSG_TYPE_SGP30 = 0x04,
  MSG_TYPE_VEML7700 = 0x05,
};

struct ble_msg_params_s {
  uint8_t type;
  uint32_t timestamp;
  int8_t temperature;
  uint16_t pressure;
  uint8_t humidity;
};

QueueHandle_t xQueueCommUartRx;
QueueHandle_t xQueueCommUartTx;

EventGroupHandle_t xEventsCommRn4871;

int8_t rn4871_send_data(const struct ble_msg_params_s *msg_params);
void vTaskCommRn4871(void *pvParameters);

#endif /* APPS_COMM_RN4871_H */
