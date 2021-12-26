#ifndef EINK_WRAPPER_H
#define EINK_WRAPPER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Comm/Rn4871/rn4871.h"

QueueHandle_t xQueueSensorData;

void eink_display_data(const struct ble_msg_params_s *msg_params);
void vTaskIhmEink(void *pvParameters);

#endif /* EINK_WRAPPER_H */