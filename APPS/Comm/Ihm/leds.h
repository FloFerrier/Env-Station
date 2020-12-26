#ifndef APPS_COMM_IHM_LEDS_H
#define APPS_COMM_IHM_LEDS_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

void ihm_ble_connected(bool status);
void ihm_ble_stream(bool status);
void ihm_system_problem(bool status);
void ihm_rtc_updated(bool status);

void vTaskCommIhm(void *pvParameters);

#endif /* APPS_COMM_IHM_LEDS_H */
