#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "BSP/led.h"
#include "BSP/uart.h"
#include "BSP/gpio.h"
#include "BSP/i2c.h"
#include "BSP/adc.h"
#include "BSP/rtc.h"

#include "Drivers/GA1A1S202WP/ga1a1s202wp.h"
#include "Drivers/BME680/bme680.h"
#include "Drivers/HC05/hc05.h"

#include "Tools/msg-protocol/msg-protocol.h"

#include "Tools/printf/printf.h"

/* Task Stack Size */
#define TASK_BME680_STACK configMINIMAL_STACK_SIZE
#define TASK_GA1A1S202WP_STACK configMINIMAL_STACK_SIZE
#define TASK_COMMUNICATOR_STACK configMINIMAL_STACK_SIZE
#define TASK_SUPERVISOR_STACK configMINIMAL_STACK_SIZE
#define TASK_RTC_UPDATER_STACK configMINIMAL_STACK_SIZE

/* Task Priorities */
#define TASK_BME680_PRIO  (configMAX_PRIORITIES-5)
#define TASK_GA1A1S202WP_PRIO  (configMAX_PRIORITIES-6)
#define TASK_COMMUNICATOR_PRIO (configMAX_PRIORITIES-2)
#define TASK_SUPERVISOR_PRIO (configMAX_PRIORITIES-4)
#define TASK_RTC_UPDATER_PRIO (configMAX_PRIORITIES-3)

#define QUEUE_ITEM_MAX_TO_SEND 20 // n latest data ready to be send

typedef struct
{
  char id;
  uint32_t value;
} sensor_measure_s;

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

TaskHandle_t xTaskBME680 = NULL;
TaskHandle_t xTaskLUX = NULL;

TimerHandle_t xTimerSampling = NULL;

QueueHandle_t xQueueAdcSensorLux = NULL;
QueueHandle_t xQueueUartIsrRx = NULL;
QueueHandle_t xQueueSensorsToSupervisor = NULL;
QueueHandle_t xQueueSupervisorToCommunicator = NULL;

void adc_isr(void);
void usart3_isr(void);

void vTaskBME680(void *pvParameters);
void vTaskSensorLux(void *pvParameters);
void vTaskSupervisor(void *pvParameters);
void vTaskCommunicator(void *pvParameters);
void vTaskRtcUpdate(void *pvParameters);

void vTimerCallback(TimerHandle_t xTimer);

void HC05_Receive(char *p_str);
void HC05_Timer(uint32_t time);
void user_delay_ms(uint32_t period);

#endif /* MAIN_H */
