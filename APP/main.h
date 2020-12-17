#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
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
#include "BSP/rtc.h"

#include "Drivers/BME680/wrapper.h"

/* Task Stack Size */
#define TASK_CONSOLE_DBG_STACK configMINIMAL_STACK_SIZE
#define TASK_BME680_STACK configMINIMAL_STACK_SIZE
#define TASK_RTC_UPDATER_STACK configMINIMAL_STACK_SIZE

/* Task Priorities */
#define TASK_CONSOLE_DBG_PRIO  (configMAX_PRIORITIES-6)
#define TASK_BME680_PRIO  (configMAX_PRIORITIES-5)
#define TASK_RTC_UPDATER_PRIO (configMAX_PRIORITIES-3)

struct sensor_measure_s
{
  char id;
  uint32_t value;
};

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

TaskHandle_t xTaskBME680 = NULL;

TimerHandle_t xTimerSampling = NULL;

QueueHandle_t xQueueUartIsrRx = NULL;
QueueHandle_t xQueueUartTx = NULL;

void usart3_isr(void);

void console_dbg(const char *format, ...);

void vTaskConsoleDbg(void *pvParameters);
void vTaskBME680(void *pvParameters);
void vTaskRtcUpdate(void *pvParameters);

void vTimerCallback(TimerHandle_t xTimer);

#endif /* MAIN_H */
