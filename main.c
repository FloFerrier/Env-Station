#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "BSP/button.h"
#include "BSP/led.h"
#include "BSP/uart.h"

#include "Debug/printf/printf.h"

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

static TaskHandle_t xTask1 = NULL;

static void vTask1(void *pvParameters);
static void vTask2(void *pvParameters);

int main(void)
{
  SystemCoreClock = rcc_ahb_frequency;
  systick_set_frequency(configTICK_RATE_HZ, SystemCoreClock);

  /* Tasks Creation */
  BaseType_t task1_status = pdFALSE;
  BaseType_t task2_status = pdFALSE;

  task1_status = xTaskCreate(vTask1,
                            "Task1",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-1,
                            &xTask1);

  task2_status = xTaskCreate(vTask2,
                            "Task2",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-1,
                            NULL);


  if((task1_status == pdPASS) &&
     (task2_status == pdPASS))
  {
    vTaskStartScheduler();
    taskENABLE_INTERRUPTS();
  }

  while(1);

  return 0;
}

void exti15_10_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  exti_reset_request(EXTI13);
  vTaskNotifyGiveFromISR(xTask1, &xHigherPriorityTaskWoken);
}

void vTask1(void *pvParameters)
{
  vLed_Setup();
  vPB_Setup();

  while(1)
  {
    /* Wait Pushed Button */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    gpio_toggle(GPIOA, GPIO5);
  }
}

void vTask2(void *pvParameters)
{
  vUART_Setup();

  printf("Debug Task 2\r\n");

  while(1)
  {
    // Nothing for the moment
  }
}
