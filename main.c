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
#include "BSP/adc.h"

#include "Driver/GA1A1S202WP/ga1a1s202wp.h"

#include "Debug/printf/printf.h"

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

static TaskHandle_t xTask1 = NULL;
static QueueHandle_t xQueue1 = NULL;

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

  xQueue1 = xQueueCreate(10, sizeof(uint32_t));

  if((task1_status == pdPASS) &&
     (task2_status == pdPASS) &&
     (xQueue1 != NULL))
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

void adc_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t raw_value = 0;

  if(adc_get_flag(ADC1, ADC_SR_EOC))
  {
    adc_clear_flag(ADC1, ADC_SR_EOC);
    raw_value = adc_read_regular(ADC1);
    xQueueSendFromISR(xQueue1, &raw_value,&xHigherPriorityTaskWoken);
  }
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
  vADC_Setup();

  uint32_t raw_value = 0;
  uint32_t volt_value = 0;
  double lux_value = 0;

  printf("Debug Task 2\r\n");

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(2500));
    adc_start_conversion_regular(ADC1);
    xQueueReceive(xQueue1, &raw_value, portMAX_DELAY);
    volt_value = xAdcRawToVolt(raw_value);
    lux_value = xVoltToLux(volt_value);
    printf("[LUX] Lux value : %.0lf\r\n", lux_value);
  }
}
