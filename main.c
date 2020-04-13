#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "BSP/button.h"
#include "BSP/led.h"
#include "BSP/uart.h"
#include "BSP/gpio.h"

#include "Driver/HC05/hc05.h"

#include "Debug/printf/printf.h"

#define MAX_BUFFER_UART_RX 255

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

static TaskHandle_t xTask1 = NULL;
static QueueHandle_t xQueue2 = NULL;

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
                            configMAX_PRIORITIES-2,
                            NULL);

  xQueue2 = xQueueCreate(10, sizeof(char) * MAX_BUFFER_UART_RX);


  if((task1_status == pdPASS) &&
     (task2_status == pdPASS) &&
     (xQueue2 != NULL))
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

/* ISR for buffering received data from UART3 */
void usart3_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static volatile uint16_t c = 0;
  static uint16_t i = 0;
  static char buffer[MAX_BUFFER_UART_RX];

  c = usart_recv(USART3);
  buffer[i++] = (char)c;

  if(c == '\n')
  {
    i = 0;
    xQueueSendFromISR(xQueue2, buffer, &xHigherPriorityTaskWoken);
  }
  else if(i >= (MAX_BUFFER_UART_RX - 1))
  {
    i = 0;
  }
}

void vTask1(void *pvParameters)
{
  (void) pvParameters;
  vLed_Setup();
  vPB_Setup();
  vConsole_Setup();

  printf("Debug LED\r\n");

  while(1)
  {
    /* Wait Pushed Button */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    gpio_toggle(GPIOA, GPIO5);
  }
}

void vTask2(void *pvParameters)
{
  (void) pvParameters;

  printf("Debug HC05\r\n");
  int8_t rslt = HC05_OK;

  vGPIO_Setup();
  vUART_Setup();

  rslt = HC05_Setup("WS-V1", "0501");

  if(rslt != HC05_OK)
  {
    printf("[HC05] Setup fail ...\r\n");
  }

  printf("[HC05] Data Mode Start !\r\n");

  /* Wait the receiver outside */
  HC05_Cmp_Response("READY\r\n");

  HC05_Send_Data("Hello World !\r\n");
  if(HC05_Cmp_Response("ACK\r\n"))
  {
    printf("[HC05] ACK !\r\n");
  }
  else
  {
    printf("[HC05] No ACK ...\r\n");
  }

  vTaskDelete(NULL);
}

void HC05_Receive(char *p_str)
{
  xQueueReceive(xQueue2, p_str, portMAX_DELAY);
  //printf("%s", p_str);
}

void HC05_Timer(uint32_t time)
{
  vTaskDelay(pdMS_TO_TICKS(time));
}
