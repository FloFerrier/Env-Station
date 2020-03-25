#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "button.h"
#include "led.h"
#include "uart.h"

#include "rn4871.h"

#define MAX_BUFFER_UART_RX 20

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

  xQueue1 = xQueueCreate(10, sizeof(uint16_t) * MAX_BUFFER_UART_RX);

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

void usart2_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint16_t c = 0;
  static uint16_t count = 0;
  static uint16_t i = 0;
  static uint16_t buffer[MAX_BUFFER_UART_RX];

  c = usart_recv(USART2);
  buffer[i++] = c;

  if(c == '%')
  {
    count++;
  }

  if((c == '\n') || (count == 2))
  {
    i = 0;
    count = 0;
    xQueueSendFromISR(xQueue1, buffer, &xHigherPriorityTaskWoken);
  }
  else if(i >= (MAX_BUFFER_UART_RX - 1))
  {
    i = 0;
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
  /*static const char serv_UUID[32]  = "010203040506070809000A0B0C0D0E0F";
  static const char char_UUID[32]  = "11223344556677889900AABBCCDDEEFF";
  static const char char_prop[2]   = "12";
  static const char char_size[2]   = "34";
  static const char char_handle[4] = "000A";
  static const char char_data[2]   = "89";*/

  static uint16_t str_recv[MAX_BUFFER_UART_RX];

  vUART_Setup();

  while(1)
  {
    /* Wait to receive RN4871 response */
    xQueueReceive(xQueue1, str_recv, portMAX_DELAY);

    RN4871_Reboot();

    /*
    RN4871_Reset();
    RN4871_Get_Services();
    RN4871_New_Services(&(serv_UUID[0]));
    RN4871_New_Characteristics(&(char_UUID[0]),
                             &(char_prop[0]),
                             &(char_size[0]));
    RN4871_Write_Char(&(char_handle[0]),
                    &(char_data[0]));
    RN4871_Read_Char(&(char_handle[0]));*/
  }
}
