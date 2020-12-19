#include "console.h"

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/usart.h>

static char buffer_uart_tx[BUFFER_CONSOLE_LEN_MAX+1] = "";
static char buffer_console_debug[BUFFER_CONSOLE_LEN_MAX+1] = "";

extern QueueHandle_t xQueueConsoleDebug;
extern void vTaskConsoleDebug(void *pvParameters);

void uart2_send(const char *buffer)
{
  if(buffer != NULL)
  {
    int buffer_size = strnlen(buffer, BUFFER_CONSOLE_LEN_MAX);
    for(int i = 0; i < buffer_size; i++)
    {
      usart_send_blocking(USART2, (uint16_t)buffer[i]);
    }
  }
}

void console_debug(const char *format, ...)
{
  va_list args;
  va_start (args, format);
  vsnprintf(buffer_console_debug, BUFFER_CONSOLE_LEN_MAX, format, args);
  xQueueSend(xQueueConsoleDebug, buffer_console_debug, 100);
  va_end(args);
}

void vTaskConsoleDebug(void *pvParameters)
{
  (void) pvParameters;

  while(1)
  {
    xQueueReceive(xQueueConsoleDebug, buffer_uart_tx, portMAX_DELAY);
    uart2_send(buffer_uart_tx);
  }
}
