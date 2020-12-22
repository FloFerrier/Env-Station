#include "rn4871.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/usart.h>

#include "Debug/console.h"

enum command_e {
  CMD_NONE,
  CMD_MODE_ENTER, /* $$$ */
  CMD_MODE_QUIT, /* --- */
  CMD_REBOOT, /* R,1 */
  CMD_RESET_FACTORY, /* SF */
  CMD_SET_BT_NAME, /* S- */
  CMD_SET_DEVICE_NAME, /* SN */
  CMD_GET_DEVICE_NAME, /* GN */
  CMD_SET_SERVICES, /* SS */
  CMD_DUMP_INFOS, /* D */
  CMD_GET_VERSION, /* V */
};

const char TABLE_COMMAND[][10] = {
  "",
  "$$$",
  "---",
  "R,1",
  "SF",
  "S-",
  "SN",
  "GN",
  "SS",
  "D",
  "V",
};

static enum command_e _current_cmd = CMD_NONE;
static bool _data_mode = false;

static char buffer_uart_rx[BUFFER_UART_LEN_MAX+1] = "";
static char buffer_uart_tx[BUFFER_UART_LEN_MAX+1] = "";
static char buffer_uplink[BUFFER_UART_LEN_MAX+1] = "";

extern void usart3_isr(void);
static void uart3_send(const char *buffer);
static void rn4871_send_cmd(enum command_e cmd);
static void rn4871_process_resp(const char *buffer);

void usart3_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static volatile uint16_t c = 0;
  static uint16_t i = 0;
  static uint8_t cnt = 0;

  c = usart_recv(USART3);
  buffer_uart_rx[i++] = (char)c;
  if(c == '%')
  {
    ++cnt;
  }

  if((c == '>') || (cnt == 2))
  {
    buffer_uart_rx[i] = 0;
    i = 0;
    cnt = 0;
    xQueueSendFromISR(xQueueCommUartRx, buffer_uart_rx, &xHigherPriorityTaskWoken);
    xEventGroupSetBitsFromISR(xEventsComm, FLAG_COMM_RX, &xHigherPriorityTaskWoken);
  }
  else if(i >= (BUFFER_UART_LEN_MAX - 1))
  {
    i = 0;
  }
}

static void uart3_send(const char *buffer)
{
  if(buffer != NULL)
  {
    int buffer_size = strnlen(buffer, BUFFER_UART_LEN_MAX);
    for(int i = 0; i < buffer_size; i++)
    {
      usart_send_blocking(USART3, (uint16_t)buffer[i]);
    }
  }
}

static void rn4871_send_cmd(enum command_e cmd)
{
  switch(cmd)
  {
    case CMD_MODE_QUIT:
    case CMD_REBOOT:
    case CMD_DUMP_INFOS:
    case CMD_GET_DEVICE_NAME:
    case CMD_GET_VERSION:
    {
      rn4871_send_data("%s\r\n", TABLE_COMMAND[cmd]);
      break;
    }
    case CMD_SET_SERVICES:
    {
      /* Enable Transparent UART */
      rn4871_send_data("%s,C0\r\n", TABLE_COMMAND[cmd]);
      break;
    }
    default:
      break;
  }
  _current_cmd = cmd;
}

static void rn4871_process_resp(const char *buffer)
{
  enum command_e cmd = _current_cmd;
  if((strstr(buffer, "CMD>") != NULL) || (strstr(buffer, "REBOOT") != NULL))
  {
    switch(cmd)
    {
      case CMD_MODE_ENTER:
        rn4871_send_cmd(CMD_DUMP_INFOS);
        break;
      case CMD_DUMP_INFOS:
        rn4871_send_cmd(CMD_GET_DEVICE_NAME);
        break;
      case CMD_GET_DEVICE_NAME:
        rn4871_send_cmd(CMD_GET_VERSION);
        break;
      case CMD_GET_VERSION:
        rn4871_send_cmd(CMD_SET_SERVICES);
        break;
      case CMD_SET_SERVICES:
        rn4871_send_cmd(CMD_REBOOT);
        break;
      case CMD_REBOOT:
        _current_cmd = CMD_NONE;
        _data_mode = true;
        break;
      default:
        _current_cmd = CMD_NONE;
        break;
    }
  }
}

void rn4871_send_data(const char *format, ...)
{
  va_list args;
  va_start (args, format);
  vsnprintf(buffer_uplink, BUFFER_UART_LEN_MAX, format, args);
  xQueueSend(xQueueCommUartTx, buffer_uplink, 100);
  xEventGroupSetBits(xEventsComm, FLAG_COMM_TX);
  va_end(args);
}

void vTaskCommRn4871(void *pvParameters)
{
  (void) pvParameters;

  console_debug("[RN4871] Start task\r\n");

  static char buffer[BUFFER_UART_LEN_MAX] = "";

  /* Send fake data for testing if command mode is available */
  uart3_send("\r\n");
  if(xQueueReceive(xQueueCommUartRx, buffer, pdMS_TO_TICKS(100)) != pdTRUE)
  {
    console_debug("Tx <- $$$\r\n");
    uart3_send("$");
    vTaskDelay(pdMS_TO_TICKS(100));
    uart3_send("$$");
    _current_cmd = CMD_MODE_ENTER;
  }
  else
  {
    rn4871_send_cmd(CMD_DUMP_INFOS);
  }
  xEventGroupClearBits(xEventsComm, FLAG_COMM_RX);

  while(1)
  {
    EventBits_t uxBits = xEventGroupWaitBits(xEventsComm, FLAG_COMM_RX | FLAG_COMM_TX, pdFALSE, pdFALSE, portMAX_DELAY);
    if(uxBits | FLAG_COMM_RX)
    {
      xEventGroupClearBits(xEventsComm, FLAG_COMM_RX);
      if(xQueueReceive(xQueueCommUartRx, buffer, 100) == pdPASS)
      {
        int data_size = strlen(buffer);
        console_debug("Rx -> (%d) %s\r\n", data_size, buffer);
        rn4871_process_resp(buffer);
      }
    }
    if(uxBits | FLAG_COMM_TX)
    {
      xEventGroupClearBits(xEventsComm, FLAG_COMM_TX);
      if(xQueueReceive(xQueueCommUartTx, buffer_uart_tx, 100) == pdPASS)
      {
        int buffer_size = strlen(buffer_uart_tx);
        console_debug("Tx <- (%d) %s\r\n", buffer_size, buffer_uart_tx);
        uart3_send(buffer_uart_tx);
      }
    }
  }
}
