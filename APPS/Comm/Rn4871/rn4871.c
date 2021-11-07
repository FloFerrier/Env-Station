#include "rn4871.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/usart.h>

#include "Debug/console.h"
#include "Sensors/Rtc/wrapper.h"
#include "Comm/Ihm/leds.h"

#define RN4871_DELIMITER_STATUS ('%')
#define MSG_PAYLOAD_LEN_MAX (255)

struct ble_msg_s {
  uint8_t type;
  uint8_t payload_len;
  uint8_t payload[MSG_PAYLOAD_LEN_MAX+1];
};

enum FLAGS_EVENTS_RN4871 {
  FLAG_RN4871_TX = 0b000001,
  FLAG_RN4871_RX = 0b000010,
};

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
static bool _stream_open = false;
static bool _rtc_update = false;

static char buffer_uart_rx[BUFFER_UART_LEN_MAX+1] = "";
static char buffer_uart_tx[BUFFER_UART_LEN_MAX+1] = "";
static char buffer_uplink[BUFFER_UART_LEN_MAX+1] = "";

extern void usart3_isr(void);
static void uart3_send(const char *buffer);
static void rn4871_send_cmd(enum command_e cmd);
static int8_t rn4871_create_msg(const struct ble_msg_params_s *msg_params, struct ble_msg_s *msg);
static int8_t rn4871_encode_msg(const struct ble_msg_s *msg, char *buffer);
static int8_t rn4871_decode_msg(const char *buffer, struct ble_msg_s *msg);
static int8_t rn4871_process_msg(struct ble_msg_s *msg);
static void rn4871_process_resp(const char *buffer);

void usart3_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static volatile uint16_t c = 0;
  static uint16_t i = 0;
  static uint8_t cnt_delim_status = 0;

  c = usart_recv(USART3);
  buffer_uart_rx[i++] = (char)c;
  if(c == RN4871_DELIMITER_STATUS)
  {
    ++cnt_delim_status;
  }

  if((c == '>') || (cnt_delim_status == 2))
  {
    buffer_uart_rx[i] = 0;
    i = 0;
    cnt_delim_status = 0;
    xQueueSendFromISR(xQueueCommUartRx, buffer_uart_rx, &xHigherPriorityTaskWoken);
    xEventGroupSetBitsFromISR(xEventsCommRn4871, FLAG_RN4871_RX, &xHigherPriorityTaskWoken);
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
      snprintf(buffer_uplink, BUFFER_UART_LEN_MAX, "%s\r\n", TABLE_COMMAND[cmd]);
      xQueueSend(xQueueCommUartTx, buffer_uplink, 100);
      xEventGroupSetBits(xEventsCommRn4871, FLAG_RN4871_TX);
      break;
    }
    case CMD_SET_SERVICES:
    {
      /* Enable Transparent UART */
      snprintf(buffer_uplink, BUFFER_UART_LEN_MAX, "%s,C0\r\n", TABLE_COMMAND[cmd]);
      xQueueSend(xQueueCommUartTx, buffer_uplink, 100);
      xEventGroupSetBits(xEventsCommRn4871, FLAG_RN4871_TX);
      break;
    }
    default:
      break;
  }
  _current_cmd = cmd;
}

static int8_t rn4871_create_msg(const struct ble_msg_params_s *msg_params, struct ble_msg_s *msg)
{
  if((msg_params != NULL) && (msg != NULL))
  {
    switch(msg_params->type)
    {
      case MSG_TYPE_INFOS:
        /* To define */
        msg->payload_len = 0;
        break;
      case MSG_TYPE_TIME:
        /* Msg for requesting a time update */
        msg->payload[0] = 0x01;
        msg->payload_len = 1;
        break;
      case MSG_TYPE_BME680:
        msg->payload[0] = (uint8_t)((msg_params->timestamp & 0xFF000000) >> 24);
        msg->payload[1] = (uint8_t)((msg_params->timestamp & 0x00FF0000) >> 16);
        msg->payload[2] = (uint8_t)((msg_params->timestamp & 0x0000FF00) >> 8);
        msg->payload[3] = (uint8_t)((msg_params->timestamp & 0x000000FF) >> 0);
        msg->payload[4] = (uint8_t)(msg_params->temperature);
        msg->payload[5] = (uint8_t)((msg_params->pressure & 0xFF00) >> 8);
        msg->payload[6] = (uint8_t)((msg_params->pressure & 0x00FF) >> 0);
        msg->payload[7] = (uint8_t)(msg_params->humidity);
        msg->payload_len = 8;
        break;
      case MSG_TYPE_LPS33W:
        msg->payload[0] = (uint8_t)((msg_params->timestamp & 0xFF000000) >> 24);
        msg->payload[1] = (uint8_t)((msg_params->timestamp & 0x00FF0000) >> 16);
        msg->payload[2] = (uint8_t)((msg_params->timestamp & 0x0000FF00) >> 8);
        msg->payload[3] = (uint8_t)((msg_params->timestamp & 0x000000FF) >> 0);
        msg->payload[4] = (uint8_t)(msg_params->temperature);
        msg->payload[5] = (uint8_t)((msg_params->pressure & 0xFF00) >> 8);
        msg->payload[6] = (uint8_t)((msg_params->pressure & 0x00FF) >> 0);
        msg->payload_len = 7;
        break;
      default:
        return -1;
        break;
    }
    msg->type = msg_params->type;
    return 0;
  }
  return -1;
}

static int8_t rn4871_encode_msg(const struct ble_msg_s *msg, char *buffer)
{
  if((msg != NULL) && (buffer != NULL))
  {
    int idx = 0;
    idx = snprintf(&(buffer[idx]), MSG_PAYLOAD_LEN_MAX, "%02X%02X", msg->type, msg->payload_len);
    for(int i = 0; i < msg->payload_len; i++)
    {
      snprintf(&(buffer[idx]), MSG_PAYLOAD_LEN_MAX - idx, "%02X", msg->payload[i]);
      idx += 2;
    }
    return 0;
  }
  return -1;
}

static int8_t rn4871_decode_msg(const char *buffer, struct ble_msg_s *msg)
{
  if((msg != NULL) && (buffer != NULL))
  {
    static char tmp[3] = "";
    static uint8_t val = 0;
    char *ptr_end;
    char *p = strchr(buffer, ',');
    p++;
    tmp[0] = *p;
    p++;
    tmp[1] = *p;
    val = (uint8_t)strtol(tmp, &ptr_end, 16);
    if(ptr_end == tmp)
    {
      return -1;
    }
    msg->type = val;

    p++;
    tmp[0] = *p;
    p++;
    tmp[1] = *p;
    val = (uint8_t)strtol(tmp, &ptr_end, 16);
    if(ptr_end == tmp)
    {
      return -1;
    }
    msg->payload_len = val;

    for(int i = 0; i < msg->payload_len; i++)
    {
      p++;
      tmp[0] = *p;
      p++;
      tmp[1] = *p;
      val = (uint8_t)strtol(tmp, &ptr_end, 16);
      if(ptr_end == tmp)
      {
        return -1;
      }
      msg->payload[i] = val;
    }
    return 0;
  }
  return -1;
}

static int8_t rn4871_process_msg(struct ble_msg_s *msg)
{
  switch(msg->type)
  {
    case MSG_TYPE_TIME:
    {
      /* Update RTC */
      time_t epoch = (msg->payload[0] << 24) | (msg->payload[1] << 16) | (msg->payload[2] << 8) | (msg->payload[3] << 0);
      rtc_epoch_set(epoch);
      ihm_rtc_updated(true);
      _rtc_update = true;
      console_debug("[RTC] Set Epoch: %d\r\n", (uint32_t)epoch);
      break;
    }
    default :
      break;
  }

  return 0;
}

static void rn4871_process_resp(const char *buffer)
{
  struct ble_msg_s msg;
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
        break;
      default:
        _current_cmd = CMD_NONE;
        break;
    }
  }
  else if(strstr(buffer, "STREAM_OPEN") != NULL)
  {
    struct ble_msg_params_s msg_params;
    _stream_open = true;
    ihm_ble_stream(true);
    msg_params.type = MSG_TYPE_TIME;
    rn4871_send_data(&msg_params);
  }
  else if(strstr(buffer, "DISCONNECT") != NULL)
  {
    ihm_ble_connected(false);
    ihm_ble_stream(false);
    _stream_open = false;
  }
  else if(strstr(buffer, "CONNECT") != NULL)
  {
    ihm_ble_connected(true);
  }
  else if(strstr(buffer, "DATA") != NULL)
  {
    int8_t ret = rn4871_decode_msg(buffer, &msg);
    if(ret != 0)
    {
      console_debug("[RN4871] Error to decode msg...\r\n");
    }
    else
    {
      /*console_debug("[RN4871] Msg type: 0x%x Payload_len: 0x%x Payload: 0x%x%x%x%x\r\n",
        msg->type, msg->payload_len, msg->payload[0], msg->payload[1], msg->payload[2], msg->payload[3]);*/
      rn4871_process_msg(&msg);
    }
  }
}

int8_t rn4871_send_data(const struct ble_msg_params_s *msg_params)
{
  if(_stream_open != true)
  {
    return -1;
  }
  else
  {
    struct ble_msg_s msg;
    if(rn4871_create_msg(msg_params, &msg) != 0)
    {
      console_debug("[RN4871] Error to create msg ...\r\n");
      return -1;
    }

    if(rn4871_encode_msg(&msg, buffer_uplink) != 0)
    {
      console_debug("[RN4871] Error to encode msg ...\r\n");
      return -1;
    }

    xQueueSend(xQueueCommUartTx, buffer_uplink, 100);
    xEventGroupSetBits(xEventsCommRn4871, FLAG_RN4871_TX);
    return 0;
  }
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
  xEventGroupClearBits(xEventsCommRn4871, FLAG_RN4871_RX);

  while(1)
  {
    EventBits_t uxBits = xEventGroupWaitBits(xEventsCommRn4871, FLAG_RN4871_RX | FLAG_RN4871_TX, pdFALSE, pdFALSE, portMAX_DELAY);
    if(uxBits | FLAG_RN4871_RX)
    {
      xEventGroupClearBits(xEventsCommRn4871, FLAG_RN4871_RX);
      if(xQueueReceive(xQueueCommUartRx, buffer, 100) == pdPASS)
      {
        int data_size = strlen(buffer);
        console_debug("Rx -> (%d) %s\r\n", data_size, buffer);
        rn4871_process_resp(buffer);
      }
    }
    if(uxBits | FLAG_RN4871_TX)
    {
      xEventGroupClearBits(xEventsCommRn4871, FLAG_RN4871_TX);
      if(xQueueReceive(xQueueCommUartTx, buffer_uart_tx, 100) == pdPASS)
      {
        int buffer_size = strlen(buffer_uart_tx);
        console_debug("Tx <- (%d) %s\r\n", buffer_size, buffer_uart_tx);
        uart3_send(buffer_uart_tx);
      }
    }
  }
}
