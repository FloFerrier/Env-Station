#include "wrapper.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "Drivers/uc8151d.h"
#include "Drivers/font_defs.h"
#include "Drivers/paint.h"
#include "Debug/console.h"

#define EINK_RESOLUTION_WIDTH  (212)
#define EINK_RESOLUTION_HEIGHT (104)
#define EINK_TOTAL_SIZE_IN_BYTES (EINK_RESOLUTION_WIDTH * EINK_RESOLUTION_HEIGHT / 8)

#define EINK_STRING_LEN_MAX (255)

static uint8_t pBufferLUT[EINK_TOTAL_SIZE_IN_BYTES + 1] = "";
static uint8_t pImageLUT[EINK_TOTAL_SIZE_IN_BYTES + 1] = "";

static char pBufferString[EINK_STRING_LEN_MAX + 1] = "";
static size_t uSizeString = 0;

void Reset_Pin_Active(bool active);
void Cmd_Pin_Active(bool active);
bool Busy_Pin_Active(void);
void Wait_Delay(enum delay_unit_e delay_unit, uint32_t value);
void Spi_Send(uint8_t data);

void Reset_Pin_Active(bool active)
{
  if(active)
    gpio_set(GPIOC, GPIO7);
  else
    gpio_clear(GPIOC, GPIO7);
}

void Cmd_Pin_Active(bool active)
{
  if(active)
    gpio_clear(GPIOA, GPIO8);
  else
    gpio_set(GPIOA, GPIO8);
}

bool Busy_Pin_Active(void)
{
  uint16_t busy_pin = gpio_get(GPIOB, GPIO6);
  if(busy_pin)
    return false;
  else
    return true;
}

void Wait_Delay(enum delay_unit_e delay_unit, uint32_t value)
{
  switch (delay_unit)
  {
    case TICKS:
	  vTaskDelay(value);
	  break;
	case MILLISECONDS:
	  vTaskDelay(pdMS_TO_TICKS(value));
	  break;
	case SECONDS:
	  vTaskDelay(pdMS_TO_TICKS(value * 1000));
	  break;
  default:
	  break;
  }
}

void Spi_Send(uint8_t data)
{
  spi_send(SPI2, data);
}

void eink_display_data(const struct ble_msg_params_s *msg_params)
{
  xQueueSend(xQueueSensorData, msg_params, 100);
}

void vTaskIhmEink(void *pvParameters)
{
  (void) pvParameters;
  console_debug("[EINK] Start Task\r\n");

  struct uc8151d_t dev;
  dev.fctGpioBusy = Busy_Pin_Active;
  dev.fctGpioCmd = Cmd_Pin_Active;
  dev.fctGpioReset = Reset_Pin_Active;
  dev.fctSpiSend = Spi_Send;
  dev.fctWaitDelay = Wait_Delay;
  dev.pBuffer = pBufferLUT;
  dev.uSizeBuffer = EINK_TOTAL_SIZE_IN_BYTES;

  Eink_Hardware_Reset(&dev);
  Wait_Delay(SECONDS, 2);
  console_debug("[EINK] Hardware reset\r\n");

  Eink_Setup(&dev);
  Wait_Delay(MILLISECONDS, 200);
  console_debug("[EINK] Setup finish\r\n");

  Eink_Display_Clear(&dev, WHITE);
  console_debug("[EINK] Display clear\r\n");
  Wait_Delay(MILLISECONDS, 100);

  Eink_Power_Off(&dev);
  console_debug("[EINK] Power off\r\n");
  Wait_Delay(MILLISECONDS, 100);

  struct ble_msg_params_s msg_params;

  while(1)
  {
    if(pdPASS == xQueueReceive(xQueueSensorData, &msg_params, 100))
    {
      Eink_Hardware_Reset(&dev);
      Wait_Delay(MILLISECONDS, 200);
      console_debug("[EINK] Hardware reset\r\n");

      Eink_Setup(&dev);
      Wait_Delay(MILLISECONDS, 200);
      console_debug("[EINK] Setup finish\r\n");

      Eink_Display_Clear(&dev, WHITE);
      console_debug("[EINK] Display clear\r\n");
      Wait_Delay(MILLISECONDS, 100);

      memset(pImageLUT, (int)WHITE, (size_t)EINK_TOTAL_SIZE_IN_BYTES);
      Paint_NewImage(pImageLUT, (uint16_t)104, (uint16_t)212, ROTATE_90, WHITE);

      if(MSG_TYPE_BME680 == msg_params.type)
      {
        uSizeString = snprintf(pBufferString, EINK_STRING_LEN_MAX, "BME680 Timestamp : %ld seconds Temp: %d degC Hum: %d rH Pres: %d hPa",
          msg_params.timestamp, msg_params.temperature, msg_params.humidity, msg_params.pressure);
      }
      else
      {
        uSizeString = snprintf(pBufferString, EINK_STRING_LEN_MAX, "No sensor data available");
      }
      if(0 != Paint_Draw_String((uint16_t)40, (uint16_t)20, pBufferString, uSizeString, &Font12, WHITE, BLACK))
      {
        console_debug("[EINK] Fail to display... \r\n");
      }
      else
      {
        Eink_Display(&dev, pImageLUT, EINK_TOTAL_SIZE_IN_BYTES);
        console_debug("[EINK] Display [%d] %s\r\n", uSizeString, pBufferString);
      }
      Wait_Delay(MILLISECONDS, 100);

      Eink_Power_Off(&dev);
      console_debug("[EINK] Power off\r\n");
      Wait_Delay(MILLISECONDS, 100);
    }
  }
}
