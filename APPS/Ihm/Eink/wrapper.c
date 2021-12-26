#include "wrapper.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "Drivers/font_defs.h"
#include "Drivers/paint.h"
#include "Debug/console.h"

#define EINK_RESOLUTION_WIDTH  (212)
#define EINK_RESOLUTION_HEIGHT (104)
#define EINK_TOTAL_SIZE_IN_BYTES (EINK_RESOLUTION_WIDTH * EINK_RESOLUTION_HEIGHT / 8)

#define EINK_STRING_LEN_MAX (255)

enum delay_unit_e {
  TICKS        = 0,
  MILLISECONDS = 1,
  SECONDS      = 2,
};

static uint8_t pBufferLUT[EINK_TOTAL_SIZE_IN_BYTES + 1] = "";
static uint8_t pImage[EINK_TOTAL_SIZE_IN_BYTES + 1] = "";

static char pBufferString[EINK_STRING_LEN_MAX + 1] = "";
static size_t uSizeString = 0;

typedef void (*uc8151d_gpios_fptr_t)(bool active);
typedef void (*uc8151d_com_fptr_t)(uint8_t data);
typedef void (*uc8151d_wait_fptr_t)(enum delay_unit_e delay_unit, uint32_t value);

struct uc8151d_t {
  uc8151d_gpios_fptr_t fctGpioReset;
  uc8151d_gpios_fptr_t fctGpioCmd;
  uc8151d_com_fptr_t fctSpiSend;
  uc8151d_wait_fptr_t fctWaitDelay;
};

void Reset_Pin_Active(bool active);
void Cmd_Pin_Active(bool active);
void Wait_Delay(enum delay_unit_e delay_unit, uint32_t value);
void Spi_Send(uint8_t data);

void Eink_Send_Cmd(struct uc8151d_t *dev, uint8_t cmd);
void Eink_Send_Data(struct uc8151d_t *dev, uint8_t* pData, uint32_t size);

void Eink_Hardware_Reset(struct uc8151d_t *dev);
void Eink_Setup(struct uc8151d_t *dev);
void Eink_Display_Clear(struct uc8151d_t *dev, uint8_t color);
void Eink_Display(struct uc8151d_t *dev);
void Eink_Power_Off(struct uc8151d_t *dev);
void Eink_Deep_Sleep(struct uc8151d_t *dev);

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

void Eink_Send_Cmd(struct uc8151d_t *dev, uint8_t cmd)
{
  if(NULL != dev)
  {
    dev->fctWaitDelay(TICKS, 1);
    dev->fctGpioCmd(true);
    dev->fctSpiSend(cmd);
    dev->fctWaitDelay(TICKS, 1);
  }
}

void Eink_Send_Data(struct uc8151d_t *dev, uint8_t* pData, uint32_t size)
{
  if((NULL != dev) && (NULL != pData))
  {
    dev->fctWaitDelay(TICKS, 1);
    dev->fctGpioCmd(false);
    for(uint32_t idx=0; idx < size; idx++)
    {
      dev->fctSpiSend(pData[idx]);
    }
    dev->fctWaitDelay(TICKS, 1);
  }
}

void Eink_Hardware_Reset(struct uc8151d_t *dev)
{
  if(NULL != dev)
  {
    dev->fctGpioReset(true);
    dev->fctWaitDelay(MILLISECONDS, 200);
    dev->fctGpioReset(false);
    dev->fctWaitDelay(MILLISECONDS, 200);
    dev->fctGpioReset(true);
    dev->fctWaitDelay(MILLISECONDS, 200);
  }
}

void Eink_Setup(struct uc8151d_t *dev)
{
  if(NULL != dev)
  {
    Eink_Send_Cmd(dev, 0x06);
    pBufferLUT[0] = 0x17;
    pBufferLUT[1] = 0x17;
    pBufferLUT[2] = 0x17;
    Eink_Send_Data(dev, pBufferLUT, 3);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x01);
    pBufferLUT[0] = 0x03;
    pBufferLUT[1] = 0x00;
    pBufferLUT[2] = 0x2B;
    pBufferLUT[3] = 0x2B;
    Eink_Send_Data(dev, pBufferLUT, 4);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x04);
    dev->fctWaitDelay(MILLISECONDS, 100);

    uint16_t busy_pin = 1;
    do {
      Eink_Send_Cmd(dev, 0x71);
      busy_pin = gpio_get(GPIOB, GPIO6);
      dev->fctWaitDelay(MILLISECONDS, 10);
    } while(!busy_pin);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x00);
    pBufferLUT[0] = 0x9F;
    Eink_Send_Data(dev, pBufferLUT, 1);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x61);
    pBufferLUT[0] = 0x68;
    pBufferLUT[1] = 0x00;
    pBufferLUT[2] = 0xD4;
    Eink_Send_Data(dev, pBufferLUT, 3);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x50);
    pBufferLUT[0] = 0xD7;
    Eink_Send_Data(dev, pBufferLUT, 1);
    dev->fctWaitDelay(MILLISECONDS, 100);
  }
}

void Eink_Power_Off(struct uc8151d_t *dev)
{
  Eink_Send_Cmd(dev, 0x02);
}

void Eink_Deep_Sleep(struct uc8151d_t *dev)
{
  Eink_Send_Cmd(dev, 0x07);
  pBufferLUT[0] = 0xA5;
  Eink_Send_Data(dev, pBufferLUT, 1);
}

void Eink_Display_Clear(struct uc8151d_t *dev, uint8_t color)
{
  /* Data start transmission 1 */
  Eink_Send_Cmd(dev, 0x10);
  memset(pBufferLUT, (int)color, (size_t)EINK_TOTAL_SIZE_IN_BYTES);
  Eink_Send_Data(dev, pBufferLUT, EINK_TOTAL_SIZE_IN_BYTES);
  dev->fctWaitDelay(MILLISECONDS, 10);

  /* Data start transmission 2 */
  Eink_Send_Cmd(dev, 0x13);
  memset(pBufferLUT, (int)color, (size_t)EINK_TOTAL_SIZE_IN_BYTES);
  Eink_Send_Data(dev, pBufferLUT, EINK_TOTAL_SIZE_IN_BYTES);
  dev->fctWaitDelay(MILLISECONDS, 100);

  /* Display refresh */
  Eink_Send_Cmd(dev, 0x12);
  dev->fctWaitDelay(MILLISECONDS, 100);

  /* Check busy pin */
  uint16_t busy_pin = 1;
  do {
    Eink_Send_Cmd(dev, 0x71);
    busy_pin = gpio_get(GPIOB, GPIO6);
    dev->fctWaitDelay(MILLISECONDS, 10);
  } while(!busy_pin);
  dev->fctWaitDelay(MILLISECONDS, 100);
}

void Eink_Display(struct uc8151d_t *dev)
{
  /* Data start transmission 1 */
  Eink_Send_Cmd(dev, 0x10);
  memset(pBufferLUT, (int)0xFF, (size_t)EINK_TOTAL_SIZE_IN_BYTES);
  Eink_Send_Data(dev, pBufferLUT, EINK_TOTAL_SIZE_IN_BYTES);
  dev->fctWaitDelay(MILLISECONDS, 10);

  /* Data start transmission 2 */
  Eink_Send_Cmd(dev, 0x13);
  Eink_Send_Data(dev, pImage, EINK_TOTAL_SIZE_IN_BYTES);
  dev->fctWaitDelay(MILLISECONDS, 100);

  /* Display refresh */
  Eink_Send_Cmd(dev, 0x12);
  dev->fctWaitDelay(MILLISECONDS, 100);

  /* Check busy pin */
  uint16_t busy_pin = 1;
  do {
    Eink_Send_Cmd(dev, 0x71);
    busy_pin = gpio_get(GPIOB, GPIO6);
    dev->fctWaitDelay(MILLISECONDS, 10);
  } while(!busy_pin);
  dev->fctWaitDelay(MILLISECONDS, 100);
}

void vTaskIhmEink(void *pvParameters)
{
  (void) pvParameters;
  console_debug("[EINK] Start Task\r\n");

  struct uc8151d_t dev;
  dev.fctGpioCmd = Cmd_Pin_Active;
  dev.fctGpioReset = Reset_Pin_Active;
  dev.fctSpiSend = Spi_Send;
  dev.fctWaitDelay = Wait_Delay;

  Eink_Hardware_Reset(&dev);
  Wait_Delay(SECONDS, 2);
  console_debug("[EINK] Hardware reset\r\n");

  Eink_Setup(&dev);
  Wait_Delay(MILLISECONDS, 200);
  console_debug("[EINK] Setup finish\r\n");

  Eink_Display_Clear(&dev, WHITE);
  console_debug("[EINK] Display clear\r\n");
  Wait_Delay(MILLISECONDS, 100);

  memset(pImage, (int)WHITE, (size_t)EINK_TOTAL_SIZE_IN_BYTES);
  Paint_NewImage(pImage, (uint16_t)104, (uint16_t)212, ROTATE_90, WHITE);
  uSizeString = snprintf(pBufferString, EINK_STRING_LEN_MAX, "Temp: 18degC Hum: 51rH Pres: 1200hPa");
  if(0 != Paint_Draw_String((uint16_t)40, (uint16_t)20, pBufferString, uSizeString, &Font12, WHITE, BLACK))
  {
    console_debug("[EINK] Fail to display... \r\n");
  }
  else
  {
    Eink_Display(&dev);
    console_debug("[EINK] Display [%d] %s\r\n", uSizeString, pBufferString);
  }
  Wait_Delay(MILLISECONDS, 100);

  Eink_Power_Off(&dev);
  console_debug("[EINK] Power off\r\n");
  Wait_Delay(MILLISECONDS, 100);

  Eink_Deep_Sleep(&dev);
  console_debug("[EINK] Deep sleep\r\n");
  Wait_Delay(MILLISECONDS, 100);

  while(1)
  {
    console_debug("[EINK] Loop !\r\n");
    Wait_Delay(SECONDS, 10);
  }
}
