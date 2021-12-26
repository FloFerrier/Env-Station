#include "uc8151d.h"

static void Eink_Send_Cmd(struct uc8151d_t *dev, uint8_t cmd);
static void Eink_Send_Data(struct uc8151d_t *dev, uint8_t* pData, uint32_t size);
static void Eink_Wait_On_Busy(struct uc8151d_t *dev);

static void Eink_Send_Cmd(struct uc8151d_t *dev, uint8_t cmd)
{
  if(NULL != dev)
  {
    dev->fctWaitDelay(TICKS, 1);
    dev->fctGpioCmd(true);
    dev->fctSpiSend(cmd);
    dev->fctWaitDelay(TICKS, 1);
  }
}

static void Eink_Send_Data(struct uc8151d_t *dev, uint8_t* pData, uint32_t size)
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

static void Eink_Wait_On_Busy(struct uc8151d_t *dev)
{
  if(NULL != dev)
  {
    bool busy_pin = false;
    do {
      Eink_Send_Cmd(dev, 0x71);
      busy_pin = dev->fctGpioBusy();
      dev->fctWaitDelay(MILLISECONDS, 10);
    } while(busy_pin);
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
  if((NULL != dev) && (NULL != dev->pBuffer) && (dev->uSizeBuffer >= 4))
  {
    Eink_Send_Cmd(dev, 0x06);
    dev->pBuffer[0] = 0x17;
    dev->pBuffer[1] = 0x17;
    dev->pBuffer[2] = 0x17;
    Eink_Send_Data(dev, dev->pBuffer, 3);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x01);
    dev->pBuffer[0] = 0x03;
    dev->pBuffer[1] = 0x00;
    dev->pBuffer[2] = 0x2B;
    dev->pBuffer[3] = 0x2B;
    Eink_Send_Data(dev, dev->pBuffer, 4);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x04);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Wait_On_Busy(dev);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x00);
    dev->pBuffer[0] = 0x9F;
    Eink_Send_Data(dev, dev->pBuffer, 1);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x61);
    dev->pBuffer[0] = 0x68;
    dev->pBuffer[1] = 0x00;
    dev->pBuffer[2] = 0xD4;
    Eink_Send_Data(dev, dev->pBuffer, 3);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, 0x50);
    dev->pBuffer[0] = 0xD7;
    Eink_Send_Data(dev, dev->pBuffer, 1);
    dev->fctWaitDelay(MILLISECONDS, 100);
  }
}

void Eink_Power_Off(struct uc8151d_t *dev)
{
  if(NULL != dev)
  {
    Eink_Send_Cmd(dev, 0x02);
  }
}

void Eink_Deep_Sleep(struct uc8151d_t *dev)
{
  if((NULL != dev) && (NULL != dev->pBuffer) && (dev->uSizeBuffer >= 1))
  {
    Eink_Send_Cmd(dev, 0x07);
    dev->pBuffer[0] = 0xA5;
    Eink_Send_Data(dev, dev->pBuffer, 1);
  }
}

void Eink_Display_Clear(struct uc8151d_t *dev, uint8_t color)
{
  if((NULL != dev) && (NULL != dev->pBuffer))
  {
    /* Data start transmission 1 */
    Eink_Send_Cmd(dev, 0x10);
    memset(dev->pBuffer, (int)color, (size_t)dev->uSizeBuffer);
    Eink_Send_Data(dev, dev->pBuffer, dev->uSizeBuffer);
    dev->fctWaitDelay(MILLISECONDS, 10);

    /* Data start transmission 2 */
    Eink_Send_Cmd(dev, 0x13);
    memset(dev->pBuffer, (int)color, (size_t)dev->uSizeBuffer);
    Eink_Send_Data(dev, dev->pBuffer, dev->uSizeBuffer);
    dev->fctWaitDelay(MILLISECONDS, 100);

    /* Display refresh */
    Eink_Send_Cmd(dev, 0x12);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Wait_On_Busy(dev);
    dev->fctWaitDelay(MILLISECONDS, 100);
  }
}

void Eink_Display(struct uc8151d_t *dev, uint8_t *pImage, uint32_t uSizeImage)
{
  if((NULL != dev) && (NULL != dev->pBuffer) && (NULL != pImage))
  {
    /* Data start transmission 1 */
    Eink_Send_Cmd(dev, 0x10);
    memset(dev->pBuffer, (int)0xFF, (size_t)dev->uSizeBuffer);
    Eink_Send_Data(dev, dev->pBuffer, dev->uSizeBuffer);
    dev->fctWaitDelay(MILLISECONDS, 10);

    /* Data start transmission 2 */
    Eink_Send_Cmd(dev, 0x13);
    Eink_Send_Data(dev, pImage, uSizeImage);
    dev->fctWaitDelay(MILLISECONDS, 100);

    /* Display refresh */
    Eink_Send_Cmd(dev, 0x12);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Wait_On_Busy(dev);
    dev->fctWaitDelay(MILLISECONDS, 100);
  }
}