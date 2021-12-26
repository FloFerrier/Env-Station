#include "uc8151d.h"

enum uc8151d_reg_cmd_e {
  UC8151D_REG_CMD_PSR    = (0x00),
  UC8151D_REG_CMD_PWR    = (0x01),
  UC8151D_REG_CMD_POF    = (0x02),
  UC8151D_REG_CMD_PFS    = (0x03),
  UC8151D_REG_CMD_PON    = (0x04),
  UC8151D_REG_CMD_PMES   = (0x05),
  UC8151D_REG_CMD_BTST   = (0x06),
  UC8151D_REG_CMD_DSLP   = (0x07),
  UC8151D_REG_CMD_DTM1   = (0x10),
  UC8151D_REG_CMD_DSP    = (0x11),
  UC8151D_REG_CMD_DRF    = (0x12),
  UC8151D_REG_CMD_DTM2   = (0x13),
  UC8151D_REG_CMD_AUTO   = (0x17),
  UC8151D_REG_CMD_LUTOTP = (0x2A),
  UC8151D_REG_CMD_PLL    = (0x30),
  UC8151D_REG_CMD_TSC    = (0x40),
  UC8151D_REG_CMD_TSE    = (0x41),
  UC8151D_REG_CMD_TSW    = (0x42),
  UC8151D_REG_CMD_TSR    = (0x43),
  UC8151D_REG_CMD_PBC    = (0x44),
  UC8151D_REG_CMD_CDI    = (0x50),
  UC8151D_REG_CMD_LPD    = (0x51),
  UC8151D_REG_CMD_TCON   = (0x60),
  UC8151D_REG_CMD_TRES   = (0x61),
  UC8151D_REG_CMD_GSST   = (0x65),
  UC8151D_REG_CMD_REV    = (0x70),
  UC8151D_REG_CMD_FLG    = (0x71),
  UC8151D_REG_CMD_AMV    = (0x80),
  UC8151D_REG_CMD_VV     = (0x81),
  UC8151D_REG_CMD_VDCS   = (0x82),
  UC8151D_REG_CMD_PTL    = (0x90),
  UC8151D_REG_CMD_PTIN   = (0x91),
  UC8151D_REG_CMD_PTOUT  = (0x92),
  UC8151D_REG_CMD_PGM    = (0xA0),
  UC8151D_REG_CMD_APG    = (0xA1),
  UC8151D_REG_CMD_ROTP   = (0xA2),
  UC8151D_REG_CMD_CCSET  = (0xE0),
  UC8151D_REG_CMD_PWS    = (0xE3),
  UC8151D_REG_CMD_LVSEL  = (0xE4),
  UC8151D_REG_CMD_TSSET  = (0xE5),
};

static void Eink_Send_Cmd(struct uc8151d_t *dev, enum uc8151d_reg_cmd_e cmd);
static void Eink_Send_Data(struct uc8151d_t *dev, uint8_t* pData, uint32_t size);
static void Eink_Wait_On_Busy(struct uc8151d_t *dev);

static void Eink_Send_Cmd(struct uc8151d_t *dev, enum uc8151d_reg_cmd_e cmd)
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
      Eink_Send_Cmd(dev, UC8151D_REG_CMD_FLG);
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
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_BTST);
    dev->pBuffer[0] = 0x17;
    dev->pBuffer[1] = 0x17;
    dev->pBuffer[2] = 0x17;
    Eink_Send_Data(dev, dev->pBuffer, 3);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, UC8151D_REG_CMD_PWR);
    dev->pBuffer[0] = 0x03;
    dev->pBuffer[1] = 0x00;
    dev->pBuffer[2] = 0x2B;
    dev->pBuffer[3] = 0x2B;
    Eink_Send_Data(dev, dev->pBuffer, 4);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, UC8151D_REG_CMD_PON);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Wait_On_Busy(dev);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, UC8151D_REG_CMD_PSR);
    dev->pBuffer[0] = 0x9F;
    Eink_Send_Data(dev, dev->pBuffer, 1);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, UC8151D_REG_CMD_TRES);
    dev->pBuffer[0] = 0x68;
    dev->pBuffer[1] = 0x00;
    dev->pBuffer[2] = 0xD4;
    Eink_Send_Data(dev, dev->pBuffer, 3);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Send_Cmd(dev, UC8151D_REG_CMD_CDI);
    dev->pBuffer[0] = 0xD7;
    Eink_Send_Data(dev, dev->pBuffer, 1);
    dev->fctWaitDelay(MILLISECONDS, 100);
  }
}

void Eink_Power_Off(struct uc8151d_t *dev)
{
  if(NULL != dev)
  {
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_POF);
  }
}

void Eink_Deep_Sleep(struct uc8151d_t *dev)
{
  if((NULL != dev) && (NULL != dev->pBuffer) && (dev->uSizeBuffer >= 1))
  {
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_DSLP);
    dev->pBuffer[0] = 0xA5;
    Eink_Send_Data(dev, dev->pBuffer, 1);
  }
}

void Eink_Display_Clear(struct uc8151d_t *dev, uint8_t color)
{
  if((NULL != dev) && (NULL != dev->pBuffer))
  {
    /* Data start transmission 1 */
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_DTM1);
    memset(dev->pBuffer, (int)color, (size_t)dev->uSizeBuffer);
    Eink_Send_Data(dev, dev->pBuffer, dev->uSizeBuffer);
    dev->fctWaitDelay(MILLISECONDS, 10);

    /* Data start transmission 2 */
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_DTM2);
    memset(dev->pBuffer, (int)color, (size_t)dev->uSizeBuffer);
    Eink_Send_Data(dev, dev->pBuffer, dev->uSizeBuffer);
    dev->fctWaitDelay(MILLISECONDS, 100);

    /* Display refresh */
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_DRF);
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
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_DTM1);
    memset(dev->pBuffer, (int)0xFF, (size_t)dev->uSizeBuffer);
    Eink_Send_Data(dev, dev->pBuffer, dev->uSizeBuffer);
    dev->fctWaitDelay(MILLISECONDS, 10);

    /* Data start transmission 2 */
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_DTM2);
    Eink_Send_Data(dev, pImage, uSizeImage);
    dev->fctWaitDelay(MILLISECONDS, 100);

    /* Display refresh */
    Eink_Send_Cmd(dev, UC8151D_REG_CMD_DRF);
    dev->fctWaitDelay(MILLISECONDS, 100);

    Eink_Wait_On_Busy(dev);
    dev->fctWaitDelay(MILLISECONDS, 100);
  }
}