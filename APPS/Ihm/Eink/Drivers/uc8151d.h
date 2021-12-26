#ifndef EINK_UC8151D_H
#define EINK_UC8151D_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

enum delay_unit_e {
  TICKS        = 0,
  MILLISECONDS = 1,
  SECONDS      = 2,
};

typedef void (*uc8151d_gpios_out_fptr_t)(bool active);
typedef bool (*uc8151d_gpios_in_fptr_t)(void);
typedef void (*uc8151d_com_fptr_t)(uint8_t data);
typedef void (*uc8151d_wait_fptr_t)(enum delay_unit_e delay_unit, uint32_t value);

struct uc8151d_t {
  uc8151d_gpios_out_fptr_t fctGpioReset;
  uc8151d_gpios_out_fptr_t fctGpioCmd;
  uc8151d_gpios_in_fptr_t fctGpioBusy;
  uc8151d_com_fptr_t fctSpiSend;
  uc8151d_wait_fptr_t fctWaitDelay;
  uint8_t *pBuffer;
  uint32_t uSizeBuffer;
};

void Eink_Hardware_Reset(struct uc8151d_t *dev);
void Eink_Setup(struct uc8151d_t *dev);
void Eink_Display_Clear(struct uc8151d_t *dev, uint8_t color);
void Eink_Display(struct uc8151d_t *dev, uint8_t *pImage, uint32_t uSizeImage);
void Eink_Power_Off(struct uc8151d_t *dev);
void Eink_Deep_Sleep(struct uc8151d_t *dev);

#endif /* EINK_UC8151D_H */