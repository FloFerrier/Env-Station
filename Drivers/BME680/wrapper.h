#ifndef BME680_WRAPPER_H
#define BME680_WRAPPER_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>

#include "BSP/i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bme680.h"

struct bme680_s
{
  uint32_t temperature;
  uint32_t pressure;
  uint32_t humidity;
};

int8_t xBme680_Setup(void);
int8_t xBme680_Get_Data(struct bme680_s *data);

#endif /* BME680_WRAPPER_H */
