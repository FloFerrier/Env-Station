#ifndef I2C_H
#define I2C_H

#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

void vI2C_Setup(void);

#endif /* I2C_H */
