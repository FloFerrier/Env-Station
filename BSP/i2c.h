#ifndef I2C_H
#define I2C_H

#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

void vI2C_Setup(void);
/* Functions used by BME680 driver */
int8_t xI2C_Read(uint8_t dev_id,
                     uint8_t reg_addr,
                     uint8_t *reg_data,
                     uint16_t len);
int8_t xI2C_Write(uint8_t dev_id,
                      uint8_t reg_addr,
                      uint8_t *reg_data,
                      uint16_t len);

#endif /* I2C_H */
