#include "i2c.h"

void vI2C_Setup(void)
{
  /* PB8 = SCL (D15) and PB9 = SDA (D14) */
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
  gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);

  rcc_periph_clock_enable(RCC_I2C1);
  i2c_peripheral_disable(I2C1);
  i2c_reset(I2C1);
  i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / 1e6);
  i2c_peripheral_enable(I2C1);
}

int8_t xI2C_Read(uint8_t dev_id,
                     uint8_t reg_addr,
                     uint8_t *reg_data,
                     uint16_t len)
{
  i2c_transfer7(I2C1, dev_id, &reg_addr, 1, NULL, 0);
  i2c_transfer7(I2C1, dev_id, NULL, 0, reg_data, (size_t)len);

  return 0;
}

int8_t xI2C_Write(uint8_t dev_id,
                      uint8_t reg_addr,
                      uint8_t *reg_data,
                      uint16_t len)
{
  uint8_t buffer[len + 1];

  memcpy(buffer, &reg_addr, 1);
  memcpy(&(buffer[1]), reg_data, len);

  i2c_transfer7(I2C1, dev_id, buffer, (size_t)(len + 1), NULL, 0);

  return 0;
}
