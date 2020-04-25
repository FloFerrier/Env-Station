#ifndef INA219_H
#define INA219_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define REG_16BITS_MSB(val) ((val & 0xFF00) >> 8)
#define REG_16BITS_LSB(val) (val & 0x00FF)

#define INA219_CHIP_ADDR (0x41)

/* Register Address - 8bits */
#define INA219_REG_ADDR_CONFIG (0x00)
#define INA219_REG_ADDR_SHUNT_VOLTAGE (0x01)
#define INA219_REG_ADDR_BUS_VOLTAGE (0x02)
#define INA219_REG_ADDR_POWER (0x03)
#define INA219_REG_ADDR_CURRENT (0x04)
#define INA219_REG_ADDR_CALIBRATION (0x05)

/* Mask Bit Register */
#define INA219_REG_CONFIG_MASK_RST (1<<15)
#define INA219_REG_CONFIG_MASK_BRNG (1<<13)
#define INA219_REG_CONFIG_MASK_PG (0b11<<12)
#define INA219_REG_CONFIG_MASK_BADC (0b1111<<7)
#define INA219_REG_CONFIG_MASK_SADC (0b1111<<3)
#define INA219_REG_CONFIG_MASK_MODE (0b111<<3)

#define INA219_REG_BUS_VOLTAGE_MASK_BUS (0xFFF8)
#define INA219_REG_BUS_VOLTAGE_MASK_CNVR (1<<1)
#define INA219_REG_BUS_VOLTAGE_MASK_OVF (1<<0)

#define INA219_CURRENT_LSB (100) /* in uA */

#include "BSP/i2c.h"

#include "Tools/printf/printf.h"

struct ina219_s
{
  uint32_t voltage;
  int32_t current;
};

extern void ina219_wait_conversion(void);
void ina219_setup(void);
bool ina219_check_conversion(void);
int8_t ina219_get_data(struct ina219_s *tmp);

#endif /* INA219_H */
