#ifndef INA219_DEFS_H
#define INA219_DEFS_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define _REG_16BITS_TO_8BITS_MSB(val) ((val & 0xFF00) >> 8)
#define _REG_16BITS_TO_8BITS_LSB(val) (val & 0x00FF)
#define _8BITS_TO_16BITS_REG(msb, lsb) ((msb << 8) | lsb)

/* Register Address - 8bits */
#define INA219_REG_ADDR_CONFIGURATION (0x00)
#define INA219_REG_ADDR_SHUNT_VOLTAGE (0x01)
#define INA219_REG_ADDR_BUS_VOLTAGE (0x02)
#define INA219_REG_ADDR_POWER (0x03)
#define INA219_REG_ADDR_CURRENT (0x04)
#define INA219_REG_ADDR_CALIBRATION (0x05)

/* Mask Bits Register */
#define INA219_REG_CONFIG_BIT_RST (1<<15)

#define INA219_REG_BUS_VOLTAGE_BIT_CNVR (1<<1)
#define INA219_REG_BUS_VOLTAGE_BIT_OVF (1<<0)

/* Offset Bits Register */
#define INA219_REG_CONFIG_OFFSET_BRNG(val) (val<<13)
#define INA219_REG_CONFIG_OFFSET_PG(val) (val<<11)
#define INA219_REG_CONFIG_OFFSET_BADC(val) (val<<7)
#define INA219_REG_CONFIG_OFFSET_SADC(val) (val<<3)
#define INA219_REG_CONFIG_OFFSET_MODE(val) (val<<0)

#define INA219_REG_CONFIG_OFFSET_CALIB(val) (val<<1)

#define INA219_REG_MASK_CONFIG (0x3FFF)
#define INA219_REG_CONFIG_MASK_MODE (0x7)
#define INA219_REG_BUS_VOLTAGE_MASK_BUS (0xFFF8)

/* ADC Conversion Time */
#define INA219_ADC_9BITS_1SAMPLE_DUR_US (84)
#define INA219_ADC_10BITS_1SAMPLE_DUR_US (148)
#define INA219_ADC_11BITS_1SAMPLE_DUR_US (276)
#define INA219_ADC_12BITS_1SAMPLE_DUR_US (532)
#define INA219_ADC_12BITS_2SAMPLES_DUR_US (1060)
#define INA219_ADC_12BITS_4SAMPLES_DUR_US (2130)
#define INA219_ADC_12BITS_8SAMPLES_DUR_US (4260)
#define INA219_ADC_12BITS_16SAMPLES_DUR_US (8510)
#define INA219_ADC_12BITS_32SAMPLES_DUR_US (17020)
#define INA219_ADC_12BITS_64SAMPLES_DUR_US (34050)
#define INA219_ADC_12BITS_128SAMPLES_DUR_US (68100)

#define INA219_BUS_VOLTAGE_LSB (4) /* in mV */
#define INA219_CURRENT_LSB (100) /* in uA */
#define INA219_POWER_LSB (20*INA219_CURRENT_LSB) /* in uW */

/* Value Register in default -- Used as Chip ID */
#define INA219_REG_VAL_DEFAULT_CONFIG (0x399f)

typedef int8_t (*ina219_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*ina219_delay_fptr_t)(uint32_t period);

#endif /* INA219_DEFS_H */