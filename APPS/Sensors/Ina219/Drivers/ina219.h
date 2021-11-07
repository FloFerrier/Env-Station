#ifndef INA219_H
#define INA219_H

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

#include "ina219_defs.h"

#define INA219_OK (0)
#define INA219_I2C_ERROR (-1)
#define INA219_HARD_ERROR (-2)
#define INA219_SOFT_ERROR (-3)

enum sensor_mode_e
{
  POWER_DOWN = (0x0),
  SHUNT_TRIGGERED = (0x1),
  BUS_TRIGGERED = (0x2),
  SHUNT_BUS_TRIGGERED = (0x3),
  ADC_DISABLE = (0x4),
  SHUNT_CONTINUOUS = (0x5),
  BUS_CONTINUOUS = (0x6),
  SHUNT_BUS_CONTINUOUS = (0x7),
};

enum bus_voltage_range_e
{
  BUS_VOLTAGE_RANGE_16_V = (0x0),
  BUS_VOLTAGE_RANGE_32_V = (0x1),
};

enum shunt_voltage_range_e
{
  SHUNT_VOLTAGE_RANGE_40_MV = (0x0),
  SHUNT_VOLTAGE_RANGE_80_MV = (0x1),
  SHUNT_VOLTAGE_RANGE_160_MV = (0x2),
  SHUNT_VOLTAGE_RANGE_320_MV = (0x3),
};

enum resolution_e
{
  RESOLUTION_9BITS_1SAMPLE = (0x0),
  RESOLUTION_10BITS_1SAMPLE = (0x1),
  RESOLUTION_11BITS_1SAMPLE = (0x2),
  RESOLUTION_12BITS_1SAMPLE = (0x3),
  RESOLUTION_12BITS_2SAMPLES = (0x9),
  RESOLUTION_12BITS_4SAMPLES = (0xA),
  RESOLUTION_12BITS_8SAMPLES = (0xB),
  RESOLUTION_12BITS_16SAMPLES = (0xC),
  RESOLUTION_12BITS_32SAMPLES = (0xD),
  RESOLUTION_12BITS_64SAMPLES = (0xE),
  RESOLUTION_12BITS_128SAMPLES = (0xF),
};

struct ina219_dev_s
{
  uint8_t slave_addr;
  ina219_com_fptr_t read;
  ina219_com_fptr_t write;
  ina219_delay_fptr_t delay_us;
};

struct ina219_params_s
{
  uint16_t calibration;
  enum sensor_mode_e sensor_mode;
  enum bus_voltage_range_e bus_voltage_range;
  enum resolution_e bus_voltage_resolution;
  enum shunt_voltage_range_e shunt_voltage_range;
  enum resolution_e shunt_voltage_resolution;
};

struct ina219_field_data_s
{
  int32_t current;
  int16_t shunt_voltage;
  uint16_t bus_voltage;
  uint16_t power;
};

int8_t ina219_init(const struct ina219_dev_s *dev);
int8_t ina219_config(const struct ina219_dev_s *dev, const struct ina219_params_s *params);
int8_t ina219_start_single_shot(const struct ina219_dev_s *dev, struct ina219_params_s *params);
int8_t ina219_get_sensor_data(const struct ina219_dev_s *dev, struct ina219_field_data_s *data);
int8_t ina219_wait_end_conversion(const struct ina219_dev_s *dev, const struct ina219_params_s *params, bool *measure_available);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* INA219_H */