#include "ina219.h"

static int8_t ina219_reset(const struct ina219_dev_s *dev)
{
  if(dev == NULL)
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};

  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_CONFIGURATION, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  tmp = (ptr_data[0] << 8) + ptr_data[1];
  tmp |= INA219_REG_CONFIG_BIT_RST;
  ptr_data[0] = _REG_16BITS_TO_8BITS_MSB(tmp);
  ptr_data[1] = _REG_16BITS_TO_8BITS_LSB(tmp);

  rslt = dev->write(dev->slave_addr, INA219_REG_ADDR_CONFIGURATION, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  return INA219_OK;
}

static int8_t ina219_set_configuration(const struct ina219_dev_s *dev, const struct ina219_params_s *params)
{
  if(dev == NULL)
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;

  uint16_t reg_value = 0;
  reg_value = INA219_REG_CONFIG_OFFSET_MODE((uint16_t)params->sensor_mode)              |
              INA219_REG_CONFIG_OFFSET_BRNG((uint16_t)params->bus_voltage_range)        |
              INA219_REG_CONFIG_OFFSET_BADC((uint16_t)params->bus_voltage_resolution)   |
              INA219_REG_CONFIG_OFFSET_PG((uint16_t)params->shunt_voltage_range)        |
              INA219_REG_CONFIG_OFFSET_SADC((uint16_t)params->shunt_voltage_resolution);

  if(reg_value > INA219_REG_MASK_CONFIG)
  {
    return INA219_SOFT_ERROR;
  }

  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};

  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_CONFIGURATION, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }
  tmp = _8BITS_TO_16BITS_REG(ptr_data[0], ptr_data[1]);

  tmp &= ~(INA219_REG_MASK_CONFIG); /* Set bits at 0 */
  tmp |= reg_value;

  ptr_data[0] = _REG_16BITS_TO_8BITS_MSB(tmp);
  ptr_data[1] = _REG_16BITS_TO_8BITS_LSB(tmp);

  rslt = dev->write(dev->slave_addr, INA219_REG_ADDR_CONFIGURATION, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  return INA219_OK;
}

static int8_t ina219_get_configuration(const struct ina219_dev_s *dev, uint16_t *reg)
{
  if((dev == NULL) || (reg == NULL))
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;
  uint8_t ptr_data[2] = {0, 0};

  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_CONFIGURATION, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }
  *reg = _8BITS_TO_16BITS_REG(ptr_data[0], ptr_data[1]);

  return INA219_OK;
}

static int8_t ina219_set_calibration(const struct ina219_dev_s *dev, const struct ina219_params_s *params)
{
  if(dev == NULL)
  {
    return INA219_SOFT_ERROR;
  }

  /* Calibration is set by user, so verify if this value is correct.
   * Value of calibration is stored on 15 bits : (FS15:FS1).
   * FS0 is a void bit and will always be 0.
   */
  if(params->calibration > 0xFFFE)
  {
    return INA219_SOFT_ERROR;
  }

  uint16_t tmp = params->calibration;

  int8_t rslt;
  tmp = INA219_REG_CONFIG_OFFSET_CALIB(tmp);

  uint8_t ptr_data[2] = {
    _REG_16BITS_TO_8BITS_MSB(tmp),
    _REG_16BITS_TO_8BITS_LSB(tmp)
  };

  rslt = dev->write(dev->slave_addr, INA219_REG_ADDR_CALIBRATION, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  return INA219_OK;
}

static int8_t ina219_get_shunt_voltage(const struct ina219_dev_s *dev, struct ina219_field_data_s *data)
{
  if((dev == NULL) || (data == NULL))
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;
  int16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_SHUNT_VOLTAGE, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  tmp = _8BITS_TO_16BITS_REG(ptr_data[0], ptr_data[1]);
  data->shunt_voltage = tmp * 10; /* Shunt Voltage LSB (in uV) */
  return INA219_OK;
}

static int8_t ina219_get_bus_voltage(const struct ina219_dev_s *dev, struct ina219_field_data_s *data)
{
  if((dev == NULL) || (data == NULL))
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_BUS_VOLTAGE, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  tmp = _8BITS_TO_16BITS_REG(ptr_data[0], ptr_data[1]);
  tmp = ((tmp & INA219_REG_BUS_VOLTAGE_MASK_BUS) >> 3);
  data->bus_voltage = (uint32_t)tmp * INA219_BUS_VOLTAGE_LSB;

  return INA219_OK;
}

static int8_t ina219_get_power(const struct ina219_dev_s *dev, struct ina219_field_data_s *data)
{
  if((dev == NULL) || (data == NULL))
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_POWER, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  tmp = _8BITS_TO_16BITS_REG(ptr_data[0], ptr_data[1]);
  data->power = tmp * INA219_POWER_LSB;

  return INA219_OK;
}

static int8_t ina219_get_current(const struct ina219_dev_s *dev, struct ina219_field_data_s *data)
{
  if((dev == NULL) || (data == NULL))
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;
  int16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_CURRENT, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }
  tmp = _8BITS_TO_16BITS_REG(ptr_data[0], ptr_data[1]);
  data->current = (int32_t)tmp * INA219_CURRENT_LSB; /* uA */

  return INA219_OK;
}

static uint8_t ina219_check_math_overflow_flag(const struct ina219_dev_s *dev)
{
  if(dev == NULL)
  {
    return INA219_SOFT_ERROR;
  }

  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  dev->read(dev->slave_addr, INA219_REG_ADDR_BUS_VOLTAGE, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];
  tmp &= INA219_REG_BUS_VOLTAGE_BIT_OVF;
  /* OVF bit is set when Power or Current calculations are out of range */
  return tmp;
}

int8_t ina219_init(const struct ina219_dev_s *dev)
{
  if(dev == NULL)
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;
  uint16_t reg = 0;

  rslt = ina219_reset(dev);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  rslt = ina219_get_configuration(dev, &reg);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  if(reg != INA219_REG_VAL_DEFAULT_CONFIG)
  {
    return INA219_HARD_ERROR;
  }

  return INA219_OK;
}

int8_t ina219_config(const struct ina219_dev_s *dev, const struct ina219_params_s *params)
{
  if((dev == NULL) || (params == NULL))
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;

  rslt = ina219_set_calibration(dev, params);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  rslt = ina219_set_configuration(dev, params);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  return INA219_OK;
}

int8_t ina219_start_single_shot(const struct ina219_dev_s *dev, struct ina219_params_s *params)
{
  if(dev == NULL)
  {
    return INA219_SOFT_ERROR;
  }

  /* Verify the parameter for sensor mode and forced to single-shot */
  if((params->sensor_mode != SHUNT_TRIGGERED)     &&
     (params->sensor_mode != BUS_TRIGGERED)       &&
     (params->sensor_mode != SHUNT_BUS_TRIGGERED))
  {
     params->sensor_mode = SHUNT_BUS_TRIGGERED;
  }

  int8_t rslt;
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_CONFIGURATION, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  tmp = _8BITS_TO_16BITS_REG(ptr_data[0], ptr_data[1]);
  tmp &= ~(INA219_REG_CONFIG_MASK_MODE);
  tmp |= INA219_REG_CONFIG_OFFSET_MODE(params->sensor_mode);
  ptr_data[0] = _REG_16BITS_TO_8BITS_MSB(tmp);
  ptr_data[1] = _REG_16BITS_TO_8BITS_LSB(tmp);
  rslt = dev->write(dev->slave_addr, INA219_REG_ADDR_CONFIGURATION, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  return INA219_OK;
}

int8_t ina219_get_sensor_data(const struct ina219_dev_s *dev, struct ina219_field_data_s *data)
{
  if((dev == NULL) || (data == NULL))
  {
    return INA219_SOFT_ERROR;
  }

  int8_t rslt;

  if(ina219_check_math_overflow_flag(dev))
  {
    return INA219_HARD_ERROR;
  }

  rslt = ina219_get_current(dev, data);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }
  rslt = ina219_get_bus_voltage(dev, data);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }
  rslt = ina219_get_shunt_voltage(dev, data);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }
  rslt = ina219_get_power(dev, data);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  return INA219_OK;
}

static uint32_t ina219_get_duration_measure(const enum resolution_e resolution)
{
  uint32_t delay_us = 0;

  switch (resolution)
  {
    case RESOLUTION_9BITS_1SAMPLE:
      delay_us = INA219_ADC_9BITS_1SAMPLE_DUR_US;
      break;
    case RESOLUTION_10BITS_1SAMPLE:
      delay_us = INA219_ADC_10BITS_1SAMPLE_DUR_US;
      break;
    case RESOLUTION_11BITS_1SAMPLE:
      delay_us = INA219_ADC_11BITS_1SAMPLE_DUR_US;
      break;
    case RESOLUTION_12BITS_1SAMPLE:
      delay_us = INA219_ADC_12BITS_1SAMPLE_DUR_US;
      break;
    case RESOLUTION_12BITS_2SAMPLES:
      delay_us = INA219_ADC_12BITS_2SAMPLES_DUR_US;
      break;
    case RESOLUTION_12BITS_4SAMPLES:
      delay_us = INA219_ADC_12BITS_4SAMPLES_DUR_US;
      break;
    case RESOLUTION_12BITS_8SAMPLES:
      delay_us = INA219_ADC_12BITS_8SAMPLES_DUR_US;
      break;
    case RESOLUTION_12BITS_16SAMPLES:
      delay_us = INA219_ADC_12BITS_16SAMPLES_DUR_US;
      break;
    case RESOLUTION_12BITS_32SAMPLES:
      delay_us = INA219_ADC_12BITS_32SAMPLES_DUR_US;
      break;
    case RESOLUTION_12BITS_64SAMPLES:
      delay_us = INA219_ADC_12BITS_64SAMPLES_DUR_US;
      break;
    case RESOLUTION_12BITS_128SAMPLES:
      delay_us = INA219_ADC_12BITS_128SAMPLES_DUR_US;
      break;
    default:
      break;
  };

  return delay_us;
}

int8_t ina219_wait_end_conversion(const struct ina219_dev_s *dev, const struct ina219_params_s *params, bool *measure_available)
{
  if((dev == NULL) || (measure_available == NULL))
  {
    return INA219_SOFT_ERROR;
  }

  uint32_t delay = 0;

  if((params->sensor_mode == SHUNT_TRIGGERED) ||
     (params->sensor_mode == SHUNT_BUS_TRIGGERED))
  {
    delay += ina219_get_duration_measure(params->shunt_voltage_resolution);
  }

  if((params->sensor_mode == BUS_TRIGGERED) ||
     (params->sensor_mode == SHUNT_BUS_TRIGGERED))
  {
    delay += ina219_get_duration_measure(params->bus_voltage_resolution);
  }

  dev->delay_us(delay);

  *measure_available = false;
  int8_t rslt;
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  rslt = dev->read(dev->slave_addr, INA219_REG_ADDR_BUS_VOLTAGE, ptr_data, 2);
  if(rslt != INA219_OK)
  {
    return INA219_I2C_ERROR;
  }

  tmp = _8BITS_TO_16BITS_REG(ptr_data[0], ptr_data[1]);
  tmp &= INA219_REG_BUS_VOLTAGE_BIT_CNVR;
  if(tmp != 0)
  {
    *measure_available = true;
  }

  return INA219_OK;
}