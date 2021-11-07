#include "wrapper.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <libopencm3/stm32/i2c.h>

#include "Debug/console.h"
#include "Sensors/Rtc/wrapper.h"
#include "Drivers/ina219.h"
#include "Comm/Rn4871/rn4871.h"

static void user_delay_us(uint32_t period)
{
  if((period/1000) > 0)
    period /= 1000;
  else
    period = 1;
  vTaskDelay(pdMS_TO_TICKS(period));
}

static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  if(reg_data != NULL)
  {
    i2c_transfer7(I2C1, dev_id, &reg_addr, 1, reg_data, (size_t)len);
    return 0;
  }

  return -1;
}

static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  if(reg_data != NULL)
  {
    uint8_t buffer[len + 1];

    memcpy(buffer, &reg_addr, 1);
    memcpy(&(buffer[1]), reg_data, len);

    i2c_transfer7(I2C1, dev_id, buffer, (size_t)(len + 1), NULL, 0);
    return 0;
  }

  return -1;
}

void vTaskSensorIna219(void *pvParameters)
{
  (void) pvParameters;

  console_debug("[INA219] Start Task\r\n");

  struct ina219_dev_s sensor;
  sensor.slave_addr = (uint8_t)0x41;
  sensor.read = user_i2c_read;
  sensor.write = user_i2c_write;
  sensor.delay_us = user_delay_us;

  struct ina219_params_s params;
  params.calibration = 4096;
  params.sensor_mode = SHUNT_BUS_CONTINUOUS;
  params.bus_voltage_range = BUS_VOLTAGE_RANGE_16_V;
  params.bus_voltage_resolution = RESOLUTION_12BITS_1SAMPLE;
  params.shunt_voltage_range = SHUNT_VOLTAGE_RANGE_40_MV;
  params.shunt_voltage_resolution = RESOLUTION_12BITS_1SAMPLE;

  struct ina219_field_data_s data;
  data.current = 0;
  data.shunt_voltage = 0;
  data.bus_voltage = 0;
  data.power = 0;

  bool measure_available = false;

  if(INA219_OK != ina219_init(&sensor))
  {
    console_debug("[INA219] Fail to init ...\r\n");
  }

  if(INA219_OK != ina219_config(&sensor, &params))
  {
    console_debug("[INA219] Fail to config ...\r\n");
  }

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(5000)); // in ms
    if(INA219_OK != ina219_get_sensor_data(&sensor, &data))
    {
      console_debug("[INA219] Fail to get data ...\r\n");
    }
    console_debug("[INA219] Current: %d Shunt voltage: %d Bus voltage: %d Power: %d\r\n",
       data.current, data.shunt_voltage, data.bus_voltage, data.power);
  }
}