#include "wrapper.h"

#include <string.h>
#include <stdlib.h>

#include <libopencm3/stm32/i2c.h>

#include "Debug/console.h"
#include "Drivers/bme680.h"

static void user_delay_ms(uint32_t period)
{
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

void vTaskSensorBme680(void *pvParameters)
{
  (void) pvParameters;

  uart2_send("[BME680] Start Task !\r\n");

  static uint16_t meas_period;

  static struct bme680_dev sensor;
  sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
  sensor.intf = BME680_I2C_INTF;
  sensor.read = user_i2c_read;
  sensor.write = user_i2c_write;
  sensor.delay_ms = user_delay_ms;
  sensor.amb_temp = 25;

  /* Software reset */
  if(bme680_soft_reset(&sensor) == BME680_OK)
  {
    uart2_send("[BME680] Soft reset: ok\r\n");
  }
  else
  {
    uart2_send("[BME680] Soft reset: fail\r\n");
  }

  /* Init sensor */
  if(bme680_init(&sensor) == BME680_OK)
  {
    uart2_send("[BME680] Init sensor: ok\r\n");
  }
  else
  {
    uart2_send("[BME680] Init sensor: fail\r\n");
  }

  /* Configuration */
  uint8_t set_required_settings;

  /* Set the temperature, pressure and humidity settings */
  sensor.tph_sett.os_hum = BME680_OS_2X;
  sensor.tph_sett.os_pres = BME680_OS_4X;
  sensor.tph_sett.os_temp = BME680_OS_8X;
  sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

  /* Set the remaining gas sensor settings and link the heating profile */
  sensor.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
  /* Create a ramp heat waveform in 3 steps */
  sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
  sensor.gas_sett.heatr_dur = 150; /* milliseconds */

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  sensor.power_mode = BME680_FORCED_MODE;

  /* Set the required sensor settings needed */
  set_required_settings = BME680_OST_SEL    |
                          BME680_OSP_SEL    |
                          BME680_OSH_SEL    |
                          BME680_FILTER_SEL;

  /* Set the desired sensor configuration */
  if(bme680_set_sensor_settings(set_required_settings, &sensor) == BME680_OK)
  {
    uart2_send("[BME680] Set desired configuration: ok\r\n");
  }
  else
  {
    uart2_send("[BME680] Set desired configuration: fail\r\n");
  }

  /* Set the power mode */
  if(bme680_set_sensor_mode(&sensor) == BME680_OK)
  {
    uart2_send("[BME680] Set power: ok\r\n");
  }
  else
  {
    uart2_send("[BME680] Set power: fail\r\n");
  }

  bme680_get_profile_dur(&meas_period, &sensor);

  struct bme680_field_data data;

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(2500)); // in ms

    user_delay_ms((uint16_t)meas_period); /* Delay till the measurement is ready */

    if(bme680_get_sensor_data(&data, &sensor) == BME680_OK)
    {
      //data.temperature /= 100;
      //data.pressure /= 100;
      //data.humidity /= 1000;

      uart2_send("[BME680] Get data: fail\r\n");

      /*console_debug("[BME680] (%d) Temp: %d Press: %d Hum: %d\r\n", \
       ++cnt, data.temperature, data.pressure, data.humidity);*/
    }
    else
    {
      uart2_send("[BME680] Get data: fail\r\n");
    }

    /* Trigger the next measurement if you would like to read data out continuously */
    if(sensor.power_mode == BME680_FORCED_MODE)
    {
      if(bme680_set_sensor_mode(&sensor) == BME680_OK)
      {
        uart2_send("[BME680] Set sensor mode: ok\r\n");
      }
      else
      {
        uart2_send("[BME680] Set sensor mode: fail\r\n");
      }
    }
  }
}
