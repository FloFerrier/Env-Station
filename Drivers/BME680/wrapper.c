#include "wrapper.h"

static struct bme680_dev sensor;
static uint16_t meas_period;

static void user_delay_ms(uint32_t period)
{
  vTaskDelay(pdMS_TO_TICKS(period));
}

static int8_t user_i2c_read(uint8_t dev_id,
                     uint8_t reg_addr,
                     uint8_t *reg_data,
                     uint16_t len)
{
  i2c_transfer7(I2C1, dev_id, &reg_addr, 1, reg_data, (size_t)len);

  return 0;
}

static int8_t user_i2c_write(uint8_t dev_id,
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

int8_t xBme680_Setup(void)
{
  vI2C_Setup();

  int8_t rslt = BME680_OK;

  sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
  sensor.intf = BME680_I2C_INTF;
  sensor.read = user_i2c_read;
  sensor.write = user_i2c_write;
  sensor.delay_ms = user_delay_ms;
  sensor.amb_temp = 25;

  rslt = bme680_soft_reset(&sensor);
  if(rslt != BME680_OK)
  {
    return -1;
  }

  rslt = bme680_init(&sensor);
  if(rslt != BME680_OK)
  {
    return -1;
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
  rslt = bme680_set_sensor_settings(set_required_settings, &sensor);
  if(rslt != BME680_OK)
  {
    return -1;
  }

  /* Set the power mode */
  rslt = bme680_set_sensor_mode(&sensor);
  if(rslt != BME680_OK)
  {
    return -1;
  }

  bme680_get_profile_dur(&meas_period, &sensor);

  return 0;
}

int8_t xBme680_Get_Data(struct bme680_s *data)
{
  struct bme680_field_data tmp;
  int8_t rslt = BME680_OK;

  /* Trigger the next measurement if you would like to read data out continuously */
  if (sensor.power_mode == BME680_FORCED_MODE)
  {
    rslt = bme680_set_sensor_mode(&sensor);
  }

  if(rslt != BME680_OK)
  {
    return -1;
  }

  user_delay_ms(meas_period); /* Delay till the measurement is ready */

  rslt = bme680_get_sensor_data(&tmp, &sensor);
  if(rslt != BME680_OK)
  {
    return -1;
  }

  data->temperature = tmp.temperature / 100;
  data->pressure = tmp.pressure / 100;
  data->humidity = tmp.humidity / 1000;

  return 0;
}
