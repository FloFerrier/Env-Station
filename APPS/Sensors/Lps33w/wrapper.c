#include "wrapper.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <libopencm3/stm32/i2c.h>

#include "Debug/console.h"
#include "Drivers/lps33w_reg.h"
#include "Sensors/Rtc/wrapper.h"
#include "Comm/Rn4871/rn4871.h"

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  (void) handle;
  if(bufp != NULL)
  {
    uint8_t buffer[len + 1];

    memcpy(buffer, &reg, 1);
    memcpy(&(buffer[1]), bufp, len);

    i2c_transfer7(I2C1, (uint8_t)0x5D, buffer, (size_t)(len + 1), NULL, 0);
    return 0;
  }

  return -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  (void) handle;
  if(bufp != NULL)
  {
    i2c_transfer7(I2C1, (uint8_t)0x5D, &reg, 1, bufp, (size_t)len);
    return 0;
  }

  return -1;
}

void vTaskSensorLps33w(void *pvParameters)
{
  (void) pvParameters;

  console_debug("[LPS33W] Start Task\r\n");

  static stmdev_ctx_t sensor;
  sensor.write_reg = platform_write;
  sensor.read_reg = platform_read;

  static uint8_t tmp = 0x00;

  /* Check device ID */
  lps33w_device_id_get(&sensor, &tmp);
  if (tmp != LPS33W_ID)
  {
    console_debug("[LPS33W] Check ID: fail...\r\n");
  }

  /* Restore default configuration */
  lps33w_reset_set(&sensor, PROPERTY_ENABLE);
  vTaskDelay(pdMS_TO_TICKS(10));
  lps33w_reset_get(&sensor, &tmp);
  if (tmp != 1)
  {
    console_debug("[LPS33W] Software reset: fail...\r\n");
  }

  /* Enable Block Data Update */
  lps33w_block_data_update_set(&sensor, PROPERTY_ENABLE);
  /* Set Output Data Rate to 0 */
  lps33w_data_rate_set(&sensor, LPS33W_POWER_DOWN);

  static uint8_t reg = 0x00;
  static uint32_t data_raw_pressure = 0x00000000;
  static int16_t data_raw_temperature = 0x0000;
  static uint16_t pressure_hPa = 0;
  static int8_t temperature_degC = 0;

  struct ble_msg_params_s msg_params;

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(30000)); // in ms
    /* Trigger one shot data acquisition */
    lps33w_one_shoot_trigger_set(&sensor, PROPERTY_ENABLE);

    /* Wait data ready */
    lps33w_press_data_ready_get(&sensor, &reg);
    if(reg)
    {
      lps33w_pressure_raw_get(&sensor, &data_raw_pressure);
      pressure_hPa = (uint16_t)(data_raw_pressure / 4096);

      lps33w_temp_data_ready_get(&sensor, &reg);
      if(reg)
      {
        lps33w_temperature_raw_get(&sensor, &data_raw_temperature);
        temperature_degC = (int8_t)(data_raw_temperature / 100);
        msg_params.type = MSG_TYPE_LPS33W;
        msg_params.timestamp = (uint32_t)rtc_epoch_get();
        msg_params.temperature = (int8_t)(temperature_degC);
        msg_params.pressure = (uint16_t)(pressure_hPa);
        rn4871_send_data(&msg_params);
        console_debug("[LPS33W] %d seconds %d degrees %d hPa\r\n",
          msg_params.timestamp, msg_params.temperature, msg_params.pressure);
      }
      else
      {
        console_debug("[LPS33W] Get temperature: fail...\r\n");
      }
    }
    else
    {
      console_debug("[LPS33W] Get pressure: fail...\r\n");
    }
  }
}
