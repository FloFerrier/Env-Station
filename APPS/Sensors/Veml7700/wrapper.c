#include "wrapper.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <libopencm3/stm32/i2c.h>

#include "Debug/console.h"
#include "Drivers/veml7700.h"

static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint16_t *data)
{
  if(data != NULL)
  {
    uint8_t reg_data[2];
    i2c_transfer7(I2C1, dev_id, &reg_addr, 1, reg_data, 2);
    console_debug("[I2C] READ reg_addr=0x%x, data=0x%x\r\n", reg_addr, *data);
    *data = (reg_data[1] << 8) | reg_data[0];
    return 0;
  }

  return -1;
}

static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint16_t *data)
{
  if(data != NULL)
  {
    uint8_t buffer[3];
    buffer[0] = (uint8_t)(reg_addr);
    buffer[1] = (uint8_t)((*data & 0xFF00) >> 8);
    buffer[2] = (uint8_t)(*data & 0x00FF);

    console_debug("[I2C] WRITE reg_addr=0x%x, data=0x%x\r\n", buffer[0], (buffer[1] << 8) | buffer[2]);
    i2c_transfer7(I2C1, dev_id, buffer, 3, NULL, 0);
    return 0;
  }

  return -1;
}

void vTaskSensorVeml7700(void *pvParameters)
{
  (void) pvParameters;
  console_debug("[VEML7700] Start task\r\n");

  static struct veml7700_dev sensor;
  sensor.read = user_i2c_read;
  sensor.write = user_i2c_write;
  sensor.config_reg.gain = ALS_GAIN_X1;
  sensor.config_reg.integration_time = ALS_IT_25_MS;
  sensor.config_reg.persistance_protect = ALS_PERS_1;
  sensor.config_reg.interrupt_enable = false;
  sensor.config_reg.shutdown = false;
  sensor.high_threshold_reg = 0x0;
  sensor.low_threshold_reg = 0x0;
  sensor.power_saving.psm = PSM_MODE_1;
  sensor.power_saving.psm_enable = false;
  sensor.high_resolution_data = 0x0;
  sensor.white_channel_data = 0x0;
  sensor.interrupt_status.int_th_low = false;
  sensor.interrupt_status.int_th_high = false;
  
  veml7700_set_config(sensor);

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(2000));
    veml7700_data_output(&sensor);

    /*console_debug("[VEML7700] Loop task\r\n");*/
  }
}
