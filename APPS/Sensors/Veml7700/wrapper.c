#include "wrapper.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <libopencm3/stm32/i2c.h>

#include "Debug/console.h"
//#include "Drivers/veml7700.h"

#define SLAVE_ADDRESS (0x10)

void vTaskSensorVeml7700(void *pvParameters)
{
  (void) pvParameters;

  console_debug("[VEML7700] Start task\r\n");

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
