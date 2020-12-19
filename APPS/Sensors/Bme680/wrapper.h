#ifndef BME680_WRAPPER_H
#define BME680_WRAPPER_H

#include "FreeRTOS.h"
#include "task.h"

void vTaskSensorBme680(void *pvParameters);

#endif /* BME680_WRAPPER_H */
