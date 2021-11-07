#ifndef INA219_WRAPPER_H
#define INA219_WRAPPER_H

#include "FreeRTOS.h"
#include "task.h"

void vTaskSensorIna219(void *pvParameters);

#endif /* INA219_WRAPPER_H */