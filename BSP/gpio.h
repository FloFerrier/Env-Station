#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define GPIO_HC05_VCC GPIO3
#define GPIO_HC05_KEY GPIO5

void vGPIO_Setup(void);
void HC05_Enable_Power(bool enable);
void HC05_Enable_Command_Mode(bool enable);

#endif /* GPIO_H */
