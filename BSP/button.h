#ifndef BUTTON_H
#define BUTTON_H

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>

void vPB_Setup(void);

#endif /* BUTTON_H */
