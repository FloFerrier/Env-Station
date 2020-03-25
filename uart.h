#ifndef UART_H
#define UART_H

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void vUART_Setup(void);
void RN4871_putchar(char c);

#endif /* UART_H */
