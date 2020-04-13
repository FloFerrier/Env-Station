#ifndef UART_H
#define UART_H

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void vConsole_Setup(void);
void vUART_Setup(void);
void _putchar(char character);
void HC05_putchar(char character);

#endif /* UART_H */
