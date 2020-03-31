#ifndef UART_H
#define UART_H

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void vUART_Setup(void);
int8_t xUART_Send(char* p_str);

#endif /* UART_H */
