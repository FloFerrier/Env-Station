#include "uart.h"

void vUART_Setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO10);
  gpio_set_af(GPIOC, GPIO_AF7, GPIO10);

  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO11);
  gpio_set_af(GPIOC, GPIO_AF7, GPIO11);

  rcc_periph_clock_enable(RCC_USART3);
  nvic_disable_irq(NVIC_USART3_IRQ);
  usart_disable_rx_interrupt(USART3);
  usart_set_baudrate(USART3, 38400);
  usart_set_databits(USART3, 8);
  usart_set_stopbits(USART3, USART_STOPBITS_1);
  usart_set_mode(USART3, USART_MODE_TX_RX);
  usart_set_parity(USART3, USART_PARITY_NONE);
  usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
  nvic_set_priority(NVIC_USART3_IRQ, 255);
  usart_enable_rx_interrupt(USART3);
  usart_enable(USART3);
  nvic_enable_irq(NVIC_USART3_IRQ);
}

void vConsole_Setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO2);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

  rcc_periph_clock_enable(RCC_USART2);
  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_enable(USART2);
}

void _putchar(char character)
{
  usart_send_blocking(USART2, (uint16_t)character);
}

void HC05_putchar(char character)
{
  usart_send_blocking(USART3, (uint16_t)character);
}
