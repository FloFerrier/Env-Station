#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>

int main(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  GPIO5);
  gpio_set_output_options(GPIOA,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_2MHZ,
                          GPIO5);
  gpio_clear(GPIOA, GPIO5);

  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC,
                  GPIO_MODE_INPUT,
                  GPIO_PUPD_NONE,
                  GPIO13);

  rcc_periph_clock_enable(RCC_SYSCFG);
  nvic_disable_irq(NVIC_EXTI15_10_IRQ);
  exti_select_source(EXTI13, GPIOC);
  exti_set_trigger(EXTI13, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI13);
  exti_reset_request(EXTI13);
  nvic_set_priority(NVIC_EXTI15_10_IRQ, 255);
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

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

  while(1);

  return 0;
}

void exti15_10_isr(void)
{
  exti_reset_request(EXTI13);
  gpio_toggle(GPIOA, GPIO5);
  usart_send_blocking(USART2, 'A');
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');
}
