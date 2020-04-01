#include "button.h"

void vPB_Setup(void)
{
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
}
