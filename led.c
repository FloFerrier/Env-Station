#include "led.h"

void vLed_Setup(void)
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
}
