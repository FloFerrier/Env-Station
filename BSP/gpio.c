#include "gpio.h"

void vGPIO_Setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_mode_setup(GPIOB,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  GPIO_HC05_VCC | GPIO_HC05_KEY);
  gpio_set_output_options(GPIOB,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_2MHZ,
                          GPIO_HC05_VCC | GPIO_HC05_KEY);

  gpio_clear(GPIOB, GPIO_HC05_VCC);
  gpio_clear(GPIOB, GPIO_HC05_KEY);
}

void HC05_Enable_Power(bool enable)
{
  if(enable)
  {
    gpio_set(GPIOB, GPIO_HC05_VCC);
  }
  else
  {
    gpio_clear(GPIOB, GPIO_HC05_VCC);
  }
}

void HC05_Enable_Command_Mode(bool enable)
{
  if(enable)
  {
    gpio_set(GPIOB, GPIO_HC05_KEY);
  }
  else
  {
    gpio_clear(GPIOB, GPIO_HC05_KEY);
  }
}
