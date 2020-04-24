#include "led.h"

void vLed_Setup(struct led_s led)
{
  enum rcc_periph_clken rcc_port = 0;

  switch(led.port)
  {
    case(GPIOA):
      rcc_port = RCC_GPIOA;
      break;
    case(GPIOB):
      rcc_port = RCC_GPIOB;
      break;
    case(GPIOC):
      rcc_port = RCC_GPIOC;
      break;
    case(GPIOD):
      rcc_port = RCC_GPIOD;
      break;
    case(GPIOH):
      rcc_port = RCC_GPIOH;
      break;
    default:
      break;
  }

  rcc_periph_clock_enable(rcc_port);
  gpio_mode_setup(led.port,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  led.pin);
  gpio_set_output_options(led.port,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_2MHZ,
                          led.pin);
  gpio_clear(led.port, led.pin);
}

void vLed_Action(struct led_s led, enum State state)
{
  switch(state)
  {
    case ON:
      gpio_set(led.port, led.pin);
      break;
    case OFF:
      gpio_clear(led.port, led.pin);
      break;
    case TOGGLE:
      gpio_toggle(led.port, led.pin);
      break;
    default:
      break;
  }
}
