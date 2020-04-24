#ifndef LED_H
#define LED_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

enum State {
  ON,
  OFF,
  TOGGLE
};

struct led_s
{
  uint32_t port;
  uint16_t pin;
};

void vled_setup(struct led_s led);
void vLed_Action(struct led_s led, enum State state);

#endif /* LED_H */
