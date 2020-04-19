#ifndef LED_H
#define LED_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

enum State {
  ON,
  OFF,
  TOGGLE
};

typedef struct
{
  uint32_t port;
  uint16_t pin;
} led_s;

void vLed_Setup(led_s led);
void vLed_Action(led_s led, enum State state);

#endif /* LED_H */
