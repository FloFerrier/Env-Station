#include "leds.h"

#include <libopencm3/stm32/gpio.h>

#define LED_GREEN (GPIO0)
#define LED_RED   (GPIO1)

enum FLAGS_EVENTS {
  FLAG_SYSTEM_PROBLEM = 0b0001,
  FLAG_BLE_CONNECTED  = 0b0010,
  FLAG_BLE_STREAM     = 0b0100,
  FLAG_RTC_UPDATED    = 0b1000,
};

static uint8_t _current_status = 0b0000;

void ihm_ble_connected(bool status)
{
  if(status)
  {
    _current_status |= FLAG_BLE_CONNECTED;
  }
  else
  {
    _current_status &= ~FLAG_BLE_CONNECTED;
  }
}

void ihm_ble_stream(bool status)
{
  if(status)
  {
    _current_status |= FLAG_BLE_STREAM;
  }
  else
  {
    _current_status &= ~FLAG_BLE_STREAM;
  }
}

void ihm_system_problem(bool status)
{
  if(status)
  {
    _current_status |= FLAG_SYSTEM_PROBLEM;
  }
  else
  {
    _current_status &= ~FLAG_SYSTEM_PROBLEM;
  }
}

void ihm_rtc_updated(bool status)
{
  if(status)
  {
    _current_status |= FLAG_RTC_UPDATED;
  }
  else
  {
    _current_status &= ~FLAG_RTC_UPDATED;
  }
}

void vTaskIhmLeds(void *pvParameters)
{
  (void) pvParameters;

  gpio_clear(GPIOC, LED_GREEN);
  gpio_clear(GPIOC, LED_RED);

  while(1)
  {
    if(_current_status & FLAG_BLE_CONNECTED)
    {
      if(_current_status & FLAG_BLE_STREAM)
      {
        gpio_toggle(GPIOC, LED_GREEN);
      }
      else
      {
        gpio_set(GPIOC, LED_GREEN);
      }
    }
    else
    {
      gpio_clear(GPIOC, LED_GREEN);
    }

    if(_current_status & FLAG_SYSTEM_PROBLEM)
    {
      gpio_set(GPIOC, LED_RED);
    }
    else if(_current_status & FLAG_RTC_UPDATED)
    {
      gpio_clear(GPIOC, LED_RED);
    }
    else
    {
      gpio_toggle(GPIOC, LED_RED);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
