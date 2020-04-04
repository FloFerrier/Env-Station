#ifndef ADC_H
#define ADC_H

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#define ADC_VOLTAGE_REF       3300 // In millivolt
#define ADC_RESOLUTION_12_BIT 4095 // ADC Max Range
#define ADC_RESOLUTION_10_BIT 1023 // ADC Max Range
#define ADC_RESOLUTION_8_BIT  255  // ADC Max Range
#define ADC_RESOLUTION_6_BIT  63   // ADC Max Range

void vADC_Setup(void);
uint32_t xAdcRawToVolt(uint32_t raw_value);

#endif /* ADC_H */
