#include "adc.h"

void vADC_Setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

  rcc_periph_clock_enable(RCC_ADC1);
  nvic_disable_irq(NVIC_ADC_IRQ);
  nvic_set_priority(NVIC_ADC_IRQ, 255);
	adc_power_off(ADC1);
	adc_enable_eoc_interrupt(ADC1);
  adc_set_single_conversion_mode (ADC1);
  adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
  //adc_set_regular_sequence(ADC1, 1, channel[]);

  nvic_enable_irq(NVIC_ADC_IRQ);
	adc_power_on(ADC1);
}

uint32_t xAdcRawToVolt(uint32_t raw_value)
{
  if((raw_value <= 0) || (raw_value > 4095))
  {
    return 0;
  }
  return (raw_value * ADC_VOLTAGE_REF / ADC_RESOLUTION_12_BIT);
}
