#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Debug/console.h"
#include "Sensors/Bme680/wrapper.h"
#include "Sensors/Lps33w/wrapper.h"

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

extern QueueHandle_t xQueueConsoleDebug;

extern void vTaskConsoleDebug(void *pvParameters);
extern void vTaskSensorBme680(void *pvParameters);
extern void vTaskSensorLps33w(void *pvParameters);

extern void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);

void uart2_setup(void);
void i2c1_setup(void);

int main(void)
{
  rcc_clock_setup_pll(rcc_hse_8mhz_3v3);

  SystemCoreClock = rcc_ahb_frequency;
  systick_set_frequency(configTICK_RATE_HZ, SystemCoreClock);

  /* HAL initialization */
  uart2_setup();
  i2c1_setup();

  xTaskCreate(vTaskConsoleDebug, "CONSOLE DEBUG", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(vTaskSensorBme680, "SENSOR BME680", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(vTaskSensorLps33w, "SENSOR LPS33W", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);

  xQueueConsoleDebug = xQueueCreate(10, sizeof(char) * BUFFER_CONSOLE_LEN_MAX);

  vTaskStartScheduler();
  taskENABLE_INTERRUPTS();

  while(1);

  return 0;
}

void uart2_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

  rcc_periph_clock_enable(RCC_USART2);
  usart_disable(USART2);
  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_enable(USART2);
}

void i2c1_setup(void)
{
  /* PB8 = SCL (D15) and PB9 = SDA (D14) */
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
  gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);

  rcc_periph_clock_enable(RCC_I2C1);
  i2c_peripheral_disable(I2C1);
  i2c_reset(I2C1);
  i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / 1e6);
  i2c_peripheral_enable(I2C1);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
  (void) xTask;
  (void) pcTaskName;
  uart2_send("[KERNEL] Stack overflow ...\r\n");
}
