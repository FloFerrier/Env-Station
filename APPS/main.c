#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Debug/console.h"
#include "Sensors/Bme680/wrapper.h"
#include "Sensors/Lps33w/wrapper.h"
#include "Sensors/Rtc/wrapper.h"
#include "Comm/Rn4871/rn4871.h"
#include "Comm/Ihm/leds.h"

#include "Ihm/Eink/wrapper.h"

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

extern QueueHandle_t xQueueConsoleDebug;
extern QueueHandle_t xQueueCommUartTx;
extern QueueHandle_t xQueueCommUartRx;

extern EventGroupHandle_t xEventsCommRn4871;

extern void vTaskConsoleDebug(void *pvParameters);
extern void vTaskSensorBme680(void *pvParameters);
extern void vTaskSensorLps33w(void *pvParameters);
extern void vTaskCommRn4871(void *pvParameters);
extern void vTaskIhmEink(void *pvParameters);

extern void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);

void uart2_setup(void);
void uart3_setup(void);
void i2c1_setup(void);
void rtc_setup(void);
void leds_setup(void);
void eink_gpio_setup(void);
void spi2_setup(void);

int main(void)
{
  rcc_clock_setup_pll(rcc_hse_8mhz_3v3);

  SystemCoreClock = rcc_ahb_frequency;
  systick_set_frequency(configTICK_RATE_HZ, SystemCoreClock);

  /* HAL initialization */
  uart2_setup();
  uart3_setup();
  i2c1_setup();
  rtc_setup();
  leds_setup();
  eink_gpio_setup();
  spi2_setup();

  if(xTaskCreate(vTaskConsoleDebug, "CONSOLE DEBUG", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS)
  {
    uart2_send("[KERNEL] Error to create console debug task...\r\n");
  }

  /*if(xTaskCreate(vTaskSensorBme680, "SENSOR BME680", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS)
  {
    uart2_send("[KERNEL] Error to create bme680 task...\r\n");
  }*/

  /*if(xTaskCreate(vTaskSensorLps33w, "SENSOR LPS33W", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS)
  {
    uart2_send("[KERNEL] Error to create lps33w task...\r\n");
  }*/

  /*if(xTaskCreate(vTaskCommRn4871, "COMM RN4871", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
  {
    uart2_send("[KERNEL] Error to create rn4871 task...\r\n");
  }*/

  if(xTaskCreate(vTaskCommIhm, "COMM IHM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
  {
    uart2_send("[KERNEL] Error to create ihm task...\r\n");
  }

  if(xTaskCreate(vTaskIhmEink, "IHM EINK", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
  {
    uart2_send("[KERNEL] Error to create eink task...\r\n");
  }

  xQueueConsoleDebug = xQueueCreate(10, sizeof(char) * BUFFER_CONSOLE_LEN_MAX);
  if(xQueueConsoleDebug == NULL)
  {
    uart2_send("[KERNEL] Error to create console debug queue...\r\n");
  }

  xQueueCommUartRx = xQueueCreate(10, sizeof(char) * BUFFER_UART_LEN_MAX);
  if(xQueueCommUartRx == NULL)
  {
    uart2_send("[KERNEL] Error to create rn4871 Rx queue...\r\n");
  }

  xQueueCommUartTx = xQueueCreate(10, sizeof(char) * BUFFER_UART_LEN_MAX);
  if(xQueueCommUartTx == NULL)
  {
    uart2_send("[KERNEL] Error to create rn4871 Tx queue...\r\n");
  }

  xEventsCommRn4871 = xEventGroupCreate();
  if(xEventsCommRn4871 == NULL)
  {
    uart2_send("[KERNEL] Error to create rn4871 events group...\r\n");
  }

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

void uart3_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
  gpio_set_af(GPIOC, GPIO_AF7, GPIO10 | GPIO11);

  rcc_periph_clock_enable(RCC_USART3);
  nvic_disable_irq(NVIC_USART3_IRQ);
  usart_disable_rx_interrupt(USART3);
  usart_disable(USART3);
  usart_set_baudrate(USART3, 115200);
  usart_set_databits(USART3, 8);
  usart_set_stopbits(USART3, USART_STOPBITS_1);
  usart_set_mode(USART3, USART_MODE_TX_RX);
  usart_set_parity(USART3, USART_PARITY_NONE);
  usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
  nvic_set_priority(NVIC_USART3_IRQ, 255); /* To do: set IT priority */
  usart_enable_rx_interrupt(USART3);
  usart_enable(USART3);
  nvic_enable_irq(NVIC_USART3_IRQ);
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
  i2c_set_dutycycle(I2C1, I2C_CCR_DUTY_DIV2);
  i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / 1e6);
  i2c_peripheral_enable(I2C1);
}

void rtc_setup(void)
{
  /* Enable Clock for RTC */
  rcc_periph_clock_enable(RCC_PWR);
  rcc_periph_clock_enable(RCC_RTC);

  pwr_disable_backup_domain_write_protect();
  /* Enable LSE for calendar using */
  RCC_BDCR |= RCC_BDCR_LSEON;
  RCC_BDCR |= RCC_BDCR_RTCEN;
  RCC_BDCR |= (1<<8); //RTCSEL at 0b01
  RCC_BDCR &= ~(1<<9); //RTCSEL at 0b01

  while(!(RCC_BDCR & RCC_BDCR_LSERDY));

  pwr_enable_backup_domain_write_protect();
}

void leds_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO0 | GPIO1);
}

void eink_gpio_setup(void)
{
  /* SPI2_MISO : PC_2
   * SPI2_MOSI : PC_3
   * SPI2_SCLK : PB_10
   * CS        : PB_12
   * D/C       : PA_8
   * SRCS      : PA_9
   * RST       : PC_7
   * BUSY      : PB_6
   * ENA       : PA_7
   */
  rcc_periph_clock_enable(RCC_GPIOA | RCC_GPIOB | RCC_GPIOC);

  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO12);
  gpio_set_af(GPIOB, GPIO_AF5, GPIO10 | GPIO12);

  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
  gpio_set_af(GPIOC, GPIO_AF5, GPIO2 | GPIO3);

  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7 | GPIO8 | GPIO9);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7 | GPIO8 | GPIO9);

  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);

  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);

  /* Desactive SRCS */
  gpio_clear(GPIOA, GPIO9);
  /* Desactive RST */
  gpio_set(GPIOC, GPIO7);
  /* Active ENA */
  gpio_set(GPIOA, GPIO7);
}

void spi2_setup(void)
{
  rcc_periph_clock_enable(RCC_SPI2);
  spi_disable(SPI2);
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_2,
    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1 , SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_ss_output(SPI2);
  spi_enable(SPI2);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
  (void) xTask;
  (void) pcTaskName;
  uart2_send("[KERNEL] Stack overflow ...\r\n");
  gpio_set(GPIOC, GPIO1);
}
