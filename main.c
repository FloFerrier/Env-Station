#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "BSP/button.h"
#include "BSP/led.h"
#include "BSP/uart.h"
#include "BSP/i2c.h"
#include "BSP/adc.h"
#include "BSP/rtc.h"

#include "Driver/GA1A1S202WP/ga1a1s202wp.h"
#include "Driver/BME680/bme680.h"

#include "Debug/printf/printf.h"

#define MAX_BUFFER_UART_RX 255

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

static TaskHandle_t xTask1 = NULL;
static QueueHandle_t xQueue1 = NULL;
static QueueHandle_t xQueue2 = NULL;

static void vTask1(void *pvParameters);
static void vTask2(void *pvParameters);
static void vTask3(void *pvParameters);
static void vTask4(void *pvParameters);

static void user_delay_ms(uint32_t period);
static time_s xExtract_Time(char *buffer);

int main(void)
{
  SystemCoreClock = rcc_ahb_frequency;
  systick_set_frequency(configTICK_RATE_HZ, SystemCoreClock);

  /* Tasks Creation */
  BaseType_t task1_status = pdFALSE;
  BaseType_t task2_status = pdFALSE;
  BaseType_t task3_status = pdFALSE;
  BaseType_t task4_status = pdFALSE;

  task1_status = xTaskCreate(vTask1,
                            "Task1",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-1,
                            &xTask1);

  task2_status = xTaskCreate(vTask2,
                            "Task2",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-4,
                            NULL);

  task3_status = xTaskCreate(vTask3,
                            "Task3",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-3,
                            NULL);

  task4_status = xTaskCreate(vTask4,
                            "Task4",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-2,
                            NULL);

  xQueue1 = xQueueCreate(10, sizeof(uint32_t));
  xQueue2 = xQueueCreate(10, sizeof(char) * MAX_BUFFER_UART_RX);

  if((task1_status == pdPASS) &&
     (task2_status == pdPASS) &&
     (task3_status == pdPASS) &&
     (task4_status == pdPASS) &&
     (xQueue1 != NULL)        &&
     (xQueue2 != NULL))
  {
    vTaskStartScheduler();
    taskENABLE_INTERRUPTS();
  }

  while(1);

  return 0;
}

/* ISR from Pushed-Button */
void exti15_10_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  exti_reset_request(EXTI13);
  vTaskNotifyGiveFromISR(xTask1, &xHigherPriorityTaskWoken);
}

/* ISR for sampling Luminosity measurement */
void adc_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t raw_value = 0;

  if(adc_get_flag(ADC1, ADC_SR_EOC))
  {
    adc_clear_flag(ADC1, ADC_SR_EOC);
    raw_value = adc_read_regular(ADC1);
    xQueueSendFromISR(xQueue1, &raw_value,&xHigherPriorityTaskWoken);
  }
}

/* ISR for buffering received data from UART3 */
void usart3_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static volatile uint16_t c = 0;
  static uint16_t i = 0;
  static char buffer[MAX_BUFFER_UART_RX];

  c = usart_recv(USART3);
  buffer[i++] = (char)c;

  if(c == '\0')
  {
    i = 0;
    xQueueSendFromISR(xQueue2, buffer, &xHigherPriorityTaskWoken);
  }
  else if(i >= (MAX_BUFFER_UART_RX - 1))
  {
    i = 0;
  }
}

/* Debug task
 * Use Pushed-Button and LED
 */
void vTask1(void *pvParameters)
{
  (void) pvParameters;

  vConsole_Setup();
  vLed_Setup();
  vPB_Setup();

  printf("Debug LED\r\n");

  while(1)
  {
    /* Wait Pushed Button */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    gpio_toggle(GPIOA, GPIO5);
  }
}

/* Get and compute BME680 data sensor
 * Temperature, relative Humidity adn Pressure
 * Use I2C1
 */
void vTask2(void *pvParameters)
{
  (void) pvParameters;

  vI2C_Setup();

  printf("Debug BME680\r\n");

  /* Initialization */
  struct bme680_dev bme680_chip;

  bme680_chip.dev_id = BME680_I2C_ADDR_PRIMARY;
  bme680_chip.intf = BME680_I2C_INTF;
  bme680_chip.read = xI2C_Read;
  bme680_chip.write = xI2C_Write;
  bme680_chip.delay_ms = user_delay_ms;
  bme680_chip.amb_temp = 19;

  int8_t rslt = BME680_OK;

  if(rslt != BME680_OK)
  {
    printf("[BME680] Reset fail ...\r\n");
  }

  bme680_soft_reset(&bme680_chip);

  rslt = BME680_OK;
  rslt = bme680_init(&bme680_chip);

  if(rslt != BME680_OK)
  {
    printf("[BME680] Init fail ...\r\n");
  }

  /* Configuration */
  uint8_t set_required_settings;

  /* Set the temperature, pressure and humidity settings */
  bme680_chip.tph_sett.os_hum = BME680_OS_2X;
  bme680_chip.tph_sett.os_pres = BME680_OS_4X;
  bme680_chip.tph_sett.os_temp = BME680_OS_8X;
  bme680_chip.tph_sett.filter = BME680_FILTER_SIZE_3;

  /* Set the remaining gas sensor settings and link the heating profile */
  bme680_chip.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
  /* Create a ramp heat waveform in 3 steps */
  bme680_chip.gas_sett.heatr_temp = 320; /* degree Celsius */
  bme680_chip.gas_sett.heatr_dur = 150; /* milliseconds */

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  bme680_chip.power_mode = BME680_FORCED_MODE;

  /* Set the required sensor settings needed */
  set_required_settings = BME680_OST_SEL    |
                          BME680_OSP_SEL    |
                          BME680_OSH_SEL    |
                          BME680_FILTER_SEL;

  /* Set the desired sensor configuration */
  rslt = bme680_set_sensor_settings(set_required_settings, &bme680_chip);

  if(rslt != BME680_OK)
  {
    printf("[BME680] Settting fail ...\r\n");
  }

  /* Set the power mode */
  rslt = bme680_set_sensor_mode(&bme680_chip);

  if(rslt != BME680_OK)
  {
    printf("[BME680] Power Mode fail ...\r\n");
  }

  uint16_t meas_period;
  bme680_get_profile_dur(&meas_period, &bme680_chip);

  struct bme680_field_data data;

  while(1)
  {
    user_delay_ms(meas_period); /* Delay till the measurement is ready */

    rslt = bme680_get_sensor_data(&data, &bme680_chip);

    if(rslt != BME680_OK)
    {
      printf("[BME680] Get Data fail ...\r\n");
    }
    else
    {
      printf("[BME680] T: %d, P: %d, H %d\r\n",
          data.temperature / 100,
          data.pressure / 100,
          data.humidity / 1000);
    }

    /* Trigger the next measurement if you would like to read data out continuously */
    if (bme680_chip.power_mode == BME680_FORCED_MODE)
    {
        rslt = bme680_set_sensor_mode(&bme680_chip);
    }

    if(rslt != BME680_OK)
    {
      printf("[BME680] Forced mode fail ...\r\n");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/* Compute Luminosity measurement
 * Used Analog 0 Pin
 */
void vTask3(void *pvParameters)
{
  (void) pvParameters;
  vADC_Setup();

  printf("Debug GA1A1S202WP\r\n");

  uint32_t raw_value = 0;
  uint32_t volt_value = 0;
  //double lux_value = 0;

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(2500));
    adc_start_conversion_regular(ADC1);
    xQueueReceive(xQueue1, &raw_value, portMAX_DELAY);
    volt_value = xAdcRawToVolt(raw_value);
    /* BUG : double precision and function xVoltToLux */
    //lux_value = xVoltToLux(volt_value);
    //printf("[LUX] Lux value : %d\r\n", volt_value);
  }
}

/* Update RTC calendar from UART command
 * Used UART3
 */
void vTask4(void *pvParameters)
{
  (void) pvParameters;
  static char buffer[MAX_BUFFER_UART_RX];
  time_s time;

  vUART_Setup();

  printf("Debug RTC\r\n");

  while(1)
  {
    xQueueReceive(xQueue2, buffer, portMAX_DELAY);
    // parse buffer
    time = xExtract_Time(buffer);
    printf("[RTC] Y:%d M:%d D:%d WD:%d H:%d M:%d S:%d\r\n",
      time.year,
      time.month,
      time.day,
      time.week_day,
      time.hour,
      time.minute,
      time.second);
    /* Decode buffer for extracting date and time calendar */
    vRTC_Calendar_Setup(time);
  }
}

void user_delay_ms(uint32_t period)
{
  vTaskDelay(pdMS_TO_TICKS(period));
}

static time_s xExtract_Time(char *buffer)
{
  time_s time;
  uint8_t i = 0;         // Current position in the buffer
  uint8_t b = 0;         // First digit for an element
  uint8_t size_elmt = 0; // Size of an element
  uint8_t i_elmt = 0;    // Position of current element in the buffer
  char elmt[MAX_BUFFER_UART_RX]; // Buffer for storing element
  int tmp = 0;
  // search the end character
  while(buffer[i] != '\0')
  {
    // search the next separator element
    while((buffer[i] != ':') && (buffer[i] != '\0'))
    {
      i++;
    }
    size_elmt = i - b; // Memorize final digits (without separator position)
    strncpy(elmt, &(buffer[b]), size_elmt);
    elmt[size_elmt] = '\0';
    tmp = atoi(elmt);
    i_elmt++;  // Update number of  element

    i++;
    b = i; // Memorize first digit position

    switch(i_elmt)
    {
      case 1:
        time.year = (uint16_t)tmp;
        break;

      case 2:
        time.month = (uint8_t)tmp;
        break;

      case 3:
        time.day = (uint8_t)tmp;
        break;

      case 4:
        time.week_day = (uint8_t)(tmp+1);
        break;

      case 5:
        time.hour = (uint8_t)tmp;
        break;

      case 6:
        time.minute = (uint8_t)tmp;
        break;

      case 7:
        time.second = (uint8_t)tmp;
        break;

      default:
        /* Nothing */
        break;
    }
  } // while on the all buffer

  return time;
}
