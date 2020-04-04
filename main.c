#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "BSP/button.h"
#include "BSP/led.h"
#include "BSP/uart.h"
#include "BSP/i2c.h"
#include "BSP/adc.h"

#include "Driver/GA1A1S202WP/ga1a1s202wp.h"
#include "Driver/BME680/bme680.h"

#include "Debug/printf/printf.h"

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

static TaskHandle_t xTask1 = NULL;
static QueueHandle_t xQueue1 = NULL;

static void vTask1(void *pvParameters);
static void vTask2(void *pvParameters);
static void vTask3(void *pvParameters);

static void user_delay_ms(uint32_t period);

int main(void)
{
  SystemCoreClock = rcc_ahb_frequency;
  systick_set_frequency(configTICK_RATE_HZ, SystemCoreClock);

  /* Tasks Creation */
  BaseType_t task1_status = pdFALSE;
  BaseType_t task2_status = pdFALSE;
  BaseType_t task3_status = pdFALSE;

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
                            configMAX_PRIORITIES-1,
                            NULL);

  task3_status = xTaskCreate(vTask3,
                            "Task3",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-1,
                            NULL);

  xQueue1 = xQueueCreate(10, sizeof(uint32_t));

  if((task1_status == pdPASS) &&
     (task2_status == pdPASS) &&
     (task3_status == pdPASS) &&
     (xQueue1 != NULL))
  {
    vTaskStartScheduler();
    taskENABLE_INTERRUPTS();
  }

  while(1);

  return 0;
}

void exti15_10_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  exti_reset_request(EXTI13);
  vTaskNotifyGiveFromISR(xTask1, &xHigherPriorityTaskWoken);
}

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

void vTask1(void *pvParameters)
{
  vUART_Setup();
  vLed_Setup();
  vPB_Setup();

  while(1)
  {
    /* Wait Pushed Button */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    gpio_toggle(GPIOA, GPIO5);
  }
}

void vTask2(void *pvParameters)
{
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
  printf("Reset : ");

  if(rslt == BME680_OK)
  {
    printf("ok !\r\n");
  }
  else
  {
    printf("fail ...\r\n");
  }
  bme680_soft_reset(&bme680_chip);

  rslt = BME680_OK;
  rslt = bme680_init(&bme680_chip);

  printf("Init : ");

  if(rslt == BME680_OK)
  {
    printf("ok !\r\n");
  }
  else
  {
    printf("fail ...\r\n");
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

  printf("Setting : ");

  if(rslt == BME680_OK)
  {
    printf("ok !\r\n");
  }
  else
  {
    printf("fail ...\r\n");
  }

  /* Set the power mode */
  rslt = bme680_set_sensor_mode(&bme680_chip);

  printf("Power Mode : ");

  if(rslt == BME680_OK)
  {
    printf("ok !\r\n");
  }
  else
  {
    printf("fail ...\r\n");
  }

  uint16_t meas_period;
  bme680_get_profile_dur(&meas_period, &bme680_chip);

  struct bme680_field_data data;

  while(1)
  {
    user_delay_ms(meas_period); /* Delay till the measurement is ready */

    rslt = bme680_get_sensor_data(&data, &bme680_chip);

    printf("Get Data : ");

    if(rslt == BME680_OK)
    {
      printf("ok !\r\n");
    }
    else
    {
      printf("fail ...\r\n");
    }

    printf("T: %d, P: %d, H %d \r\n",
        data.temperature / 100,
        data.pressure / 100,
        data.humidity / 1000);

    /* Trigger the next measurement if you would like to read data out continuously */
    if (bme680_chip.power_mode == BME680_FORCED_MODE)
    {
        rslt = bme680_set_sensor_mode(&bme680_chip);
    }

    printf("Forced Mode : ");

    if(rslt == BME680_OK)
    {
      printf("ok !\r\n");
    }
    else
    {
      printf("fail ...\r\n");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTask3(void *pvParameters)
{
  vADC_Setup();

  printf("Debug GA1A1S202WP\r\n");

  uint32_t raw_value = 0;
  uint32_t volt_value = 0;
  double lux_value = 0;

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(2500));
    adc_start_conversion_regular(ADC1);
    xQueueReceive(xQueue1, &raw_value, portMAX_DELAY);
    volt_value = xAdcRawToVolt(raw_value);
    lux_value = xVoltToLux(volt_value);
    printf("[LUX] Lux value : %.0lf\r\n", lux_value);
  }
}

void user_delay_ms(uint32_t period)
{
  vTaskDelay(pdMS_TO_TICKS(period));
}
