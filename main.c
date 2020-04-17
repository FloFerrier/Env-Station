#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "BSP/button.h"
#include "BSP/led.h"
#include "BSP/uart.h"
#include "BSP/gpio.h"
#include "BSP/i2c.h"
#include "BSP/adc.h"
#include "BSP/rtc.h"

#include "Driver/GA1A1S202WP/ga1a1s202wp.h"
#include "Driver/BME680/bme680.h"
#include "Driver/HC05/hc05.h"

#include "Tool/msg-protocol/msg-protocol.h"

#include "Tool/printf/printf.h"

#define QUEUE_ITEM_MAX_TO_SEND 20 // n latest data ready to be send

typedef struct
{
  char id;
  uint32_t value;
} sensor_measure_s;

/* Necessary for FreeRTOS */
uint32_t SystemCoreClock;

static TaskHandle_t xTaskSafety = NULL;
static TaskHandle_t xTaskBME680 = NULL;
static TaskHandle_t xTaskLUX = NULL;

static TimerHandle_t xTimerSampling = NULL;

static QueueHandle_t xQueueAdcSensorLux = NULL;
static QueueHandle_t xQueueUartIsrRx = NULL;
static QueueHandle_t xQueueSensorsToSupervisor = NULL;
static QueueHandle_t xQueueSupervisorToCommunicator = NULL;

static void vTaskSafety(void *pvParameters);
static void vTaskBME680(void *pvParameters);
static void vTaskSensorLux(void *pvParameters);
static void vTaskSupervisor(void *pvParameters);
static void vTaskCommunicator(void *pvParameters);
static void vTaskRtcUpdate(void *pvParameters);

static void vTimerCallback(TimerHandle_t xTimer);

void HC05_Receive(char *p_str);
void HC05_Timer(uint32_t time);
void user_delay_ms(uint32_t period);

int main(void)
{
  SystemCoreClock = rcc_ahb_frequency;
  systick_set_frequency(configTICK_RATE_HZ, SystemCoreClock);

  /* Console Debug */
  vConsole_Setup();

  /* Tasks Creation */
  BaseType_t task1_status = pdFALSE;
  BaseType_t task2_status = pdFALSE;
  BaseType_t task3_status = pdFALSE;
  BaseType_t task4_status = pdFALSE;
  BaseType_t task5_status = pdFALSE;
  BaseType_t task6_status = pdFALSE;

  task1_status = xTaskCreate(vTaskSafety,
                            "Task Safety",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-1,
                            &xTaskSafety);

  task2_status = xTaskCreate(vTaskBME680,
                            "Task BME680",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-5,
                            &xTaskBME680);

  task3_status = xTaskCreate(vTaskSensorLux,
                            "Task Sensor Lux",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-6,
                            &xTaskLUX);

  task4_status = xTaskCreate(vTaskCommunicator,
                            "Task Communicator",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-2,
                            NULL);

  task5_status = xTaskCreate(vTaskSupervisor,
                            "Task Supervisor",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-4,
                            NULL);

  task6_status = xTaskCreate(vTaskRtcUpdate,
                            "Task RTC Update",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES-3,
                            NULL);

  xTimerSampling = xTimerCreate("Timer Sampling",
                            pdMS_TO_TICKS(2000),
                            pdTRUE,	// Auto-reload
                            (void *) 0,
                            vTimerCallback);

  xQueueAdcSensorLux = xQueueCreate(10, sizeof(uint32_t));
  xQueueUartIsrRx = xQueueCreate(10, sizeof(char) * MAX_BUFFER_UART_RX);
  xQueueSensorsToSupervisor = xQueueCreate(10, sizeof(sensor_measure_s));
  xQueueSupervisorToCommunicator = xQueueCreate(QUEUE_ITEM_MAX_TO_SEND, sizeof(sensors_data_s));

  if((task1_status == pdPASS)            &&
     (task2_status == pdPASS)            &&
     (task3_status == pdPASS)            &&
     (task4_status == pdPASS)            &&
     (task5_status == pdPASS)            &&
     (task6_status == pdPASS)            &&
     (xTimerSampling != NULL)            &&
     (xQueueAdcSensorLux != NULL)        &&
     (xQueueUartIsrRx != NULL)           &&
     (xQueueSensorsToSupervisor != NULL) &&
     (xQueueSupervisorToCommunicator != NULL))
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
  vTaskNotifyGiveFromISR(xTaskSafety, &xHigherPriorityTaskWoken);
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
    xQueueSendFromISR(xQueueAdcSensorLux, &raw_value,&xHigherPriorityTaskWoken);
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

  if(c == '\n')
  {
    i = 0;
    xQueueSendFromISR(xQueueUartIsrRx, buffer, &xHigherPriorityTaskWoken);
  }
  else if(i >= (MAX_BUFFER_UART_RX - 1))
  {
    i = 0;
  }
}

/* Daemon which schedule the sampling with software timer
 *
 */
void vTimerCallback(TimerHandle_t xTimer)
{
  (void) xTimer;
  xTaskNotifyGive(xTaskBME680);
  xTaskNotifyGive(xTaskLUX);
}

/* Debug task
 * Use Pushed-Button and LED
 */
void vTaskSafety(void *pvParameters)
{
  (void) pvParameters;
  printf("Debug LED\r\n");

  vLed_Setup();
  vPB_Setup();

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
void vTaskBME680(void *pvParameters)
{
  (void) pvParameters;
  printf("Debug BME680\r\n");

  sensor_measure_s measure_temperature = {.value = 0, .id = 'T'};
  sensor_measure_s measure_pressure = {.value = 0, .id = 'P'};
  sensor_measure_s measure_humidity = {.value = 0, .id = 'H'};

  vI2C_Setup();

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
    printf("[BME680] Setting fail ...\r\n");
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

  printf("[BME680] Setup finished.\r\n");

  while(1)
  {
    /* Wait Sampling */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* Trigger the next measurement if you would like to read data out continuously */
    if (bme680_chip.power_mode == BME680_FORCED_MODE)
    {
        rslt = bme680_set_sensor_mode(&bme680_chip);
    }

    if(rslt != BME680_OK)
    {
      printf("[BME680] Forced mode fail ...\r\n");
    }

    user_delay_ms(meas_period); /* Delay till the measurement is ready */

    rslt = bme680_get_sensor_data(&data, &bme680_chip);

    if(rslt != BME680_OK)
    {
      printf("[BME680] Get Data fail ...\r\n");
      measure_temperature.value = 0;
      measure_pressure.value = 0;
      measure_humidity.value = 0;
    }
    else
    {
      measure_temperature.value = data.temperature / 100;
      measure_pressure.value = data.pressure / 100;
      measure_humidity.value = data.humidity / 1000;

      printf("[BME680] T: %d, P: %d, H %d\r\n",
          measure_temperature.value,
          measure_pressure.value,
          measure_humidity.value);
    }

    xQueueSend(xQueueSensorsToSupervisor, &measure_temperature, pdMS_TO_TICKS(1));
    xQueueSend(xQueueSensorsToSupervisor, &measure_pressure, pdMS_TO_TICKS(1));
    xQueueSend(xQueueSensorsToSupervisor, &measure_humidity, pdMS_TO_TICKS(1));
  }
}

/* Compute Luminosity measurement
 * Used Analog 0 Pin
 */
void vTaskSensorLux(void *pvParameters)
{
  (void) pvParameters;
  printf("Debug GA1A1S202WP\r\n");

  sensor_measure_s measure_luminosity = {.value = 0, .id = 'L'};
  vADC_Setup();

  uint32_t raw_value = 0;
  uint32_t volt_value = 0;
  //double lux_value = 0;

  printf("[GA1A1S202WP] Setup finished.\r\n");

  while(1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    adc_start_conversion_regular(ADC1);
    xQueueReceive(xQueueAdcSensorLux, &raw_value, portMAX_DELAY);
    volt_value = xAdcRawToVolt(raw_value);
    /* BUG : double precision and function xVoltToLux */
    //lux_value = xVoltToLux(volt_value);
    printf("[LUX] Lux value : %d\r\n", volt_value);

    measure_luminosity.value = volt_value;
    xQueueSend(xQueueSensorsToSupervisor, &measure_luminosity, pdMS_TO_TICKS(1));
  }
}

void vTaskSupervisor(void *pvParameters)
{
  (void) pvParameters;
  UBaseType_t uxFreeItems = 0;
  sensor_measure_s tmp;
  uint32_t counter = 0;
  time_s time = {.year = 0, .month = 0, .day = 0, .week_day = 0,
    .hour = 0, .minute = 0, .second = 0};
  sensors_data_s data = {.horodatage = time, .temperature = 0,
    .pressure = 0, .humidity = 0, .luminosity = 0, .gas = 0};

  printf("Debug Supervisor\r\n");

  while(1)
  {
    /* Wait ALL data treatment before sending to Communicator task */
    while(counter != 4)
    {
      xQueueReceive(xQueueSensorsToSupervisor, &tmp, portMAX_DELAY);
      counter++;
      switch(tmp.id)
      {
        case('T'):
          data.temperature = tmp.value;
          break;
        case('P'):
          data.pressure = tmp.value;
          break;
        case('H'):
          data.humidity = tmp.value;
          break;
        case('L'):
          data.luminosity = tmp.value;
          break;
        case('G'):
          data.gas = tmp.value;
          break;
        default:
          break;
      }
    }
    counter = 0;
    data.horodatage = rtc_calendar_get();

    /* Store n lastest data sensors */
    uxFreeItems = uxQueueSpacesAvailable(xQueueSupervisorToCommunicator);
    if(uxFreeItems == 0)
    {
      xQueueReset(xQueueSupervisorToCommunicator);
    }
    xQueueSend(xQueueSupervisorToCommunicator, &data, pdMS_TO_TICKS(1));
  }
}

/* Bluetooth communication
 * Used UART3 and HC05 bluetooth module
 */
void vTaskCommunicator(void *pvParameters)
{
  (void) pvParameters;
  printf("Debug Communicator\r\n");

  static char buffer[MAX_BUFFER_UART_RX];
  static char p_msg[MAX_BUFFER_UART_RX];
  time_s time = {.year = 0, .month = 0, .day = 0, .week_day = 0,
    .hour = 0, .minute = 0, .second = 0};
  sensors_data_s sensors_data = {.horodatage = time, .temperature = 0,
    .pressure = 0, .humidity = 0, .luminosity = 0, .gas = 0};

  int8_t rslt = HC05_OK;

  vGPIO_Setup();
  vUART_Setup();

  rslt = HC05_Setup("WS-V1", "0501");

  if(rslt != HC05_OK)
  {
    printf("[HC05] Setup fail ...\r\n");
  }

  printf("[HC05] Data Mode Start !\r\n");

  /* Eliminating garbage data on UART */
  HC05_Receive(buffer);

  /* Only when RTC is set, you start sampling */
  if(xTimerStart(xTimerSampling, 0) != pdPASS)
  {
    printf("[COM] Error to start sampling timer...\r\n");
  }

  while(1)
  {
    xQueueReceive(xQueueSupervisorToCommunicator,
                  &sensors_data,
                  portMAX_DELAY);

    Serialize_Msg(sensors_data, p_msg);
    HC05_Send_Data(p_msg);
    printf("[HC05] %s\r\n", p_msg);
    if(HC05_Cmp_Response("ACK\r\n"))
    {
      printf("[HC05] ACK !\r\n");
    }
    else
    {
      printf("[HC05] No ACK ...\r\n");
    }
  }
}

/* Update RTC calendar
 * Used UART3 and HC05 bluetooth module
 */
void vTaskRtcUpdate(void *pvParameters)
{
  (void) pvParameters;

  static char buffer[MAX_BUFFER_UART_RX];
  time_s time = {.year = 0, .month = 0, .day = 0, .week_day = 0,
    .hour = 0, .minute = 0, .second = 0};

  rtc_calendar_config();

  printf("Debug RTC Update\r\n");

  while(1)
  {
    /* Read item without removing from the queue */
    xQueuePeek(xQueueUartIsrRx, buffer, portMAX_DELAY);

    /* Test item if it is a time msg*/
    if(strncmp(buffer, "D=", 2) == 0)
    {
      /* Remove item from the queue if it is a time msg */
      HC05_Receive(buffer);

      time = Deserialize_Time(buffer);
      printf("[RTC] Y:%d M:%d D:%d WD:%d H:%d M:%d S:%d\r\n",
        time.year,
        time.month,
        time.day,
        time.week_day,
        time.hour,
        time.minute,
        time.second);
      /* Decode buffer for extracting date and time calendar */
      rtc_calendar_set(time);
    }
  }
}

void HC05_Receive(char *p_str)
{
  xQueueReceive(xQueueUartIsrRx, p_str, portMAX_DELAY);
}

void HC05_Timer(uint32_t time)
{
  vTaskDelay(pdMS_TO_TICKS(time));
}

void user_delay_ms(uint32_t period)
{
  vTaskDelay(pdMS_TO_TICKS(period));
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
  (void) xTask;
  printf("[WARNING] Stack overflow : %s\r\n", pcTaskName);
}
