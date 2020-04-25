#include "main.h"

int main(void)
{
  rcc_clock_setup_pll(rcc_hse_8mhz_3v3);

  SystemCoreClock = rcc_ahb_frequency;
  systick_set_frequency(configTICK_RATE_HZ, SystemCoreClock);

  /* Console Debug */
  vConsole_Setup();
  /* I2C used by sensors */
  vI2C_Setup();

  xTaskCreate(vTaskBME680,
              "BME680",
              TASK_BME680_STACK,
              NULL,
              TASK_BME680_PRIO,
              &xTaskBME680);

  xTaskCreate(vTaskSensorLux,
              "GA1A1S202WP",
              TASK_GA1A1S202WP_STACK,
              NULL,
              TASK_GA1A1S202WP_PRIO,
              &xTaskLUX);

  xTaskCreate(vTaskCommunicator,
              "Communicator",
              TASK_COMMUNICATOR_STACK,
              NULL,
              TASK_COMMUNICATOR_PRIO,
              NULL);

  xTaskCreate(vTaskSupervisor,
              "Supervisor",
              TASK_SUPERVISOR_STACK,
              NULL,
              TASK_SUPERVISOR_PRIO,
              NULL);

  xTaskCreate(vTaskRtcUpdate,
              "RTC Updater",
              TASK_RTC_UPDATER_STACK,
              NULL,
              TASK_RTC_UPDATER_PRIO,
              NULL);

  xTaskCreate(vTaskIna219,
              "INA219",
              TASK_INA219_STACK,
              NULL,
              TASK_INA219_PRIO,
              &xTaskINA219);

  xTimerSampling = xTimerCreate("Timer Sampling",
                            pdMS_TO_TICKS(2000),
                            pdTRUE,	// Auto-reload
                            (void *) 0,
                            vTimerCallback);

  xQueueAdcSensorLux = xQueueCreate(10, sizeof(uint32_t));
  xQueueUartIsrRx = xQueueCreate(10, sizeof(char) * MAX_BUFFER_UART_RX);
  xQueueSensorsToSupervisor = xQueueCreate(10, sizeof(struct sensor_measure_s));
  xQueueSupervisorToCommunicator = xQueueCreate(QUEUE_ITEM_MAX_TO_SEND, sizeof(struct sensors_data_s));

  vTaskStartScheduler();
  taskENABLE_INTERRUPTS();

  while(1);

  return 0;
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
  xTaskNotifyGive(xTaskINA219);
}

/* Get and compute BME680 data sensor
 * Temperature, relative Humidity adn Pressure
 * Use I2C1
 */
void vTaskBME680(void *pvParameters)
{
  (void) pvParameters;
  printf("Debug BME680\r\n");

  struct sensor_measure_s data[3] =
  {
    {.value = 0, .id = 'T'},
    {.value = 0, .id = 'P'},
    {.value = 0, .id = 'H'}
  };
  struct bme680_s tmp = {.temperature = 0, .pressure = 0, .humidity = 0};

  if(xBme680_Setup() != 0)
  {
    printf("[BME680] Setup fail ...\r\n");
    vTaskDelete(NULL);
  }

  printf("[BME680] Setup finished.\r\n");

  while(1)
  {
    /* Wait Sampling */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xBme680_Get_Data(&tmp) != 0)
    {
      printf("[BME680] Get Data fail ...\r\n");
    }
    else
    {
      data[0].value = tmp.temperature;
      data[1].value = tmp.pressure;
      data[2].value = tmp.humidity;
      printf("[BME680] T: %d, P: %d, H %d\r\n",
          tmp.temperature,
          tmp.pressure,
          tmp.humidity);
    }

    xQueueSend(xQueueSensorsToSupervisor, &data[0], pdMS_TO_TICKS(1));
    xQueueSend(xQueueSensorsToSupervisor, &data[1], pdMS_TO_TICKS(1));
    xQueueSend(xQueueSensorsToSupervisor, &data[2], pdMS_TO_TICKS(1));
  }
}

/* Compute Luminosity measurement
 * Used Analog 0 Pin
 */
void vTaskSensorLux(void *pvParameters)
{
  (void) pvParameters;
  printf("Debug GA1A1S202WP\r\n");

  struct sensor_measure_s measure_luminosity = {.value = 0, .id = 'L'};
  vADC_Setup();

  uint32_t raw_value = 0;
  uint32_t volt_value = 0;
  double lux_value = 0.0;

  printf("[GA1A1S202WP] Setup finished.\r\n");

  while(1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    adc_start_conversion_regular(ADC1);
    xQueueReceive(xQueueAdcSensorLux, &raw_value, portMAX_DELAY);
    volt_value = xAdcRawToVolt(raw_value);
    lux_value = xVoltToLux(volt_value);
    printf("[LUX] L: %d\r\n", (int)lux_value);

    measure.value = (int)lux_value;
    xQueueSend(xQueueSensorsToSupervisor, &measure, pdMS_TO_TICKS(1));
  }
}

/* Get and compute INA219 data sensor
 * Voltage and current for alimenting STM32 board
 * Use I2C1
 */
void vTaskIna219(void *pvParameters)
{
  (void) pvParameters;

  printf("Debug INA219\r\n");

  int8_t rslt = 0;
  struct ina219_s data = {
    .voltage = 0,
    .current = 0,
  };

  struct sensor_measure_s measure[] =
  {
    {.id = 'V', .value = 0},
    {.id = 'C', .value = 0},
  };

  ina219_setup();

  printf("[INA219] Setup finished.\r\n");

  while(1)
  {
    /* Wait Sampling */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    rslt = ina219_get_data(&data);
    if(rslt == 0)
    {
      measure[0].value = data.voltage;
      measure[1].value = data.current;

      printf("[INA219] V: %d, C: %d\r\n",
        data.voltage, data.current);
    }
    else
    {
      printf("[INA219] Get data fail ...\r\n");
    }

    xQueueSend(xQueueSensorsToSupervisor, &measure[0], pdMS_TO_TICKS(1));
    xQueueSend(xQueueSensorsToSupervisor, &measure[1], pdMS_TO_TICKS(1));
  }
}

void vTaskSupervisor(void *pvParameters)
{
  (void) pvParameters;
  UBaseType_t uxFreeItems = 0;
  struct sensor_measure_s tmp;
  uint32_t counter = 0;
  struct time_s time =
  {
    .year = 0,
    .month = 0,
    .day = 0,
    .week_day = 0,
    .hour = 0,
    .minute = 0,
    .second = 0
  };

  struct sensors_data_s data =
  {
    .horodatage = time,
    .voltage = 0,
    .current = 0,
    .temperature = 0,
    .pressure = 0,
    .humidity = 0,
    .luminosity = 0,
  };

  printf("Debug Supervisor\r\n");

  while(1)
  {
    /* Wait ALL data treatment before sending to Communicator task */
    while(counter != 6)
    {
      xQueueReceive(xQueueSensorsToSupervisor, &tmp, portMAX_DELAY);
      counter++;
      switch(tmp.id)
      {
        case('V'):
          data.voltage = tmp.value;
          break;
        case('C'):
          data.current = tmp.value;
          break;
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
  struct time_s time = {.year = 0, .month = 0, .day = 0, .week_day = 0,
    .hour = 0, .minute = 0, .second = 0};
  struct sensors_data_s sensors_data = {.horodatage = time, .temperature = 0,
    .pressure = 0, .humidity = 0, .luminosity = 0, .gas = 0};
  struct led_s led_green = {.port = GPIOC, .pin = GPIO7};
  struct led_s led_red = {.port = GPIOA, .pin = GPIO9};

  int8_t rslt = HC05_OK;

  vLed_Setup(led_green);
  vLed_Setup(led_red);
  vGPIO_Setup();
  vUART_Setup();

  rslt = HC05_Setup("WS-V1", "0501");

  if(rslt != HC05_OK)
  {
    printf("[HC05] Setup fail ...\r\n");
    vLed_Action(led_red, TOGGLE);
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
      vLed_Action(led_green, TOGGLE);
    }
    else
    {
      printf("[HC05] No ACK ...\r\n");
      vLed_Action(led_red, TOGGLE);
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
  struct time_s time = {.year = 0, .month = 0, .day = 0, .week_day = 0,
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

void ina219_wait_conversion()
{
  while(ina219_check_conversion() != true)
  {
    taskYIELD();
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

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
  (void) xTask;
  printf("[WARNING] Stack overflow : %s\r\n", pcTaskName);
}
