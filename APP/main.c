#include "main.h"

#define MAX_BUFFER_UART_TX (255)
#define MAX_BUFFER_UART_RX (255)

static char buffer_uart_tx[MAX_BUFFER_UART_TX+1] = "";
static char buffer_console_dbg[MAX_BUFFER_UART_TX+1] = "";

int main(void)
{
  rcc_clock_setup_pll(rcc_hse_8mhz_3v3);

  SystemCoreClock = rcc_ahb_frequency;
  systick_set_frequency(configTICK_RATE_HZ, SystemCoreClock);

  /* I2C used by sensors */
  vI2C_Setup();

  xTaskCreate(vTaskConsoleDbg,
              "CONSOLE DBG",
              TASK_CONSOLE_DBG_STACK,
              NULL,
              TASK_CONSOLE_DBG_PRIO,
              NULL);

  xTaskCreate(vTaskBME680,
              "BME680",
              TASK_BME680_STACK,
              NULL,
              TASK_BME680_PRIO,
              &xTaskBME680);

  xTaskCreate(vTaskRtcUpdate,
              "RTC Updater",
              TASK_RTC_UPDATER_STACK,
              NULL,
              TASK_RTC_UPDATER_PRIO,
              NULL);

  xTimerSampling = xTimerCreate("Timer Sampling",
                            pdMS_TO_TICKS(2000),
                            pdTRUE,	// Auto-reload
                            (void *) 0,
                            vTimerCallback);

  xQueueUartIsrRx = xQueueCreate(10, sizeof(char) * MAX_BUFFER_UART_RX);

  xQueueUartTx = xQueueCreate(10, sizeof(char) * MAX_BUFFER_UART_TX);

  vTaskStartScheduler();
  taskENABLE_INTERRUPTS();

  while(1);

  return 0;
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
}

void console_dbg(const char *format, ...)
{
  va_list args;
  va_start (args, format);
  vsnprintf(buffer_uart_tx, MAX_BUFFER_UART_TX, format, args);
  xQueueSend(xQueueUartTx, buffer_uart_tx, 100);
  va_end(args);
}

/* Console debug
 * Used UART2 Tx
 */
void vTaskConsoleDbg(void *pvParameters)
{
  (void) pvParameters;

  vConsole_Setup();

  int bufferConsoleDebugLen = 0;

  while(1)
  {
    xQueueReceive(xQueueUartTx, buffer_console_dbg, portMAX_DELAY);
    bufferConsoleDebugLen = strlen(buffer_console_dbg);

    for(int i = 0; i < bufferConsoleDebugLen; i++)
    {
      usart_send_blocking(USART2, (uint16_t)buffer_console_dbg[i]);
    }
  }
}

/* Get and compute BME680 data sensor
 * Temperature, relative Humidity adn Pressure
 * Use I2C1
 */
void vTaskBME680(void *pvParameters)
{
  (void) pvParameters;
  console_dbg("Debug BME680\r\n");

  struct sensor_measure_s data[] =
  {
    {.value = 0, .id = 'T'},
    {.value = 0, .id = 'P'},
    {.value = 0, .id = 'H'}
  };
  struct bme680_s tmp = {.temperature = 0, .pressure = 0, .humidity = 0};

  if(xBme680_Setup() != 0)
  {
    console_dbg("[BME680] Setup fail ...\r\n");
    vTaskDelete(NULL);
  }

  console_dbg("[BME680] Setup finished.\r\n");

  while(1)
  {
    /* Wait Sampling */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xBme680_Get_Data(&tmp) != 0)
    {
      console_dbg("[BME680] Get Data fail ...\r\n");
    }
    else
    {
      data[0].value = tmp.temperature;
      data[1].value = tmp.pressure;
      data[2].value = tmp.humidity;
      console_dbg("[BME680] T: %d, P: %d, H %d\r\n",
          tmp.temperature,
          tmp.pressure,
          tmp.humidity);
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

  console_dbg("Debug RTC Update\r\n");

  while(1)
  {
    /* Read item without removing from the queue */
    xQueuePeek(xQueueUartIsrRx, buffer, portMAX_DELAY);

    /* Test item if it is a time msg*/
    if(strncmp(buffer, "D=", 2) == 0)
    {

      //time = Deserialize_Time(buffer);
      console_dbg("[RTC] Y:%d M:%d D:%d WD:%d H:%d M:%d S:%d\r\n",
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

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
  (void) xTask;
  console_dbg("[WARNING] Stack overflow : %s\r\n", pcTaskName);
}
