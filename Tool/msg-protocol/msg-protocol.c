#include "msg-protocol.h"

time_s Deserialize_Time(char *buffer)
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

static void Serialize_Time(time_s time, char *p_time, size_t *size_msg)
{
  char p_tmp[10];
  size_t size = 0;

  memset(p_time, '\0', MAX_BUFFER_UART_RX);

  strncpy(p_time, "D=", 2);

  for(uint8_t i= 1; i<= 7; i++)
  {
    switch(i)
    {
      case(1):
        itoa(time.year, p_tmp, 10);
        strncat(p_tmp, ":", 1);
        break;
      case(2):
        itoa(time.month, p_tmp, 10);
        strncat(p_tmp, ":", 1);
        break;
      case(3):
        itoa(time.day, p_tmp, 10);
        strncat(p_tmp, ":", 1);
        break;
      case(4):
        itoa(time.week_day, p_tmp, 10);
        strncat(p_tmp, ":", 1);
        break;
      case(5):
        itoa(time.hour, p_tmp, 10);
        strncat(p_tmp, ":", 1);
        break;
      case(6):
        itoa(time.minute, p_tmp, 10);
        strncat(p_tmp, ":", 1);
        break;
      case(7):
        itoa(time.second, p_tmp, 10);
        break;
      default:
        break;
    }

    size = strlen(p_tmp);
    strncat(p_time, p_tmp, size);
  }

  *size_msg = strlen(p_time);
}

static void Serialize_Data(char *id, uint32_t data, char *p_data, size_t *size_msg)
{
  char p_tmp[10];
  size_t size = 0;

  memset(p_data, '\0', MAX_BUFFER_UART_RX);
  itoa(data, p_tmp, 10);
  size = strlen(p_tmp);

  strncpy(p_data, id, 1);
  strncat(p_data, "=", 1);
  strncat(p_data, p_tmp, size);

  *size_msg = strlen(p_data);
}

void Serialize_Msg(time_s time, sensor_data_s data, char *p_msg)
{
  static char p_time[MAX_BUFFER_UART_RX];
  static char p_data[MAX_BUFFER_UART_RX];
  static size_t size = 0;

  memset(p_msg, '\0', MAX_BUFFER_UART_RX);
  Serialize_Time(time, p_time, &size);
  strncat(p_msg, p_time, size);
  strncat(p_msg, ",", 1);
  Serialize_Data("T", data.temperature, p_data, &size); // Temperature
  strncat(p_msg, p_data, size);
  strncat(p_msg, ",", 1);
  Serialize_Data("P", data.pressure, p_data, &size); // Pressure
  strncat(p_msg, p_data, size);
  strncat(p_msg, ",", 1);
  Serialize_Data("H", data.humidity, p_data, &size); // Humidity
  strncat(p_msg, p_data, size);
  strncat(p_msg, ",", 1);
  Serialize_Data("L", data.luminosity, p_data, &size); // Luminosity
  strncat(p_msg, p_data, size);
  strncat(p_msg, ",", 1);
  Serialize_Data("G", data.gas, p_data, &size); // Gas
  strncat(p_msg, p_data, size);
  strncat(p_msg, "\n", 2);
}
