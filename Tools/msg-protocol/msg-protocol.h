#ifndef MSG_PROTOCOL_H
#define MSG_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "BSP/rtc.h" // Need for having struct time_s

#define MAX_BUFFER_UART_RX 255

struct sensors_data_s
{
  struct time_s   horodatage;
  uint32_t voltage;
  uint32_t current;
  uint32_t temperature;
  uint32_t humidity;
  uint32_t pressure;
  uint32_t luminosity;
};

struct time_s Deserialize_Time(char *buffer);
void Serialize_Msg(struct sensors_data_s data,
                   char *p_msg);

#endif /* MSG_PROTOCOL_H */
