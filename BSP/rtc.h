#ifndef RTC_H
#define RTC_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rtc.h>

#define MONDAY    0b001
#define TUESDAY   0b010
#define WEDNESDAY 0b011
#define THURSDAY  0b100
#define FRIDAY    0b101
#define SATURDAY  0b110
#define SUNDAY    0b111

typedef struct
{
  uint8_t  hour;     /* hour : 0-23 */
  uint8_t  minute;   /* minute : 0-59 */
  uint8_t  second;   /* second : 0-59 */
  uint16_t year;     /* year (no limit of definition) */
  uint8_t  month;    /* month : January = 1 ... December = 12 */
  uint8_t  day;      /* day of the month : 1-31 */
  uint8_t  week_day; /* day of the week : Monday = 1 ... Sunday = 7 */
} time_s;

uint8_t vRTC_Calendar_Setup(time_s time);
void vRTC_Calendar_Read(time_s *time);

#endif /* RTC_H */
