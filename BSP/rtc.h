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

struct time_s
{
  uint8_t  hour;     /* hour : 0-23 */
  uint8_t  minute;   /* minute : 0-59 */
  uint8_t  second;   /* second : 0-59 */
  uint16_t year;     /* year (no limit of definition) */
  uint8_t  month;    /* month : January = 1 ... December = 12 */
  uint8_t  day;      /* day of the month : 1-31 */
  uint8_t  week_day; /* day of the week : Monday = 1 ... Sunday = 7 */
};

void rtc_calendar_config(void);
void rtc_calendar_set(struct time_s time);
struct time_s rtc_calendar_get(void);

#endif /* RTC_H */
