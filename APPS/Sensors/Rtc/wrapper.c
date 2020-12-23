#include "wrapper.h"

time_t rtc_epoch_get(void)
{
  struct tm time;
  time = rtc_calendar_get();
  return mktime(&time);
}

void rtc_epoch_set(const time_t epoch)
{
  struct tm *time;
  time = gmtime(&epoch);
  rtc_calendar_set(*time);
}
