#ifndef APPS_DRIVERS_RTC_H
#define APPS_DRIVERS_RTC_H

#include <time.h>

void rtc_calendar_set(struct tm time);
struct tm rtc_calendar_get(void);

#endif /* APPS_DRIVERS_RTC_H */
