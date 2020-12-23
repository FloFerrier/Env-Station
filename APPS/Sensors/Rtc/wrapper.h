#ifndef RTC_WRAPPER_H
#define RTC_WRAPPER_H

#include <time.h>

#include "Drivers/rtc.h"

time_t rtc_epoch_get(void);
void rtc_epoch_set(const time_t epoch);

#endif /* RTC_WRAPPER_H */
