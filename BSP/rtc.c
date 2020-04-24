#include "rtc.h"

struct bcd_s
{
  uint8_t tens;
  uint8_t units;
};

static struct bcd_s rtc_dec_to_bcd(uint8_t dec)
{
  struct bcd_s bcd;
  bcd.tens = dec / 10;
  bcd.units = dec - bcd.tens * 10;
  return bcd;
}

static uint8_t rtc_bcd_to_dec(struct bcd_s bcd)
{
  return((uint8_t)(bcd.tens * 10 + bcd.units));
}

static uint32_t rtc_set_hour(uint8_t val)
{
  struct bcd_s bcd;
  bcd = rtc_dec_to_bcd(val);
  uint32_t tmp = 0;
  tmp |= ((bcd.tens & RTC_TR_HT_MASK) << RTC_TR_HT_SHIFT);
  tmp |= ((bcd.units & RTC_TR_HU_MASK) << RTC_TR_HU_SHIFT);
  return tmp;
}

static uint32_t rtc_set_minute(uint8_t val)
{
  struct bcd_s bcd;
  bcd = rtc_dec_to_bcd(val);
  uint32_t tmp = 0;
  tmp |= ((bcd.tens & RTC_TR_MNT_MASK) << RTC_TR_MNT_SHIFT);
  tmp |= ((bcd.units & RTC_TR_MNU_MASK) << RTC_TR_MNU_SHIFT);
  return tmp;
}

static uint32_t rtc_set_second(uint8_t val)
{
  struct bcd_s bcd;
  bcd = rtc_dec_to_bcd(val);
  uint32_t tmp = 0;
  tmp |= ((bcd.tens & RTC_TR_ST_MASK) << RTC_TR_ST_SHIFT);
  tmp |= ((bcd.units & RTC_TR_SU_MASK) << RTC_TR_SU_SHIFT);
  return tmp;
}

/* Date Register contains only 1 byte for storing Year.
 * Year Register = (Year - 1970)
 */
static uint32_t rtc_set_year(uint16_t val)
{
  struct bcd_s bcd;
  bcd = rtc_dec_to_bcd((uint8_t)(val - 1970));
  uint32_t tmp = 0;
  tmp |= ((bcd.tens & RTC_DR_YT_MASK) << RTC_DR_YT_SHIFT);
  tmp |= ((bcd.units & RTC_DR_YU_MASK) << RTC_DR_YU_SHIFT);
  return tmp;
}

static uint32_t rtc_set_week_day(uint8_t val)
{
  uint32_t tmp = 0;
  tmp |= ((val & RTC_DR_WDU_MASK) << RTC_DR_WDU_SHIFT);
  return tmp;
}

static uint32_t rtc_set_month(uint8_t val)
{
  struct bcd_s bcd;
  bcd = rtc_dec_to_bcd(val);
  uint32_t tmp = 0;
  tmp |= ((bcd.tens & RTC_DR_MT_MASK) << RTC_DR_MT_SHIFT);
  tmp |= ((bcd.units & RTC_DR_MU_MASK) << RTC_DR_MU_SHIFT);
  return tmp;
}

static uint32_t rtc_set_day(uint8_t val)
{
  struct bcd_s bcd;
  bcd = rtc_dec_to_bcd(val);
  uint32_t tmp = 0;
  tmp |= ((bcd.tens & RTC_DR_DT_MASK) << RTC_DR_DT_SHIFT);
  tmp |= ((bcd.units & RTC_DR_DU_MASK) << RTC_DR_DU_SHIFT);
  return tmp;
}

static uint8_t rtc_get_hour(void)
{
  uint8_t val;
  struct bcd_s bcd;
  bcd.tens = ((RTC_TR & (RTC_TR_HT_MASK << RTC_TR_HT_SHIFT)) >> 20);
  bcd.units = ((RTC_TR & (RTC_TR_HU_MASK << RTC_TR_HU_SHIFT)) >> 16);
  val = rtc_bcd_to_dec(bcd);
  return val;
}

static uint8_t rtc_get_minute(void)
{
  uint8_t val;
  struct bcd_s bcd;
  bcd.tens = ((RTC_TR & (RTC_TR_MNT_MASK << RTC_TR_MNT_SHIFT)) >> 12);
  bcd.units = ((RTC_TR & (RTC_TR_MNU_MASK << RTC_TR_MNU_SHIFT)) >> 8);
  val = rtc_bcd_to_dec(bcd);
  return val;
}

static uint8_t rtc_get_second(void)
{
  uint8_t val;
  struct bcd_s bcd;
  bcd.tens = ((RTC_TR & (RTC_TR_ST_MASK << RTC_TR_ST_SHIFT)) >> 4);
  bcd.units = ((RTC_TR & (RTC_TR_SU_MASK << RTC_TR_SU_SHIFT)) >> 0);
  val = rtc_bcd_to_dec(bcd);
  return val;
}

/* Date Register contains only 1 byte for storing Year.
 * Year Register = (Year - 1970)
 */
static uint16_t rtc_get_year(void)
{
  uint16_t val;
  struct bcd_s bcd;
  bcd.tens = ((RTC_DR & (RTC_DR_YT_MASK << RTC_DR_YT_SHIFT)) >> 20);
  bcd.units = ((RTC_DR & (RTC_DR_YU_MASK << RTC_DR_YU_SHIFT)) >> 16);
  val = rtc_bcd_to_dec(bcd) + 1970;
  return val;
}

static uint8_t rtc_get_week_day(void)
{
  uint8_t val;
  val = ((RTC_DR & (RTC_DR_WDU_MASK << RTC_DR_WDU_SHIFT)) >> 13);
  return val;
}

static uint8_t rtc_get_month(void)
{
  uint8_t val;
  struct bcd_s bcd;
  bcd.tens = ((RTC_DR & (RTC_DR_MT_MASK << RTC_DR_MT_SHIFT)) >> 12);
  bcd.units = ((RTC_DR & (RTC_DR_MU_MASK << RTC_DR_MU_SHIFT)) >> 8);
  val = rtc_bcd_to_dec(bcd);
  return val;
}

static uint8_t rtc_get_day(void)
{
  uint8_t val;
  struct bcd_s bcd;
  bcd.tens = ((RTC_DR & (RTC_DR_DT_MASK << RTC_DR_DT_SHIFT)) >> 4);
  bcd.units = ((RTC_DR & (RTC_DR_DU_MASK << RTC_DR_DU_SHIFT)) >> 0);
  val = rtc_bcd_to_dec(bcd);
  return val;
}

void rtc_calendar_config(void)
{
  /* Enable Clock for RTC */
  rcc_periph_clock_enable(RCC_PWR);
  rcc_periph_clock_enable(RCC_RTC);

  pwr_disable_backup_domain_write_protect();
  /* Enable LSE for calendar using */
  RCC_BDCR |= RCC_BDCR_LSEON;
  RCC_BDCR |= RCC_BDCR_RTCEN;
  RCC_BDCR |= (1<<8); //RTCSEL at 0b01
  RCC_BDCR &= ~(1<<9); //RTCSEL at 0b01

  while(!(RCC_BDCR & RCC_BDCR_LSERDY));

  pwr_enable_backup_domain_write_protect();
}

void rtc_calendar_set(struct time_s time)
{
  pwr_disable_backup_domain_write_protect();
  rtc_unlock();
  /* Allow to update Calendar value */
  RTC_ISR |= RTC_ISR_INIT;
  while(!(RTC_ISR & RTC_ISR_INITF));

  /* Set Time Register */
  /* Be careful : TR register must be setting in one time */
  uint32_t reg_tr = rtc_set_hour(time.hour)      + \
                    rtc_set_minute(time.minute)  + \
                    rtc_set_second(time.second);
  RTC_TR = reg_tr;
  /* Be careful : DR register must be setting in one time */
  /* Set Date Register */
  uint32_t reg_dr = rtc_set_year(time.year)          + \
                    rtc_set_month(time.month)        + \
                    rtc_set_week_day(time.week_day)  + \
                    rtc_set_day(time.day);
  RTC_DR = reg_dr;

  RTC_CR &= ~RTC_CR_FMT; // 24-hour format

  /* Exit Initialization sequence */
  RTC_ISR &= ~RTC_ISR_INIT;
  rtc_lock();
  pwr_enable_backup_domain_write_protect();

  while(!(RTC_ISR & RTC_ISR_RSF)); // Wait for allowing Read Date and Time register
}

struct time_s rtc_calendar_get(void)
{
  struct time_s tmp;

  while(RTC_CR & RTC_CR_BYPSHAD);
  tmp.hour     = rtc_get_hour();
  tmp.minute   = rtc_get_minute();
  tmp.second   = rtc_get_second();
  tmp.year     = rtc_get_year();
  tmp.month    = rtc_get_month();
  tmp.day      = rtc_get_day();
  tmp.week_day = rtc_get_week_day();

  return tmp;
}
