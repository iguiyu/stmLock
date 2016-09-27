/*
 *
 */
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "gsm.h"
#include "misc.h"

#include <time.h>

#define YEAR0           	1900	/* the first year */
#define EPOCH_YR		1970	/* EPOCH = Jan 1 1970 00:00:00 */
#define SECS_DAY        	(24L * 60L * 60L)
#define LEAPYEAR(year)  	(!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)  	(LEAPYEAR(year) ? 366 : 365)
#define FIRSTSUNDAY(timp)       (((timp)->tm_yday - (timp)->tm_wday + 420) % 7)
#define FIRSTDAYOF(timp)        (((timp)->tm_wday - (timp)->tm_yday + 420) % 7)

extern RTC_HandleTypeDef hrtc;
extern void Error_Handler(void);

static const int _ytab[2][12] = {
                  { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
                  { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};
static struct tm br_time;

struct tm *gmtime(register const time_t *timer)
{
	register struct tm *timep = &br_time;
	time_t time = *timer;
	register unsigned long dayclock, dayno;
	int year = EPOCH_YR;
 
	dayclock = (unsigned long)time % SECS_DAY;
	dayno = (unsigned long)time / SECS_DAY;

	timep->tm_sec = dayclock % 60;
	timep->tm_min = (dayclock % 3600) / 60;
	timep->tm_hour = dayclock / 3600;
	timep->tm_wday = (dayno + 4) % 7;       /* day 0 was a thursday */
	while (dayno >= YEARSIZE(year)) {
		dayno -= YEARSIZE(year);
		year++;
	}
	timep->tm_year = year - YEAR0;
	timep->tm_yday = dayno;
	timep->tm_mon = 0;
	while (dayno >= _ytab[LEAPYEAR(year)][timep->tm_mon]) {
		dayno -= _ytab[LEAPYEAR(year)][timep->tm_mon];
		timep->tm_mon++;
	}
	timep->tm_mday = dayno + 1;
	timep->tm_isdst = 0; 
	return timep;
}

void set_date_time(uint8_t *tsbuf)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	uint32_t ts;
	struct tm *tm;
	
	ts  = (tsbuf[0] - '0') * 1000000000;
	ts += (tsbuf[1] - '0') * 100000000;
	ts += (tsbuf[2] - '0') * 10000000;
	ts += (tsbuf[3] - '0') * 1000000;
	ts += (tsbuf[4] - '0') * 100000;
	ts += (tsbuf[5] - '0') * 10000;
	ts += (tsbuf[6] - '0') * 1000;
	ts += (tsbuf[7] - '0') * 100;
	ts += (tsbuf[8] - '0') * 10;
	ts += (tsbuf[9] - '0') * 1;
	
	tm = gmtime((time_t *)&ts);
	
	sTime.Hours = tm->tm_hour;
	sTime.Minutes = tm->tm_min;
	sTime.Seconds = tm->tm_sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	sDate.WeekDay = tm->tm_wday;
	sDate.Month = RTC_ByteToBcd2(tm->tm_mon + 1);
	sDate.Date = tm->tm_mday;
	sDate.Year = tm->tm_year - 100;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}
#if 1
static __inline unsigned long to_unix_ts (unsigned int year, unsigned int mon,
					unsigned int day, unsigned int hour,
					unsigned int min, unsigned int sec)
{
	if (0 >= (int) (mon -= 2)) {    /* 1..12 -> 11,12,1..10 */
		mon += 12;      /* Puts Feb last since it has leap day */
		year -= 1;
	}

	return ((((unsigned long) (year/4 - year/100 + year/400 + 367*mon/12 + day) +
		year*365 - 719499
		)*24 + hour /* now have hours */
		)*60 + min /* now have minutes */
		)*60 + sec; /* finally seconds */
}
#endif

void set_unix_ts(uint8_t *buf)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	unsigned long t;
	
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_HOURFORMAT_24);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_HOURFORMAT_24);
	t = to_unix_ts(sDate.Year + 2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);
	buf[9] = (t / 1)% 10 + 0x30;
	buf[8] = (t / 10) % 10 + 0x30;
	buf[7] = (t / 100) % 10 + 0x30;
	buf[6] = (t / 1000) % 10 + 0x30;
	buf[5] = (t / 10000) % 10 + 0x30;
	buf[4] = (t / 100000) % 10 + 0x30;
	buf[3] = (t / 1000000) % 10 + 0x30;
	buf[2] = (t / 10000000) % 10 + 0x30;
	buf[1] = (t / 100000000) % 10 + 0x30;
	buf[0] = (t / 1000000000) % 10 + 0x30;
}

