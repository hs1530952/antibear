#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/time.h"
#include "drivers/persistent.h"
#include "drivers/time.h"

RTC_HandleTypeDef hrtc;

#define RTC_ASYNCHPREDIV        31
#define RTC_SYNCHPREDIV         1023

static RTC_TIMEZONE_UTC_e timezone = RTC_TIMEZONE_UTC_BASE;

// For the "modulo 4" arithmetic to work, we need a leap base year
#define REFERENCE_YEAR 2000
// Offset (seconds) from the UNIX epoch (1970-01-01) to 2000-01-01
#define EPOCH_2000_OFFSET 946684800

#define MILLIS_PER_SECOND 1000

static const uint16_t days[4][12] =
{
    {   0,  31,     60,     91,     121,    152,    182,    213,    244,    274,    305,    335},
    { 366,  397,    425,    456,    486,    517,    547,    578,    609,    639,    670,    700},
    { 731,  762,    790,    821,    851,    882,    912,    943,    974,    1004,   1035,   1065},
    {1096,  1127,   1155,   1186,   1216,   1247,   1277,   1308,   1339,   1369,   1400,   1430},
};

void rtcInit(void)
{
    /*##-1- Enable peripherals Clocks ##########################################*/
    /* Enable the RTC clock */
    __HAL_RCC_RTC_ENABLE();

    hrtc.Instance = RTC;
    HAL_RTC_WaitForSynchro(&hrtc);

    timezone = persistentObjectRead(PERSISTENT_OBJECT_TIMEZONE);
    if (timezone == 0xFFFFFFFF) {
        /*##-2- RTC parameter configuration ########################################*/
        hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
        hrtc.Init.AsynchPrediv = RTC_ASYNCHPREDIV;
        hrtc.Init.SynchPrediv = RTC_SYNCHPREDIV;
        hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
        HAL_RTC_Init(&hrtc);

        RTC_TimeTypeDef sTime = {0};
        RTC_DateTypeDef sDate = {0};

        sTime.Hours = RTC_DEFAULT_HOUR;
        sTime.Minutes = RTC_DEFAULT_MINUTE;
        sTime.Seconds = RTC_DEFAULT_SECOND;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;

        sDate.WeekDay = RTC_DEFAULT_WEEKDAY;
        sDate.Month = RTC_DEFAULT_MONTH;
        sDate.Date = RTC_DEFAULT_DATE;
        sDate.Year = RTC_DEFAULT_YEAR;

        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        timezone = RTC_DEFAULT_TIMEZONE;
        persistentObjectWrite(PERSISTENT_OBJECT_TIMEZONE, timezone);
    }
}

bool rtcSetTimeZone(RTC_TIMEZONE_UTC_e *tz)
{
    if (HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY) {
        return false;
    }

    timezone = *tz;
    persistentObjectWrite(PERSISTENT_OBJECT_TIMEZONE, timezone);

    return true;
}

bool rtcGetTimeZone(RTC_TIMEZONE_UTC_e *tz)
{
    if (HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY) {
        return false;
    }

    *tz = timezone;

    return true;
}

bool rtcGetDateTime(dateTime_t *dt)
{
    if (HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY) {
        return false;
    }

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    dt->year = sDate.Year + REFERENCE_YEAR;
    dt->month = sDate.Month;
    dt->date = sDate.Date;
    dt->wday = sDate.WeekDay;

    dt->hours = sTime.Hours;
    dt->minutes = sTime.Minutes;
    dt->seconds = sTime.Seconds;
    dt->millis = ((uint32_t)sTime.SecondFraction - (uint32_t)sTime.SubSeconds) * 1000 / ((uint32_t)sTime.SecondFraction + 1);

    return true;
}

bool rtcSetDateTime(dateTime_t *dt)
{
    if (HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY) {
        return false;
    }

    if (dt->year < REFERENCE_YEAR) {
        return false;
    }

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = dt->hours;
    sTime.Minutes = dt->minutes;
    sTime.Seconds = dt->seconds;

    sDate.WeekDay = dt->wday;
    sDate.Month = dt->month;
    sDate.Date = dt->date;
    sDate.Year = dt->year - REFERENCE_YEAR;

    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    return true;
}

static rtcTime_t rtcTimeMake(int32_t secs, uint16_t millis)
{
    return ((rtcTime_t)secs) * MILLIS_PER_SECOND + millis;
}

static int32_t rtcTimeGetSeconds(const rtcTime_t *t)
{
    return *t / MILLIS_PER_SECOND;
}

static uint16_t rtcTimeGetMillis(const rtcTime_t *t)
{
    return *t % MILLIS_PER_SECOND;
}

static rtcTime_t dateTimeToRtcTime(dateTime_t *dt)
{
    unsigned int second = dt->seconds;  // 0-59
    unsigned int minute = dt->minutes;  // 0-59
    unsigned int hour = dt->hours;      // 0-23
    unsigned int day = dt->date - 1;    // 0-30
    unsigned int month = dt->month - 1; // 0-11
    unsigned int year = dt->year - REFERENCE_YEAR; // 0-99
    int32_t unixTime = (((year / 4 * (365 * 4 + 1) + days[year % 4][month] + day) * 24 + hour) * 60 + minute) * 60 + second + EPOCH_2000_OFFSET;
    return rtcTimeMake(unixTime, dt->millis);
}

static uint8_t rtcGetWday(int year, int month, int day)
{
    if (month == 1 || month == 2) {
        month += 12;
        year -= 1;
    }

    int K = year % 100;
    int J = year / 100;

    int h = (day + (13 * (month + 1)) / 5 + K + K / 4 + J / 4 + 5 * J) % 7;

    return (h + 5) % 7 + 1;
}

static void rtcTimeToDateTime(dateTime_t *dt, rtcTime_t t)
{
    int32_t unixTime = t / MILLIS_PER_SECOND - EPOCH_2000_OFFSET;
    dt->seconds = unixTime % 60;
    unixTime /= 60;
    dt->minutes = unixTime % 60;
    unixTime /= 60;
    dt->hours = unixTime % 24;
    unixTime /= 24;

    unsigned int years = unixTime / (365 * 4 + 1) * 4;
    unixTime %= 365 * 4 + 1;

    unsigned int year;
    for (year = 3; year > 0; year--) {
        if (unixTime >= days[year][0]) {
            break;
        }
    }

    unsigned int month;
    for (month = 11; month > 0; month--) {
        if (unixTime >= days[year][month]) {
            break;
        }
    }

    dt->year = years + year + REFERENCE_YEAR;
    dt->month = month + 1;
    dt->date = unixTime - days[year][month] + 1;
    dt->millis = t % MILLIS_PER_SECOND;

    dt->wday = rtcGetWday(dt->year, dt->month, dt->date);
}

static void dateTimeWithOffset(dateTime_t *dateTimeOffset, dateTime_t *dateTimeInitial, RTC_TIMEZONE_UTC_e timezone)
{
    rtcTime_t initialTime = dateTimeToRtcTime(dateTimeInitial);
    rtcTime_t offsetTime = rtcTimeMake(rtcTimeGetSeconds(&initialTime) + timezone * 6 * 60, rtcTimeGetMillis(&initialTime));
    rtcTimeToDateTime(dateTimeOffset, offsetTime);
}

bool rtcGetLocalDateTime(dateTime_t *localDateTime)
{
    if (HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY) {
        return false;
    }

    dateTime_t utcDateTime;
    rtcGetDateTime(&utcDateTime);
    dateTimeWithOffset(localDateTime, &utcDateTime, timezone);

    return true;
}

bool rtcSetLocalDateTime(dateTime_t *localDateTime)
{
    if (HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY) {
        return false;
    }

    dateTime_t utcDateTime;
    dateTimeWithOffset(&utcDateTime, localDateTime, -timezone);
    rtcSetDateTime(&utcDateTime);

    return true;
}
