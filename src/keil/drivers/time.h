#pragma once

#include <stdint.h>

#include "common/time.h"

void delayMicroseconds(timeUs_t us);
void delay(timeMs_t ms);

timeUs_t micros(void);
timeUs_t microsISR(void);
timeMs_t millis(void);

void rtcInit(void);

bool rtcSetTimeZone(RTC_TIMEZONE_UTC_e *tz);
bool rtcGetTimeZone(RTC_TIMEZONE_UTC_e *tz);

bool rtcGetDateTime(dateTime_t *dt);
bool rtcSetDateTime(dateTime_t *dt);
bool rtcGetLocalDateTime(dateTime_t *localDateTime);
bool rtcSetLocalDateTime(dateTime_t *localDateTime);
