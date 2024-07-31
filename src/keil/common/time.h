#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

// time difference, 32 bits always sufficient
typedef int32_t timeDelta_t;
// millisecond time
typedef uint32_t timeMs_t;
// microsecond time
typedef uint32_t timeUs_t;
#define TIMEUS_MAX UINT32_MAX

#define TIMEZONE_OFFSET_MINUTES_MIN -780  // -13 hours
#define TIMEZONE_OFFSET_MINUTES_MAX 780   // +13 hours

#define SECONDS_PER_MINUTE          60.0f

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }
static inline int32_t cmpTimeCycles(uint32_t a, uint32_t b) { return (int32_t)(a - b); }

#define FORMATTED_DATE_TIME_BUFSIZE 30
