#pragma once

#include "drivers/time.h"

void adcInternalInit(void);
void adcInternalProcess(timeUs_t currentTimeUs);
int16_t getCoreTemperatureCelsius(void);
uint16_t getVrefMv(void);
