#pragma once

#include <stdint.h>
#include <stdbool.h>

void systemInit(void);

// bootloader/IAP
void systemReset(void);
void systemResetWithoutDisablingCaches(void);
void cycleCounterInit(void);
int32_t clockCyclesToMicros(int32_t clockCycles);
float clockCyclesToMicrosf(int32_t clockCycles);
int32_t clockCyclesTo10thMicros(int32_t clockCycles);
int32_t clockCyclesTo100thMicros(int32_t clockCycles);
uint32_t clockMicrosToCycles(uint32_t micros);
uint32_t getCycleCounter(void);

// memory
void initialiseMemorySections(void);
void initialiseD2MemorySections(void);
