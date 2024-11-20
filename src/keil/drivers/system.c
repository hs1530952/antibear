#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"

#include "drivers/nvic.h"

#include "drivers/system.h"

// See "RM CoreSight Architecture Specification"
// B2.3.10  "LSR and LAR, Software Lock Status Register and Software Lock Access Register"
// "E1.2.11  LAR, Lock Access Register"

#define DWT_LAR_UNLOCK_VALUE 0xC5ACCE55

// cycles per microsecond
static uint32_t usTicks = 0;
static float usTicksInv = 0.0f;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;
static uint32_t systemTIMClockFrequency = 0;

void cycleCounterInit(void)
{
    extern uint32_t Get_SystemTIMClockFrequency(void);
    systemTIMClockFrequency = Get_SystemTIMClockFrequency();
    usTicks = systemTIMClockFrequency / 1000000;
    usTicksInv = 1e6f / systemTIMClockFrequency;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM->LAR = DWT_LAR_UNLOCK_VALUE;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// System TIM

static volatile int sysTickPending = 0;

void SystemTIM_IRQHandler(void)
{
    if (SystemTIM_CheckUpdateFlag()) {
        SystemTIM_ClearUpdateFlag();

        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            sysTickUptime++;
            sysTickValStamp = SystemTIM_GetCNT();
            sysTickPending = 0;
        }
    }
    // used by the HAL for some timekeeping and timeouts, should always be 1ms
    HAL_IncTick();
}

// Return system uptime in microseconds (rollover in 70minutes)

uint32_t microsISR(void)
{
    register uint32_t ms, pending, cycle_cnt;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        cycle_cnt = SystemTIM_GetCNT();

        if (SystemTIM_CheckUpdateFlag()) {
            // Update pending.
            // Record it for multiple calls within the same rollover period
            // (Will be cleared when serviced).
            // Note that multiple rollovers are not considered.

            sysTickPending = 1;

            // Read VAL again to ensure the value is read after the rollover.

            cycle_cnt = SystemTIM_GetCNT();
        }

        ms = sysTickUptime;
        pending = sysTickPending;
    }

    return ((ms + pending) * 1000) + (cycle_cnt / usTicks);
}

uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;

    // Call microsISR() in interrupt and elevated (non-zero) BASEPRI context

    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
        return microsISR();
    }

    do {
        ms = sysTickUptime;
        cycle_cnt = SystemTIM_GetCNT();
    } while (ms != sysTickUptime || cycle_cnt < sysTickValStamp);

    return (ms * 1000) + (cycle_cnt / usTicks);
}

uint32_t getCycleCounter(void)
{
    return DWT->CYCCNT;
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / usTicks;
}

float clockCyclesToMicrosf(int32_t clockCycles)
{
    return clockCycles * usTicksInv;
}

// Note that this conversion is signed as this is used for periods rather than absolute timestamps
int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return 10 * clockCycles / (int32_t)usTicks;
}

// Note that this conversion is signed as this is used for periods rather than absolute timestamps
int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return 100 * clockCycles / (int32_t)usTicks;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

void initialiseMemorySections(void)
{
#if !defined (__CC_ARM) && !defined(__ARMCC_VERSION)
    /* Load fast-functions into ITCM RAM */
    extern uint8_t tcm_code_start;
    extern uint8_t tcm_code_end;
    extern uint8_t tcm_code;
    memcpy(&tcm_code_start, &tcm_code, (size_t) (&tcm_code_end - &tcm_code_start));

    /* Load FAST_DATA variable initializers into DTCM RAM */
    extern uint8_t _sfastram_data;
    extern uint8_t _efastram_data;
    extern uint8_t _sfastram_idata;
    memcpy(&_sfastram_data, &_sfastram_idata, (size_t) (&_efastram_data - &_sfastram_data));
#endif
}

void initialiseD2MemorySections(void)
{
#if !defined (__CC_ARM) && !defined(__ARMCC_VERSION)
    /* Load DMA_DATA variable intializers into D2 RAM */
    extern uint8_t _sdmaram_bss;
    extern uint8_t _edmaram_bss;
    extern uint8_t _sdmaram_data;
    extern uint8_t _edmaram_data;
    extern uint8_t _sdmaram_idata;
    bzero(&_sdmaram_bss, (size_t) (&_edmaram_bss - &_sdmaram_bss));
    memcpy(&_sdmaram_data, &_sdmaram_idata, (size_t) (&_edmaram_data - &_sdmaram_data));
#endif
}
