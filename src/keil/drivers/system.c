#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/system.h"

void SysTick_Handler(void)
{
#ifdef USE_HAL_DRIVER
    // used by the HAL for some timekeeping and timeout, should always be 1ms
    HAL_IncTick();
#endif
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
    memset(&_sdmaram_bss, 0x00, (size_t) (&_edmaram_bss - &_sdmaram_bss));
    memcpy(&_sdmaram_data, &_sdmaram_idata, (size_t) (&_edmaram_data - &_sdmaram_data));
#endif
}
