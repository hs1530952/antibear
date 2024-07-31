#pragma once

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "system_stm32h7xx.h"

#define SystemTIM                       TIM7
#define SystemTIM_IRQHandler            TIM7_IRQHandler
#define SystemTIM_CheckUpdateFlag()     ((SystemTIM->SR & TIM_IT_UPDATE) && (SystemTIM->DIER & TIM_IT_UPDATE))
#define SystemTIM_ClearUpdateFlag()     (SystemTIM->SR = ~TIM_IT_UPDATE)
#define SystemTIM_GetCNT()              (SystemTIM->CNT)

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".bss.dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA

#define DMA_RAM                     __attribute__((section("DMA_RAM"), aligned(32)))
#define DMA_RW_AXI                  __attribute__((section("DMA_RW_AXI"), aligned(32)))

#define SDRAM2_DATA_ZERO_INIT       __attribute__ ((section(".bss.sdram2_bss"), aligned(4)))
#define SDRAM2_DATA                 __attribute__ ((section(".sdram2_data"), aligned(4)))
#define DMA_VRAM                    __attribute__((section("DMA_VRAM"), aligned(32)))

#elif defined(__GNUC__)
#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA

#define DMA_RAM                     __attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI                  __attribute__((section(".DMA_RW_AXI"), aligned(32)))

#define SDRAM2_DATA_ZERO_INIT       __attribute__ ((section(".sdram2_bss"), aligned(4)))
#define SDRAM2_DATA                 __attribute__ ((section(".sdram2_data"), aligned(4)))
#define DMA_VRAM                    __attribute__((section(".DMA_VRAM"), aligned(32)))
#endif
