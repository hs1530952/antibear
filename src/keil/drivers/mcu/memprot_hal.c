#include <string.h>

#include "platform.h"

#include "drivers/memprot.h"

#define MAX_MPU_REGIONS 16

typedef struct mpuRegion_s {
    uint32_t start;
    uint32_t end;        // Zero if determined by size member (MPU_REGION_SIZE_xxx)
    uint8_t  size;       // Zero if determined from linker symbols
    uint8_t  perm;
    uint8_t  exec;
    uint8_t  shareable;
    uint8_t  cacheable;
    uint8_t  bufferable;
} mpuRegion_t;

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern uint8_t DMA_RAM$$Base;
extern uint8_t DMA_RAM$$Limit;

extern uint8_t DMA_RW_AXI$$Base;
extern uint8_t DMA_RW_AXI$$Limit;

extern uint8_t DMA_VRAM$$Base;
extern uint8_t DMA_VRAM$$Limit;

#define dmaram_start DMA_RAM$$Base
#define dmaram_end DMA_RAM$$Limit

#define dmarwaxi_start DMA_RW_AXI$$Base
#define dmarwaxi_end DMA_RW_AXI$$Limit

#define dmavram_start DMA_VRAM$$Base
#define dmavram_end DMA_VRAM$$Limit

#elif defined(__GNUC__)
// Defined in linker script
extern uint8_t dmaram_start;
extern uint8_t dmaram_end;

extern uint8_t dmarwaxi_start;
extern uint8_t dmarwaxi_end;

extern uint8_t dmavram_start;
extern uint8_t dmavram_end;

#endif

static mpuRegion_t mpuRegions[] = {
    {
        // Mark ITCM-RAM as read-only
        // "For Cortex®-M7, TCMs memories always behave as Non-cacheable, Non-shared normal memories, irrespective of the memory type attributes defined in the MPU for a memory region containing addresses held in the TCM"
        // See AN4838
        .start      = 0x00000000,
        .end        = 0, // Size defined by "size"
        .size       = MPU_REGION_SIZE_64KB,
        .perm       = MPU_REGION_PRIV_RO_URO,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_NOT_SHAREABLE,
        .cacheable  = MPU_ACCESS_NOT_CACHEABLE,
        .bufferable = MPU_ACCESS_BUFFERABLE,
    },
    {
        // DMA transmit buffer in D2 SRAM1
        // Reading needs cache coherence operation
        .start      = (uint32_t)&dmaram_start,
        .end        = (uint32_t)&dmaram_end,
        .size       = 0, // Size determined by ".end"
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
    {
        // A region in AXI RAM accessible from SDIO internal DMA
        .start      = (uint32_t)&dmarwaxi_start,
        .end        = (uint32_t)&dmarwaxi_end,
        .size       = 0, // Size determined by ".end"
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_NOT_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
    {
        // DMA transmit buffer in SDRAM2
        // Reading needs cache coherence operation
        .start      = (uint32_t)&dmavram_start,
        .end        = (uint32_t)&dmavram_end,
        .size       = 0, // Size determined by ".end"
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
};

unsigned mpuRegionCount = ARRAYLEN(mpuRegions);

void memProtConfigure(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    // Disable the MPU
    HAL_MPU_Disable();

    // Setup common members
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;

    for (unsigned number = 0, valid = 0; number < mpuRegionCount; number++) {
        mpuRegion_t *region = &mpuRegions[number];

        if (region->end == 0 && region->size == 0) {
            continue;
        }

        MPU_InitStruct.Number       = valid;
        MPU_InitStruct.BaseAddress  = region->start;

        if (region->size) {
            MPU_InitStruct.Size     = region->size;
        } else {
            // Adjust start of the region to align with cache line size.
            uint32_t start = region->start & ~0x1F;
            uint32_t length = region->end - start;

            if (length < 32) {
                // This will also prevent flsl from returning negative (case length == 0)
                length = 32;
            }

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
            int msbpos = sizeof(length) * 8 - __builtin_clz(length) - 2;
#else
            int msbpos = flsl(length) - 1;
#endif

            if (length != (1U << msbpos)) {
                msbpos += 1;
            }

            MPU_InitStruct.Size = msbpos;
        }

        // Copy per region attributes
        MPU_InitStruct.AccessPermission = region->perm;
        MPU_InitStruct.DisableExec      = region->exec;
        MPU_InitStruct.IsShareable      = region->shareable;
        MPU_InitStruct.IsCacheable      = region->cacheable;
        MPU_InitStruct.IsBufferable     = region->bufferable;

        HAL_MPU_ConfigRegion(&MPU_InitStruct);

        valid++;
    }

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void memProtReset(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disable the MPU */
    HAL_MPU_Disable();

    // Disable existing regions

    for (uint8_t region = 0; region <= MAX_MPU_REGIONS; region++) {
        MPU_InitStruct.Enable = MPU_REGION_DISABLE;
        MPU_InitStruct.Number = region;
        HAL_MPU_ConfigRegion(&MPU_InitStruct);
    }

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
