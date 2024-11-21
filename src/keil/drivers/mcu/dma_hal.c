#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/dma.h"

dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA1, 0,  0),
    DEFINE_DMA_CHANNEL(DMA1, 1,  6),
    DEFINE_DMA_CHANNEL(DMA1, 2, 16),
    DEFINE_DMA_CHANNEL(DMA1, 3, 22),
    DEFINE_DMA_CHANNEL(DMA1, 4, 32),
    DEFINE_DMA_CHANNEL(DMA1, 5, 38),
    DEFINE_DMA_CHANNEL(DMA1, 6, 48),
    DEFINE_DMA_CHANNEL(DMA1, 7, 54),

    DEFINE_DMA_CHANNEL(DMA2, 0,  0),
    DEFINE_DMA_CHANNEL(DMA2, 1,  6),
    DEFINE_DMA_CHANNEL(DMA2, 2, 16),
    DEFINE_DMA_CHANNEL(DMA2, 3, 22),
    DEFINE_DMA_CHANNEL(DMA2, 4, 32),
    DEFINE_DMA_CHANNEL(DMA2, 5, 38),
    DEFINE_DMA_CHANNEL(DMA2, 6, 48),
    DEFINE_DMA_CHANNEL(DMA2, 7, 54),
};

static void enableDmaClock(int index)
{
    switch ((uint32_t)dmaDescriptors[index].dma) {
        case (uint32_t)DMA1: {
            __HAL_RCC_DMA1_CLK_ENABLE();
        } break;

        case (uint32_t)DMA2: {
            __HAL_RCC_DMA2_CLK_ENABLE();
        } break;
    }
}

void dmaEnable(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    enableDmaClock(index);
}
