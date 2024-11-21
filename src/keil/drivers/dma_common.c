#include <stdbool.h>

#include "platform.h"

#include "drivers/dma_impl.h"

#include "drivers/dma.h"

dmaIdentifier_e dmaAllocate(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    // Prevent Wstringop-overflow warning
    if (index < 0 || index >= DMA_LAST_HANDLER) {
        return DMA_NONE;
    }

    if (dmaDescriptors[index].isUsed) {
        return DMA_NONE;
    }

    dmaDescriptors[index].isUsed = true;

    return identifier;
}

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t *channel)
{
    for (int i = 0; i < DMA_LAST_HANDLER; i++) {
        if (dmaDescriptors[i].ref == channel) {
            return i + 1;
        }
    }

    return 0;
}
