#include <stdint.h>

#include "platform.h"

#include "drivers/adc.h"
#include "drivers/dma_reqmap.h"

#include "drivers/mcu/dma_reqmap_mcu.h"

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
    uint8_t dmaRequest;
} dmaPeripheralMapping_t;

#define REQMAP(periph, device) { DMA_PERIPH_ ## periph, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device }

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
    REQMAP(ADC, 1),
    REQMAP(ADC, 2),
    REQMAP(ADC, 3),
};

#define DMA(d, s) { DMA_CODE(d, s, 0), (dmaResource_t *)DMA ## d ## _Stream ## s, 0 }

static dmaChannelSpec_t dmaChannelSpec[MAX_PERIPHERAL_DMA_OPTIONS] = {
    DMA(1, 0),
    DMA(1, 1),
    DMA(1, 2),
    DMA(1, 3),
    DMA(1, 4),
    DMA(1, 5),
    DMA(1, 6),
    DMA(1, 7),
    DMA(2, 0),
    DMA(2, 1),
    DMA(2, 2),
    DMA(2, 3),
    DMA(2, 4),
    DMA(2, 5),
    DMA(2, 6),
    DMA(2, 7),
};

#undef DMA

static void dmaSetupRequest(dmaChannelSpec_t *dmaSpec, uint8_t request)
{
    // Setup request as channel
    dmaSpec->channel = request;

    // Insert DMA request into code
    dmaCode_t code = dmaSpec->code;
    dmaSpec->code = DMA_CODE(DMA_CODE_CONTROLLER(code), DMA_CODE_STREAM(code), dmaSpec->channel);
}

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (const dmaPeripheralMapping_t *periph = dmaPeripheralMapping; periph < ARRAYEND(dmaPeripheralMapping); periph++) {
        if (periph->device == device && periph->index == index) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[opt];
            dmaSetupRequest(dmaSpec, periph->dmaRequest);
            return dmaSpec;
        }
    }

    return NULL;
}
