#pragma once

#include "platform.h"

#include "drivers/dma.h"

typedef uint16_t dmaCode_t;

typedef struct dmaChannelSpec_s {
    dmaCode_t             code;
    dmaResource_t         *ref;
    uint32_t              channel;
} dmaChannelSpec_t;

typedef enum {
    DMA_PERIPH_ADC,
} dmaPeripheral_e;

#define DMA_CODE(dma, stream, chanreq) ((dma << 12)|(stream << 8)|(chanreq << 0))
#define DMA_CODE_CONTROLLER(code) ((code >> 12) & 0xf)
#define DMA_CODE_STREAM(code) ((code >> 8) & 0xf)

#define DMA_OPT_UNUSED (-1)

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt);
