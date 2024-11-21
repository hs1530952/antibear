#pragma once

// dmaResource_t is a opaque data type which represents a single DMA engine,
// called and implemented differently in different families of STM32s.
// The opaque data type provides uniform handling of the engine in source code.
// The engines are referenced by dmaResource_t through out the Betaflight code,
// and then converted back to DMA_ARCH_TYPE which is a native type for
// the particular MCU type when calling library functions.

typedef struct dmaResource_s dmaResource_t;

#define DMA_ARCH_TYPE DMA_Stream_TypeDef

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
    dmaResource_t               *ref;
    uint8_t                     stream;
    uint32_t                    channel;
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    userParam;
    uint32_t                    completeFlag;
    bool                        isUsed;
} dmaChannelDescriptor_t;

#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - 1)

typedef enum {
    DMA_NONE = 0,
    DMA1_ST0_HANDLER = 1,
    DMA1_ST1_HANDLER,
    DMA1_ST2_HANDLER,
    DMA1_ST3_HANDLER,
    DMA1_ST4_HANDLER,
    DMA1_ST5_HANDLER,
    DMA1_ST6_HANDLER,
    DMA1_ST7_HANDLER,
    DMA2_ST0_HANDLER,
    DMA2_ST1_HANDLER,
    DMA2_ST2_HANDLER,
    DMA2_ST3_HANDLER,
    DMA2_ST4_HANDLER,
    DMA2_ST5_HANDLER,
    DMA2_ST6_HANDLER,
    DMA2_ST7_HANDLER,
    DMA_LAST_HANDLER = DMA2_ST7_HANDLER
} dmaIdentifier_e;

#define DEFINE_DMA_CHANNEL(d, s, f) { \
    .dma = d, \
    .ref = (dmaResource_t *)d ## _Stream ## s, \
    .stream = s, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Stream ## s ## _IRQn, \
    .userParam = 0,\
    .isUsed = false, \
    }

dmaIdentifier_e dmaAllocate(dmaIdentifier_e identifier);
void dmaEnable(dmaIdentifier_e identifier);

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel);
