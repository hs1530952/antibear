#pragma once

#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/io.h"

#define ADC_TAG_MAP_COUNT 4

typedef struct adcTagMap_s {
    pinDef_t pin;
    uint8_t devices;
    uint32_t channel;
    uint8_t channelOrdinal;
} adcTagMap_t;

// Encoding for adcTagMap_t.devices

#define ADC_DEVICES_1   (1 << ADCDEV_1)
#define ADC_DEVICES_2   (1 << ADCDEV_2)
#define ADC_DEVICES_3   (1 << ADCDEV_3)
#define ADC_DEVICES_12  ((1 << ADCDEV_1)|(1 << ADCDEV_2))
#define ADC_DEVICES_123 ((1 << ADCDEV_1)|(1 << ADCDEV_2)|(1 << ADCDEV_3))

typedef struct adcDevice_s {
    ADC_TypeDef *ADCx;
    ADC_HandleTypeDef ADCHandle;
    DMA_HandleTypeDef DmaHandle;
    uint8_t irq;
    uint32_t channelBits;
} adcDevice_t;

extern int32_t adcVREFINTCAL;      // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
extern int32_t adcTSCAL1;
extern int32_t adcTSCAL2;
extern int32_t adcTSSlopeK;

extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(pinDef_t ioTag);
ADCDevice adcDeviceByInstance(const ADC_TypeDef *instance);
bool adcVerifyPin(pinDef_t tag, ADCDevice device);

// Marshall values in DMA instance/channel based order to adcChannel based order.
// Required for multi DMA instance implementation
void adcGetChannelValues(void);

