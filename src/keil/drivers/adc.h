#pragma once

#include <stdbool.h>

#include "drivers/io.h"
#include "drivers/time.h"

#define ADC_INSTANCE                ADC3

typedef enum ADCDevice {
    ADCINVALID = -1,
    ADCDEV_1   = 0,
    ADCDEV_2,
    ADCDEV_3,
    ADCDEV_COUNT
} ADCDevice;

#define ADC_CFG_TO_DEV(x) ((x) - 1)
#define ADC_DEV_TO_CFG(x) ((x) + 1)

typedef enum {
    ADC_EXTERNAL1 = 0,
    // On H7 internal sensors are treated in the similar fashion as regular ADC inputs
    ADC_CHANNEL_INTERNAL_FIRST_ID = 1,

    ADC_TEMPSENSOR = 1,
    ADC_VREFINT = 2,
    ADC_VBAT4 = 3,
    ADC_CHANNEL_COUNT
} AdcChannel;

typedef struct adcOperatingConfig_s {
    pinDef_t pin;
    ADCDevice adcDevice;        // ADCDEV_x for this input
    uint32_t adcChannel;        // Channel number for this input. Note that H7 and G4 HAL requires this to be 32-bit encoded number.
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
    bool enabled;
    uint8_t sampleTime;
} adcOperatingConfig_t;

void adcInit(void);
uint16_t adcGetChannel(uint8_t channel);

bool adcInternalIsBusy(void);
void adcInternalStartConversion(void);
uint16_t adcInternalReadVrefint(void);
uint16_t adcInternalReadTempsensor(void);
uint16_t adcInternalCompensateVref(uint16_t vrefAdcValue);
int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue);
