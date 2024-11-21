#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/dma.h"

#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#define ADC1_DMA_OPT (DMA_OPT_UNUSED)
#define ADC2_DMA_OPT (DMA_OPT_UNUSED)
#define ADC3_DMA_OPT 9

typedef struct adcChannelConfig_s {
    pinDef_t pin;
    int8_t device;
} adcChannelConfig_t;

typedef struct adcConfig_s {
    adcChannelConfig_t external1;
    int8_t device;

    uint16_t vrefIntCalibration;
    uint16_t tempSensorCalibration1;
    uint16_t tempSensorCalibration2;

    int8_t dmaopt[ADCDEV_COUNT];
} adcConfig_t;

const adcConfig_t adcConfig = {
    .external1 = {
        .pin = {GPIOC, GPIO_PIN_3},
        .device = ADC_DEVICES_3,
    },
    .device = ADC_DEVICES_3,

    .vrefIntCalibration = 0,
    .tempSensorCalibration1 = 0,
    .tempSensorCalibration2 = 0,

    .dmaopt[ADCDEV_1] = ADC1_DMA_OPT,
    .dmaopt[ADCDEV_2] = ADC2_DMA_OPT,
    .dmaopt[ADCDEV_3] = ADC3_DMA_OPT,
};

const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
    },
    {
        .ADCx = ADC2,
    },
    {
        .ADCx = ADC3,
    }
};

adcDevice_t adcDevice[ADCDEV_COUNT];

static void adcSetRCC(adcDevice_t *device)
{
    switch ((uint32_t)device->ADCx) {
        case (uint32_t)ADC1: {
            __HAL_RCC_ADC12_CLK_ENABLE();
        } break;

        case (uint32_t)ADC2: {
            __HAL_RCC_ADC12_CLK_ENABLE();
        } break;

        case (uint32_t)ADC3: {
            __HAL_RCC_ADC3_CLK_ENABLE();
        } break;
    }
}

#define ADC_DEVICE_FOR_INTERNAL ADC_DEVICES_3

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
    // Pseudo entries for internal sensor.
    // Keep these at the beginning for easy indexing by ADC_TAG_MAP_{VREFINT,TEMPSENSOR}
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1
#define ADC_TAG_MAP_VBAT4      2

    { {NULL,  NULL},       ADC_DEVICE_FOR_INTERNAL, ADC_CHANNEL_VREFINT,    19 }, // 19 VREFINT
    { {NULL,  NULL},       ADC_DEVICE_FOR_INTERNAL, ADC_CHANNEL_TEMPSENSOR, 18 }, // 18 VSENSE
    { {NULL,  NULL},       ADC_DEVICE_FOR_INTERNAL, ADC_CHANNEL_VBAT,       17 }, // 17 VBAT/4

    { {GPIOC, GPIO_PIN_3}, ADC_DEVICES_3,           ADC_CHANNEL_1,           1 },
};

// Translate rank number x to ADC_REGULAR_RANK_x (Note that array index is 0-origin)

#define RANK(n) ADC_REGULAR_RANK_ ## n

static uint32_t adcRegularRankMap[] = {
    RANK(1),
    RANK(2),
    RANK(3),
    RANK(4),
    RANK(5),
    RANK(6),
    RANK(7),
    RANK(8),
    RANK(9),
    RANK(10),
    RANK(11),
    RANK(12),
    RANK(13),
    RANK(14),
    RANK(15),
    RANK(16),
};

#undef RANK

static void errorHandler(void) { while (1) { } }

// Note on sampling time.
// Temperature sensor has minimum sample time of 9us.
// With prescaler = 4 at 240MHz (AHB1), fADC = 60MHz (tcycle = 0.0167us), 9us = 540cycles < 810

void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{
    ADC_HandleTypeDef *hadc = &adcdev->ADCHandle; // For clarity

    hadc->Instance = adcdev->ADCx;

    // DeInit is done in adcInit().

    hadc->Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution               = ADC_RESOLUTION_12B;
    hadc->Init.ScanConvMode             = ENABLE;                        // Works with single channel, too
    hadc->Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait         = DISABLE;
    hadc->Init.ContinuousConvMode       = ENABLE;
    hadc->Init.NbrOfConversion          = channelCount;
    hadc->Init.DiscontinuousConvMode    = DISABLE;
    hadc->Init.NbrOfDiscConversion      = 1;                             // Don't care
    hadc->Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE; // Don't care

    // Enable circular DMA.
    hadc->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;

    hadc->Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    hadc->Init.OversamplingMode         = DISABLE;

    // Initialize this ADC peripheral

    if (HAL_ADC_Init(hadc) != HAL_OK) {
      errorHandler();
    }

    // Execute calibration

    if (HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
      errorHandler();
    }
}

int adcFindTagMapEntry(pinDef_t pin)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].pin.GPIOx == pin.GPIOx && adcTagMap[i].pin.GPIO_Pin == pin.GPIO_Pin) {
            return i;
        }
    }
    return -1;
}

#define VREFINT_CAL_SHIFT 4

void adcInitCalibrationValues(void)
{
    adcVREFINTCAL = *VREFINT_CAL_ADDR >> VREFINT_CAL_SHIFT;
    adcTSCAL1 = *TEMPSENSOR_CAL1_ADDR >> VREFINT_CAL_SHIFT;
    adcTSCAL2 = *TEMPSENSOR_CAL2_ADDR >> VREFINT_CAL_SHIFT;
    adcTSSlopeK = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * 1000 / (adcTSCAL2 - adcTSCAL1);
}

// ADC conversion result DMA buffer
// Need this separate from the main adcValue[] array, because channels are numbered
// by ADC instance order that is different from ADC_xxx numbering.

#define ADC_BUF_LENGTH ADC_CHANNEL_COUNT
#define ADC_BUF_BYTES (ADC_BUF_LENGTH * sizeof(uint16_t))
#define ADC_BUF_CACHE_ALIGN_BYTES  ((ADC_BUF_BYTES + 0x20) & ~0x1f)
#define ADC_BUF_CACHE_ALIGN_LENGTH (ADC_BUF_CACHE_ALIGN_BYTES / sizeof(uint16_t))

static volatile DMA_RAM uint16_t adcConversionBuffer[ADC_BUF_CACHE_ALIGN_LENGTH] __attribute__((aligned(32)));

void adcInit(void)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memcpy(adcDevice, adcHardware, sizeof(adcDevice));

    adcOperatingConfig[ADC_EXTERNAL1].pin = adcConfig.external1.pin;
    adcOperatingConfig[ADC_EXTERNAL1].adcDevice = adcConfig.external1.device;

    adcInitCalibrationValues();

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        int map;
        int dev;

        if (i == ADC_TEMPSENSOR) {
            map = ADC_TAG_MAP_TEMPSENSOR;
            dev = __builtin_ffs(adcTagMap[map].devices) - 1;
        } else if (i == ADC_VREFINT) {
            map = ADC_TAG_MAP_VREFINT;
            dev = __builtin_ffs(adcTagMap[map].devices) - 1;
        } else if (i == ADC_VBAT4) {
            map = ADC_TAG_MAP_VBAT4;
            dev = __builtin_ffs(adcTagMap[map].devices) - 1;
        } else {
            dev = ADC_CFG_TO_DEV(adcOperatingConfig[i].adcDevice);

            if (!adcOperatingConfig[i].pin.GPIOx) {
                continue;
            }

            map = adcFindTagMapEntry(adcOperatingConfig[i].pin);
            if (map < 0) {
                continue;
            }

            // Found a tag map entry for this input pin
            // Find an ADC device that can handle this input pin

            bool useConfiguredDevice = (dev != ADCINVALID) && (adcTagMap[map].devices & (1 << dev));

            if (!useConfiguredDevice) {
                // If the ADC was configured to use a specific device, but that device was not active, then try and find another active instance that works for the pin.

                for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                    if (!adcDevice[dev].ADCx) {
                        // Instance not activated
                        continue;
                    }
                    if (adcTagMap[map].devices & (1 << dev)) {
                        // Found an activated ADC instance for this input pin
                        break;
                    }
                }

                if (dev == ADCDEV_COUNT) {
                    // No valid device found, go next channel.
                    continue;
                }
            }
        }

        // At this point, map is an entry for the input pin and dev is a valid ADCx for the pin for input i

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_810CYCLES_5;
        adcOperatingConfig[i].enabled = true;

        adcDevice[dev].channelBits |= (1 << adcTagMap[map].channelOrdinal);

        // Configure a pin for ADC
        if (adcOperatingConfig[i].pin.GPIOx) {
            IOConfigGPIO(&adcOperatingConfig[i].pin, IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
        }
    }

    // DeInit ADCx with inputs
    // We have to batch call DeInit() for all devices as DeInit() initializes ADCx_COMMON register.

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        adc->ADCHandle.Instance = adc->ADCx;

        if (HAL_ADC_DeInit(&adc->ADCHandle) != HAL_OK) {
            // ADC de-initialization Error
            errorHandler();
        }
    }

    // Configure ADCx with inputs

    int dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!adc->channelBits) {
            continue;
        }

        adcSetRCC(adc);

        int configuredAdcChannels = popcount(adc->channelBits);

        adcInitDevice(adc, configuredAdcChannels);

        // Configure channels

        int rank = 0;

        for (int adcChan = 0; adcChan < ADC_CHANNEL_COUNT; adcChan++) {

            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }

            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;

            ADC_ChannelConfTypeDef sConfig;

            sConfig.Channel      = adcOperatingConfig[adcChan].adcChannel; /* Sampled channel number */
            sConfig.Rank         = adcRegularRankMap[rank++];   /* Rank of sampled channel number ADCx_CHANNEL */
            sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;  /* Sampling time (number of clock cycles unit) */
            sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
            sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
            sConfig.Offset       = 0;                           /* Parameter discarded because offset correction is disabled */

            if (HAL_ADC_ConfigChannel(&adc->ADCHandle, &sConfig) != HAL_OK) {
                errorHandler();
            }
        }

        // Configure DMA for this ADC peripheral

        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, adcConfig.dmaopt[dev]);
        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);

        if (!dmaSpec || !dmaAllocate(dmaIdentifier)) {
            return;
        }

        adc->DmaHandle.Instance                 = dmaSpec->ref;
        adc->DmaHandle.Init.Request             = dmaSpec->channel;
        adc->DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        adc->DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
        adc->DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
        adc->DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        adc->DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        adc->DmaHandle.Init.Mode                = DMA_CIRCULAR;
        adc->DmaHandle.Init.Priority            = DMA_PRIORITY_MEDIUM;

        // Deinitialize & Initialize the DMA for new transfer

        // dmaEnable must be called before calling HAL_DMA_Init,
        // to enable clock for associated DMA if not already done so.
        dmaEnable(dmaIdentifier);

        HAL_DMA_DeInit(&adc->DmaHandle);
        HAL_DMA_Init(&adc->DmaHandle);

        // Associate the DMA handle

        __HAL_LINKDMA(&adc->ADCHandle, DMA_Handle, adc->DmaHandle);
    }

    // Start channels.
    // This must be done after channel configuration is complete, as HAL_ADC_ConfigChannel
    // throws an error when configuring internal channels if ADC1 or ADC2 are already enabled.

    dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {

        adcDevice_t *adc = &adcDevice[dev];

        if (!adc->channelBits) {
            continue;
        }

        // Start conversion in DMA mode

        if (HAL_ADC_Start_DMA(&adc->ADCHandle, (uint32_t *)&adcConversionBuffer[dmaBufferIndex], popcount(adc->channelBits)) != HAL_OK) {
            errorHandler();
        }

        dmaBufferIndex += popcount(adc->channelBits);
    }
}

void adcGetChannelValues(void)
{
    // Transfer values in conversion buffer into adcValues[]
    SCB_InvalidateDCache_by_Addr((uint32_t*)adcConversionBuffer, ADC_BUF_CACHE_ALIGN_BYTES);
    for (int i = 0; i < ADC_CHANNEL_INTERNAL_FIRST_ID; i++) {
        if (adcOperatingConfig[i].enabled) {
            adcValues[adcOperatingConfig[i].dmaIndex] = adcConversionBuffer[adcOperatingConfig[i].dmaIndex];
        }
    }
}

bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
    return;
}

uint16_t adcInternalRead(int channel)
{
    int dmaIndex = adcOperatingConfig[channel].dmaIndex;

    return adcConversionBuffer[dmaIndex];
}

uint16_t adcInternalReadVrefint(void)
{
    uint16_t value = adcInternalRead(ADC_VREFINT);
    return value;
}

uint16_t adcInternalReadTempsensor(void)
{
    uint16_t value = adcInternalRead(ADC_TEMPSENSOR);
    return value;
}
