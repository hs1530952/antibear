#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/adc.h"

adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];

volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

int32_t adcVREFINTCAL;
int32_t adcTSCAL1;
int32_t adcTSCAL2;
int32_t adcTSSlopeK;

uint16_t adcInternalCompensateVref(uint16_t intVRefAdcValue)
{
    // This is essentially a tuned version of
    // __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefAdcValue, ADC_RESOLUTION_12B);
    return (uint16_t)((uint32_t)(adcVREFINTCAL * VREFINT_CAL_VREF) / intVRefAdcValue);
}

int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue)
{
    // This is essentially a tuned version of
    // __HAL_ADC_CALC_TEMPERATURE(vrefValue, tempAdcValue, ADC_RESOLUTION_12B);

    return ((((int32_t)((tempAdcValue * vrefValue) / TEMPSENSOR_CAL_VREFANALOG) - adcTSCAL1) * adcTSSlopeK) + 500) / 1000 + TEMPSENSOR_CAL1_TEMP;
}
