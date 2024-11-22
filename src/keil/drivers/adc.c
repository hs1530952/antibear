#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "common/utils.h"

#include "drivers/adc_impl.h"
#include "drivers/io.h"

#include "drivers/adc.h"

adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];

volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(pinDef_t pin)
{
    for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (IOCheckSame(pin, adcTagMap[i].pin))
            return adcTagMap[i].channel;
    }
    return 0;
}

ADCDevice adcDeviceByInstance(const ADC_TypeDef *instance)
{
    if (instance == ADC1) {
        return ADCDEV_1;
    }

    if (instance == ADC2) {
        return ADCDEV_2;
    }

    if (instance == ADC3) {
        return ADCDEV_3;
    }

    return ADCINVALID;
}

uint16_t adcGetChannel(uint8_t channel)
{
    adcGetChannelValues();

    return adcValues[adcOperatingConfig[channel].dmaIndex];
}

// Verify a pin designated by tag has connection to an ADC instance designated by device

bool adcVerifyPin(pinDef_t pin, ADCDevice device)
{
    if (!pin.GPIOx) {
        return false;
    }

    for (int map = 0 ; map < ADC_TAG_MAP_COUNT ; map++) {
        if (IOCheckSame(pin, adcTagMap[map].pin) && (adcTagMap[map].devices & (1 << device))) {
            return true;
        }
    }

    return false;
}

int32_t adcVREFINTCAL;      // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
int32_t adcTSCAL1;
int32_t adcTSCAL2;
int32_t adcTSSlopeK;

/**
 * Use a measurement of the fixed internal vref to calculate the external Vref+
 *
 * The ADC full range reading equates to Vref+ on the channel. Vref+ is typically
 * fed from Vcc at 3.3V, but since Vcc isn't a critical value it may be off
 * by a little due to variation in the regulator. Some chips are provided with a
 * known internal voltage reference, typically around 1.2V. By measuring this
 * reference with an internally connected ADC channel we can then calculate a more
 * accurate value for Vref+ instead of assuming that it is 3.3V
 *
 * @param intVRefAdcValue reading from the internal calibration voltage
 *
 * @return the calculated value of Vref+
*/
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
