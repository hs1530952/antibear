#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/adc.h"

typedef struct movingAverageStateUint16_s {
    uint32_t sum;
    uint16_t *values;
    uint8_t size;
    uint8_t pos;
} movingAverageStateUint16_t;

uint16_t updateMovingAverageUint16_t(movingAverageStateUint16_t *state, uint16_t newValue)
{
    state->sum -= state->values[state->pos];
    state->values[state->pos] = newValue;
    state->sum += newValue;
    state->pos = (state->pos + 1) % state->size;

    return state->sum / state->size;
}

static uint16_t adcVrefintValue;
static uint16_t adcVrefintValues[8];
movingAverageStateUint16_t adcVrefintAverageState = { 0, adcVrefintValues, 8, 0 };

static uint16_t adcTempsensorValue;
static uint16_t adcTempsensorValues[8];
movingAverageStateUint16_t adcTempsensorAverageState = { 0, adcTempsensorValues, 8, 0 };

static int16_t coreTemperature;
static uint16_t vrefMv;

uint16_t getVrefMv(void)
{
    return vrefMv;
}

int16_t getCoreTemperatureCelsius(void)
{
    return coreTemperature;
}

void adcInternalProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (adcInternalIsBusy()) {
        return;
    }

    uint16_t vrefintSample = adcInternalReadVrefint();
    uint16_t tempsensorSample = adcInternalReadTempsensor();

    adcVrefintValue = updateMovingAverageUint16_t(&adcVrefintAverageState, vrefintSample);
    adcTempsensorValue = updateMovingAverageUint16_t(&adcTempsensorAverageState, tempsensorSample);

    vrefMv = adcInternalCompensateVref(adcVrefintValue);
    coreTemperature = adcInternalComputeTemperature(adcTempsensorValue, vrefMv);

    adcInternalStartConversion(); // Start next conversion
}

void adcInternalInit(void)
{
    // Call adcInternalProcess repeatedly to fill moving average array
    for (int i = 0; i < 9; i++) {
        while (adcInternalIsBusy()) {
            // empty
        }
        adcInternalProcess(0);
    }
}
