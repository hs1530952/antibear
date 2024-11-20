#include <stdint.h>
#include "platform.h"

#include "drivers/time.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

#define PERSISTENT_OBJECT_MAGIC_VALUE (('C' << 24)|('B' << 16)|('R' << 8)|('M' << 0))

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    RTC_HandleTypeDef rtcHandle = { .Instance = RTC };

    uint32_t value = HAL_RTCEx_BKUPRead(&rtcHandle, id);

    return value;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    RTC_HandleTypeDef rtcHandle = { .Instance = RTC };

    __HAL_RTC_WRITEPROTECTION_DISABLE(&rtcHandle); // Apply sequence
    HAL_RTCEx_BKUPWrite(&rtcHandle, id, value);
    __HAL_RTC_WRITEPROTECTION_ENABLE(&rtcHandle); // Reset sequence
}

void persistentObjectRTCEnable(void)
{
    HAL_PWR_EnableBkUpAccess(); // Disable backup domain protection

    __HAL_RCC_RTC_CLK_ENABLE(); // Enable RTC module
}

void persistentObjectInit(void)
{
    // Configure and enable RTC for backup register access

    persistentObjectRTCEnable();

    // XXX Magic value checking may be sufficient

    uint32_t wasSoftReset = RCC->RSR & RCC_RSR_SFTRSTF;

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
        persistentObjectWrite(PERSISTENT_OBJECT_TIMEZONE, 0xFFFFFFFF);
    }
}
