#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debugSerial.h"

#include "drivers/adc.h"
#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/system.h"
#include "drivers/eeprom/eeprom.h"
#include "drivers/eeprom/eeprom_impl.h"

#include "system/tasks.h"

#include "system/init.h"

#include "sensors/adcinternal.h"

uint8_t systemState = SYSTEM_STATE_INITIALISING;

void init(void)
{
    systemInit();

    debugSerialInit();
    debugSerialPrint("System Initailizeing!");

    // Initialize task data as soon as possible. Has to be done before tasksInit()
    // and any init code that may try to modify task behaviour before tasksInit().
    tasksInitData();

    i2cHardwareConfigure();
    for (int i2cindex = 0; i2cindex < I2CDEV_COUNT; i2cindex++) {
        i2cInit(i2cindex);
    }
    eepromInit();

    adcInit();
    adcInternalInit();

    tasksInit();

    systemState |= SYSTEM_STATE_READY;
}
