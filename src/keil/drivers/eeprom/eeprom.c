#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"

#include "drivers/eeprom/eeprom.h"
#include "drivers/eeprom/eeprom_impl.h"
#include "drivers/eeprom/eeprom_at24c02.h"

typedef struct eepromConfig_s {
    uint8_t i2c_device;
    uint8_t i2c_address;
    uint8_t eeprom_hardware;
} eepromConfig_t;

const eepromConfig_t eepromConfig = {
    .i2c_device = I2C_DEV_TO_CFG(I2CDEV_1),
    .i2c_address = 0x50,
    .eeprom_hardware = EEPROM_AT24C02,
};

static extDevice_t devInstance;

static eepromDevice_t eepromDevice; 

bool eepromIsReady(void)
{
    return eepromDevice.vTable->isReady(&eepromDevice);
}

bool eepromWaitForReady(void)
{
    return eepromDevice.vTable->waitForReady(&eepromDevice);
}

void eepromProgram(uint32_t address, const uint8_t *buffer, uint16_t length)
{
    eepromDevice.vTable->program(&eepromDevice, address, buffer, length);
}

int eepromReadBytes(uint32_t address, uint8_t *buffer, uint16_t length)
{
    return eepromDevice.vTable->readBytes(&eepromDevice, address, buffer, length);
}

const eepromGeometry_t *eepromGetGeometry(void)
{
    return eepromDevice.vTable->getGeometry(&eepromDevice);
}

static bool eepromDetect(void)
{
    extDevice_t *dev = &devInstance;

    eeprom_e eepromHardware = eepromConfig.eeprom_hardware;

    if (!i2cBusSetInstance(dev, eepromConfig.i2c_device)) {
        return false;
    }
    dev->busType_u.i2c.address = eepromConfig.i2c_address;

    bool isI2CValid = false;
    for (uint8_t try = 0; try < 3 && !isI2CValid; try++) {
        uint8_t data;
        isI2CValid = i2cBusReadRegisterBufferStart(dev, 0x00, &data, 1);
    }

    if (!isI2CValid) {
        return false;
    }

    busDeviceRegister(dev);
    eepromDevice.dev = dev;

    switch (eepromHardware) {
        case EEPROM_AT24C02: {
            at24c02_init(&eepromDevice);
        } break;
        
        default:
            return false;
    }

    return true;
}

bool eepromInit(void)
{
    bool haveEeprom = eepromDetect();

    return haveEeprom;
}