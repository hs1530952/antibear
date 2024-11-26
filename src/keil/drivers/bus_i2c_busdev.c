#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"

static uint8_t i2cRegisteredDeviceCount = 0;

bool i2cBusWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    return i2cWrite(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, reg, data);
}

bool i2cBusWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    // Need a static value, not on the stack
    static uint8_t byte;

    if (i2cBusy(dev->bus->busType_u.i2c.device, NULL)) {
        return false;
    }

    byte = data;

    return i2cWriteBuffer(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, reg, sizeof(byte), &byte);
}

bool i2cBusWriteBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    return i2cWriteBuffer(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, reg, length, data);
}

bool i2cBusReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    return i2cRead(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, reg, length, data);
}

uint8_t i2cBusReadRegister(const extDevice_t *dev, uint8_t reg)
{
    uint8_t data;
    i2cRead(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, reg, 1, &data);
    return data;
}

bool i2cBusReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    return i2cReadBuffer(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, reg, length, data);
}

bool i2cBusBusy(const extDevice_t *dev, bool *error)
{
    return i2cBusy(dev->bus->busType_u.i2c.device, error);
}

bool i2cBusSetInstance(extDevice_t *dev, uint32_t device)
{
    // I2C bus structures to associate with external devices
    static busDevice_t i2cBus[I2CDEV_COUNT];

    if ((device < 1) || (device > I2CDEV_COUNT)) {
        return false;
    }

    dev->bus = &i2cBus[I2C_CFG_TO_DEV(device)];
    dev->bus->busType = BUS_TYPE_I2C;
    dev->bus->busType_u.i2c.device = I2C_CFG_TO_DEV(device);

    return true;
}

void i2cBusDeviceRegister(const extDevice_t *dev)
{
    UNUSED(dev);

    i2cRegisteredDeviceCount++;
}

uint8_t i2cGetRegisteredDeviceCount(void)
{
    return i2cRegisteredDeviceCount;
}
