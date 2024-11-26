#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"

// Access routines where the register is accessed directly
bool busRawWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    return busWriteRegister(dev, reg, data);
}

bool busRawWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    return busWriteRegisterStart(dev, reg, data);
}

bool busRawReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    return busReadRegisterBuffer(dev, reg, data, length);
}

bool busRawReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    return busReadRegisterBufferStart(dev, reg, data, length);
}

// Write routines where the register is masked with 0x7f
bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    switch (dev->bus->busType) {
        case BUS_TYPE_I2C:
            return i2cBusWriteRegister(dev, reg, data);

        default:
            return false;
    }
}

bool busWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    switch (dev->bus->busType) {
        case BUS_TYPE_I2C:
            return i2cBusWriteRegisterStart(dev, reg, data);

        default:
            return false;
    }
}

bool busWriteBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    switch (dev->bus->busType) {
        case BUS_TYPE_I2C:
            return i2cBusWriteBufferStart(dev, reg, data, length);

        default:
            return false;
    }
}

// Read routines where the register is ORed with 0x80
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    switch (dev->bus->busType) {
        case BUS_TYPE_I2C:
            return i2cBusReadRegisterBuffer(dev, reg, data, length);

        default:
            return false;
    }
}

// Start the I2C read, but do not wait for completion
bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    switch (dev->bus->busType) {
        case BUS_TYPE_I2C:
            // Initiate the read access
            return i2cBusReadRegisterBufferStart(dev, reg, data, length);

        default:
            return false;
    }
}

// Returns true if bus is still busy
bool busBusy(const extDevice_t *dev, bool *error)
{
    switch (dev->bus->busType) {
        case BUS_TYPE_I2C:
            return i2cBusBusy(dev, error);

        default:
            return false;
    }
}

uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg)
{
    uint8_t data;
    busReadRegisterBuffer(dev, reg, &data, 1);
    return data;
}

void busDeviceRegister(const extDevice_t *dev)
{
    switch (dev->bus->busType) {
        case BUS_TYPE_I2C:
            i2cBusDeviceRegister(dev);

            break;

        default:
            break;
    }
}
