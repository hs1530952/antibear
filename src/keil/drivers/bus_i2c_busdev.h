#pragma once

bool i2cBusWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool i2cBusWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool i2cBusWriteBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
bool i2cBusReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
uint8_t i2cBusReadRegister(const extDevice_t *dev, uint8_t reg);
bool i2cBusReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
bool i2cBusBusy(const extDevice_t *dev, bool *error);
// Associate a device with an I2C bus
bool i2cBusSetInstance(const extDevice_t *dev, uint32_t device);
void i2cBusDeviceRegister(const extDevice_t *dev);
