#pragma once

#include "platform.h"

#include "drivers/io.h"

typedef enum I2CDevice {
    I2CINVALID = -1,
    I2CDEV_1   = 0,
    I2CDEV_2,
} I2CDevice;

#define I2CDEV_COUNT 2

// Macros to convert between CLI bus number and I2CDevice.
#define I2C_CFG_TO_DEV(x)   ((x) - 1)
#define I2C_DEV_TO_CFG(x)   ((x) + 1)

// I2C device address range in 7-bit address mode
#define I2C_ADDR7_MIN       8
#define I2C_ADDR7_MAX       119

struct i2cConfig_s;
void i2cHardwareConfigure(void);
void i2cInit(I2CDevice device);
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *data);
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *buf);
bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *buf);
bool i2cBusy(I2CDevice device, bool *error);

uint16_t i2cGetErrorCounter(void);
uint8_t i2cGetRegisteredDeviceCount(void);
