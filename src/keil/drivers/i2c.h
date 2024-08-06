#pragma once

typedef enum I2CDevice {
    I2CDEV_1 = 0,
    I2CDEV_COUNT,
} I2CDevice;

// I2C device address range in 7-bit address mode
#define I2C_ADDR7_MIN       8
#define I2C_ADDR7_MAX       119

void i2cInit(void);
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *data);
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *buf);
bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *buf);
bool i2cBusy(I2CDevice device, bool *error);

uint16_t i2cGetErrorCounter(void);