#pragma once

#include "platform.h"

#include "drivers/io.h"

#define I2C_TIMEOUT_US          10000
#define I2C_TIMEOUT_SYS_TICKS   (I2C_TIMEOUT_US / 1000)

typedef struct i2cHardware_s {
    I2CDevice device;
    I2C_TypeDef *reg;
    uint8_t ev_irq;
    uint8_t er_irq;
} i2cHardware_t;

extern const i2cHardware_t i2cHardware[];

typedef struct i2cDevice_s {
    const i2cHardware_t *hardware;
    I2C_TypeDef *reg;
    pinDef_t scl;
    pinDef_t sda;
    uint8_t sclAF;
    uint8_t sdaAF;
    bool pullUp;
    uint16_t clockSpeed;

    I2C_HandleTypeDef handle;
} i2cDevice_t;

extern i2cDevice_t i2cDevice[];
