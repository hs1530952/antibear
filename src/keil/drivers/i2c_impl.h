#pragma once

#define I2C_TIMEOUT_US          10000
#define I2C_TIMEOUT_SYS_TICKS   (I2C_TIMEOUT_US / 1000)

#define I2C_CLOCKSPEED_MIN_KHZ  100
#define I2C_CLOCKSPEED_MAX_KHZ  1300

#define I2C_DEFAULT_CLOCK_SPEED 400

typedef struct i2cPinDef_s {
    pinDef_t pin;
    uint8_t af;
} i2cPinDef_t;

typedef struct i2cHardware_s {
    I2CDevice device;
    I2C_TypeDef *reg;

    i2cPinDef_t sclPin;
    i2cPinDef_t sdaPin;
    bool pullUp;

    uint8_t ev_irq;
    uint8_t er_irq;
} i2cHardware_t;

typedef struct i2cPort_s {
    const i2cHardware_t *hardware;
    uint16_t clockSpeed;

    I2C_HandleTypeDef Handle;
    I2C_TypeDef *I2Cx;
} i2cPort_t;