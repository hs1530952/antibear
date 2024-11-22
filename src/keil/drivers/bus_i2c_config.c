#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/io.h"

typedef struct i2cConfig_s {
    pinDef_t sclPin;
    uint8_t sclAF;
    pinDef_t sdaPin;
    uint8_t sdaAF;
    bool pullUp;
    uint16_t clockSpeed;
} i2cConfig_t;

const i2cConfig_t i2cConfig[I2CDEV_COUNT] = {
    {
        .sclPin = {GPIOB, GPIO_PIN_8},
        .sclAF = GPIO_AF4_I2C1,
        .sdaPin = {GPIOB, GPIO_PIN_9},
        .sdaAF = GPIO_AF4_I2C1,
        .pullUp = true,
        .clockSpeed = 800,
    },
    {
        .sclPin = {GPIOH, GPIO_PIN_4},
        .sclAF = GPIO_AF4_I2C2,
        .sdaPin = {GPIOH, GPIO_PIN_4},
        .sdaAF = GPIO_AF4_I2C2,
        .pullUp = true,
        .clockSpeed = 800,
    }
};

void i2cHardwareConfigure(void)
{
    for (int index = 0; index < I2CDEV_COUNT; index++) {
        const i2cHardware_t *hardware = &i2cHardware[index];

        if (!hardware->reg) {
            continue;
        }

        I2CDevice device = hardware->device;
        i2cDevice_t *pDev = &i2cDevice[device];

        memset(pDev, 0x00, sizeof(*pDev));

        pDev->scl = i2cConfig[device].sclPin;
        pDev->sclAF = i2cConfig[device].sclAF;
        pDev->sda = i2cConfig[device].sdaPin;
        pDev->sdaAF = i2cConfig[device].sdaAF;

        pDev->hardware = hardware;
        pDev->reg = hardware->reg;
        pDev->pullUp = i2cConfig[device].pullUp;
        pDev->clockSpeed = i2cConfig[device].clockSpeed;
    }
}
