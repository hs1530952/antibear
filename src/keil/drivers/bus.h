#pragma once

#include "platform.h"

#include "drivers/bus_i2c.h"

typedef enum {
    BUS_TYPE_NONE = 0,
    BUS_TYPE_I2C,
} busType_e;

typedef enum {
    BUS_READY,
    BUS_BUSY,
    BUS_ABORT
} busStatus_e;

struct extDevice_s;

// Bus interface, independent of connected device
typedef struct busDevice_s {
    busType_e busType;
    union {
        struct busI2C_s {
            I2CDevice device;
        } i2c;
    } busType_u;
} busDevice_t;

typedef struct extDevice_s {
    busDevice_t *bus;
    union {
        struct extI2C_s {
            uint8_t address;
        } i2c;
    } busType_u;
} extDevice_t;

// Access routines where the register is accessed directly
bool busRawWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busRawWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busRawReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
bool busRawReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
// Write routines where the register is masked with 0x7f
bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busWriteBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
// Read routines where the register is ORed with 0x80
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg);

bool busBusy(const extDevice_t *dev, bool *error);
void busDeviceRegister(const extDevice_t *dev);
