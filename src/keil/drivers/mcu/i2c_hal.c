#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/nvic.h"

#include "drivers/i2c.h"
#include "drivers/i2c_impl.h"
#include "drivers/i2c_timing.h"
#include "drivers/i2c_utils.h"

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
    {
        .device = I2CDEV_1,
        .reg = I2C1,

        .sclPin = {{GPIOB, GPIO_PIN_8}, GPIO_AF4_I2C1},
        .sdaPin = {{GPIOB, GPIO_PIN_9}, GPIO_AF4_I2C1},
        .pullUp = true,

        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
};

i2cPort_t i2cPort[I2CDEV_COUNT];

static void i2cSetRCC(I2CDevice device)
{
    i2cPort_t *pPort = &i2cPort[device];

    switch ((uint32_t)pPort->I2Cx) {
        case (uint32_t)I2C1: {
            __HAL_RCC_I2C1_CLK_ENABLE();
        } break;

        case (uint32_t)I2C2: {
            __HAL_RCC_I2C2_CLK_ENABLE();
        } break;

        case (uint32_t)I2C3: {
            __HAL_RCC_I2C3_CLK_ENABLE();
        } break;

        case (uint32_t)I2C4: {
            __HAL_RCC_I2C4_CLK_ENABLE();
        } break;
    }
}

static void i2cHardwareConfigure(void)
{
    for (uint8_t index = 0; index < I2CDEV_COUNT; index++) {
        const i2cHardware_t *hardware = &i2cHardware[index];
        i2cPort_t *pPort = &i2cPort[index];

        memset(pPort, 0x00, sizeof(*pPort));

        pPort->hardware = hardware;
        pPort->I2Cx = hardware->reg;
        pPort->clockSpeed = I2C_DEFAULT_CLOCK_SPEED;
    }
}

void i2cInit(void)
{
    i2cHardwareConfigure();

    for (uint8_t index = 0; index < I2CDEV_COUNT; index++) {
        i2cPort_t *pPort = &i2cPort[index];

        const i2cHardware_t *hardware = pPort->hardware;
        pinDef_t *scl = (pinDef_t *)&hardware->sclPin.pin;
        pinDef_t *sda = (pinDef_t *)&hardware->sdaPin.pin;

        // Enable RCC
        i2cSetRCC(index);

        i2cUnstick(scl, sda);

        // Init pins
        IOConfigGPIOAF(scl, hardware->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, hardware->sclPin.af);
        IOConfigGPIOAF(sda, hardware->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, hardware->sdaPin.af);

        // Init I2C peripheral
        I2C_HandleTypeDef *pHandle = &pPort->Handle;
        memset(pHandle, 0x00, sizeof(*pHandle));
        pHandle->Instance = pPort->hardware->reg;

        // Compute TIMINGR value based on peripheral clock for this device instance
        uint32_t i2cPclk;
        // Clock sources configured in startup/stm32/system_stm32h7xx.c as:
        //   I2C123 : D2PCLK1 (rcc_pclk1 for APB1)
        //   I2C4   : D3PCLK1 (rcc_pclk4 for APB4)
        i2cPclk = (pHandle->Instance == I2C4) ? HAL_RCCEx_GetD3PCLK1Freq() : HAL_RCC_GetPCLK1Freq();

        pHandle->Init.Timing = i2cClockTIMINGR(i2cPclk, pPort->clockSpeed, 0);

        pHandle->Init.OwnAddress1 = 0x00;
        pHandle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        pHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        pHandle->Init.OwnAddress2 = 0x00;
        pHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        pHandle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

        HAL_I2C_Init(pHandle);

        // Enable the Analog I2C Filter
        HAL_I2CEx_ConfigAnalogFilter(pHandle, I2C_ANALOGFILTER_ENABLE);

        // Setup interrupt handlers
        HAL_NVIC_SetPriority(hardware->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
        HAL_NVIC_EnableIRQ(hardware->er_irq);
        HAL_NVIC_SetPriority(hardware->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
        HAL_NVIC_EnableIRQ(hardware->ev_irq);
    }
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cPort[I2CDEV_1].Handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cPort[I2CDEV_1].Handle);
}

static volatile uint16_t i2cErrorCount = 0;

static bool i2cHandleHardwareFailure(I2CDevice device)
{
    (void)device;
    i2cErrorCount++;
    return false;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

// Blocking write
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    if (device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cPort[device].Handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Transmit(pHandle, addr_ << 1, &data, 1, I2C_TIMEOUT_SYS_TICKS);
    else
        status = HAL_I2C_Mem_Write(pHandle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_SYS_TICKS);

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

// Non-blocking write
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *data)
{
    if (device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cPort[device].Handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write_IT(pHandle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT, data, len_);

    if (status == HAL_BUSY) {
        return false;
    }

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

// Blocking read
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *buf)
{
    if (device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cPort[device].Handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Receive(pHandle, addr_ << 1, buf, len_, I2C_TIMEOUT_SYS_TICKS);
    else
        status = HAL_I2C_Mem_Read(pHandle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT, buf, len_, I2C_TIMEOUT_SYS_TICKS);

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

// Non-blocking read
bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint16_t len_, uint8_t *buf)
{
    if (device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cPort[device].Handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read_IT(pHandle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT, buf, len_);

    if (status == HAL_BUSY) {
        return false;
    }

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
    I2C_HandleTypeDef *pHandle = &i2cPort[device].Handle;

    if (error) {
        *error = pHandle->ErrorCode;
    }

    if (pHandle->State == HAL_I2C_STATE_READY)
    {
        if (__HAL_I2C_GET_FLAG(pHandle, I2C_FLAG_BUSY) == SET)
        {
            return true;
        }

        return false;
    }

    return true;
}