#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/nvic.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_timing.h"
#include "drivers/bus_i2c_utils.h"

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    }
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

static void i2cSetRCC(I2CDevice device)
{
    i2cDevice_t *pDev = &i2cDevice[device];

    switch ((uint32_t)pDev->reg) {
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

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];

    const i2cHardware_t *hardware = pDev->hardware;
    pinDef_t *scl = (pinDef_t *)&pDev->scl;
    pinDef_t *sda = (pinDef_t *)&pDev->sda;

    // Enable RCC
    i2cSetRCC(device);

    i2cUnstick(scl, sda);

    // Init pins
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);

    // Init I2C peripheral

    I2C_HandleTypeDef *pHandle = &pDev->handle;

    memset(pHandle, 0x00, sizeof(*pHandle));

    pHandle->Instance = pDev->hardware->reg;

    // Compute TIMINGR value based on peripheral clock for this device instance

    // Clock sources configured in startup/stm32/system_stm32h7xx.c as:
    //   I2C123 : D2PCLK1 (rcc_pclk1 for APB1)
    //   I2C4   : D3PCLK1 (rcc_pclk4 for APB4)
    uint32_t i2cPclk = (pHandle->Instance == I2C4) ? HAL_RCCEx_GetD3PCLK1Freq() : HAL_RCC_GetPCLK1Freq();

    pHandle->Init.Timing = i2cClockTIMINGR(i2cPclk, pDev->clockSpeed, 0);

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
