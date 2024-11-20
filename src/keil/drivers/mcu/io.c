#include "platform.h"

#include "drivers/io.h"

static void IOSetRCC(pinDef_t *io)
{
    if (!io) {
        return;
    }

    switch ((uint32_t)io->GPIOx) {
        case (uint32_t)GPIOA: {
            __HAL_RCC_GPIOA_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOB: {
            __HAL_RCC_GPIOB_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOC: {
            __HAL_RCC_GPIOC_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOD: {
            __HAL_RCC_GPIOD_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOE: {
            __HAL_RCC_GPIOE_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOF: {
            __HAL_RCC_GPIOF_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOG: {
            __HAL_RCC_GPIOG_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOH: {
            __HAL_RCC_GPIOH_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOI: {
            __HAL_RCC_GPIOI_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOJ: {
            __HAL_RCC_GPIOJ_CLK_ENABLE();
        } break;

        case (uint32_t)GPIOK: {
            __HAL_RCC_GPIOK_CLK_ENABLE();
        } break;
    }
}

static int IO_GPIOPinIdx(pinDef_t *io)
{
    if (!io) {
        return - 1;
    }
    return 31 - __builtin_clz(io->GPIO_Pin);
}

// mask on stm32f103, bit index on stm32f303
uint32_t IO_EXTI_Line(pinDef_t *io)
{
    if (!io) {
        return 0;
    }
    return 1 << IO_GPIOPinIdx(io);
}

bool IORead(pinDef_t *io)
{
    if (!io) {
        return false;
    }

    return !! HAL_GPIO_ReadPin(io->GPIOx, io->GPIO_Pin);
}

void IOWrite(pinDef_t *io, bool hi)
{
    if (!io) {
        return;
    }

    HAL_GPIO_WritePin(io->GPIOx, io->GPIO_Pin, hi ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void IOHi(pinDef_t *io)
{
    if (!io) {
        return;
    }

    HAL_GPIO_WritePin(io->GPIOx, io->GPIO_Pin, GPIO_PIN_SET);
}

void IOLo(pinDef_t *io)
{
    if (!io) {
        return;
    }

    HAL_GPIO_WritePin(io->GPIOx, io->GPIO_Pin, GPIO_PIN_RESET);
}

void IOToggle(pinDef_t *io)
{
    if (!io) {
        return;
    }

    HAL_GPIO_TogglePin(io->GPIOx, io->GPIO_Pin);
}

void IOConfigGPIO(pinDef_t *io, ioConfig_t cfg)
{
    IOConfigGPIOAF(io, cfg, 0);
}

void IOConfigGPIOAF(pinDef_t *io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    IOSetRCC(io);

    GPIO_InitTypeDef init = {
        .Pin = io->GPIO_Pin,
        .Mode = (cfg >> 0) & 0x13,
        .Speed = (cfg >> 2) & 0x03,
        .Pull = (cfg >> 5) & 0x03,
        .Alternate = af
    };

    HAL_GPIO_Init(io->GPIOx, &init);
}
