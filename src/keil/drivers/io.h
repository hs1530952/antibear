#pragma once

#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)

// Expose these for EXTIConfig
#define IO_CONFIG_GET_MODE(cfg) (((cfg) >> 0) & 0x03)
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_OTYPE(cfg) (((cfg) >> 4) & 0x01)
#define IO_CONFIG_GET_PULL(cfg) (((cfg) >> 5) & 0x03)

typedef uint8_t ioConfig_t;

typedef struct pinDef_s {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
} pinDef_t;

bool IORead(pinDef_t *io);
void IOWrite(pinDef_t *io, bool value);
void IOHi(pinDef_t *io);
void IOLo(pinDef_t *io);
void IOToggle(pinDef_t *io);

void IOConfigGPIO(pinDef_t *io, ioConfig_t cfg);
void IOConfigGPIOAF(pinDef_t *io, ioConfig_t cfg, uint8_t af);