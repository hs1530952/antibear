#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/uart.h"
#include "drivers/uart_impl.h"

#define DEBUG_SERIAL_OPTIONS    (SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_UNIDIR | SERIAL_CHECK_TX)

DMA_RAM uint8_t debugSerialRxBuffer[256];
DMA_RAM uint8_t debugSerialTxBuffer[256];

uartPort_t debugSerial;
const uartHardware_t debugSerialHardware = {
    .reg = USART1,
    
    .rxPin = {{GPIOA, GPIO_PIN_10}, GPIO_AF7_USART1},
    .txPin = {{GPIOA, GPIO_PIN_9}, GPIO_AF7_USART1},
    .pinSwap = false,

    .txIrq = USART1_IRQn,
    .rxIrq = USART1_IRQn,

    .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART1,

    .rxBufferSize = 256,
    .txBufferSize = 256,
    .rxBuffer = debugSerialRxBuffer,
    .txBuffer = debugSerialTxBuffer,
};

FAST_IRQ_HANDLER void USART1_IRQHandler(void)
{
    uartIrqHandler(&debugSerial);
}

void debugSerialInit(void)
{
    memset(&debugSerial, 0x00, sizeof(uartPort_t));

    debugSerial.hardware = &debugSerialHardware;

    uartOpen(&debugSerial, NULL, NULL, 460800, MODE_RXTX, DEBUG_SERIAL_OPTIONS);
}

void debugSerialPrint(const char *str)
{
    uartPrint(&debugSerial, str);
}