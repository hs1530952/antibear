#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/uart.h"
#include "drivers/uart_impl.h"

#define DEBUG_SERIAL_OPTIONS    (SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_UNIDIR | SERIAL_CHECK_TX)

uartPort_t debugSerial;
DMA_RAM uint8_t rxBuffer[256];
DMA_RAM uint8_t txBuffer[256];

FAST_IRQ_HANDLER void USART1_IRQHandler(void)
{
    uartIrqHandler(&debugSerial);
}

void debugSerialInit(void)
{
    memset(&debugSerial, 0x00, sizeof(uartPort_t));

    debugSerial.hardware.reg = USART1;

    debugSerial.hardware.rxPin.pin.GPIOx = GPIOA;
    debugSerial.hardware.rxPin.pin.GPIO_Pin = GPIO_PIN_10;
    debugSerial.hardware.rxPin.af = GPIO_AF7_USART1;

    debugSerial.hardware.txPin.pin.GPIOx = GPIOA;
    debugSerial.hardware.txPin.pin.GPIO_Pin = GPIO_PIN_9;
    debugSerial.hardware.txPin.af = GPIO_AF7_USART1;

    debugSerial.hardware.pinSwap = false;

    debugSerial.hardware.rxBufferSize = 256;
    debugSerial.hardware.txBufferSize = 256;
    debugSerial.hardware.rxBuffer = rxBuffer;
    debugSerial.hardware.txBuffer = txBuffer;

    debugSerial.hardware.txIrq = USART1_IRQn;
    debugSerial.hardware.rxIrq = USART1_IRQn;

    debugSerial.hardware.txPriority = NVIC_PRIO_SERIALUART1_TXDMA;
    debugSerial.hardware.rxPriority = NVIC_PRIO_SERIALUART1;

    uartOpen(&debugSerial, NULL, NULL, 460800, MODE_RXTX, DEBUG_SERIAL_OPTIONS);
}

void debugSerialPrint(const char *str)
{
    uartPrint(&debugSerial, str);
}