#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/uart.h"
#include "drivers/uart_impl.h"

void uartOpen(uartPort_t *uartPort, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    if (!uartPort)
        return;

    uartPreInit(uartPort, baudRate, mode, options);

    uartPort->rxBufferHead = uartPort->rxBufferTail = 0;
    uartPort->txBufferHead = uartPort->txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    uartPort->config.rxCallback = rxCallback;
    uartPort->config.rxCallbackData = rxCallbackData;
    uartPort->config.mode = mode;
    uartPort->config.baudRate = baudRate;
    uartPort->config.options = options;

    uartReconfigure(uartPort);
}

void uartSetBaudRate(uartPort_t *uartPort, uint32_t baudRate)
{
    uartPort->config.baudRate = baudRate;
    uartReconfigure(uartPort);
}

void uartSetMode(uartPort_t *uartPort, portMode_e mode)
{
    uartPort->config.mode = mode;
    uartReconfigure(uartPort);
}

uint32_t uartTotalRxBytesWaiting(const uartPort_t *uartPort)
{
    if (uartPort->rxBufferHead >= uartPort->rxBufferTail) {
        return uartPort->rxBufferHead - uartPort->rxBufferTail;
    } else {
        return uartPort->rxBufferSize + uartPort->rxBufferHead - uartPort->rxBufferTail;
    }
}

uint32_t uartTotalTxBytesFree(const uartPort_t *uartPort)
{
    uint32_t bytesUsed;

    if (uartPort->txBufferHead >= uartPort->txBufferTail) {
        bytesUsed = uartPort->txBufferHead - uartPort->txBufferTail;
    } else {
        bytesUsed = uartPort->txBufferSize + uartPort->txBufferHead - uartPort->txBufferTail;
    }

    return (uartPort->txBufferSize - 1) - bytesUsed;
}

bool isUartTransmitBufferEmpty(const uartPort_t *uartPort)
{
    return uartPort->txBufferTail == uartPort->txBufferHead;
}

uint8_t uartRead(uartPort_t *uartPort)
{
    uint8_t ch;

    ch = uartPort->rxBuffer[uartPort->rxBufferTail];
    if (uartPort->rxBufferTail + 1 >= uartPort->rxBufferSize) {
        uartPort->rxBufferTail = 0;
    } else {
        uartPort->rxBufferTail++;
    }

    return ch;
}

void uartWrite(uartPort_t *uartPort, uint8_t ch)
{
    // Check if the TX line is being pulled low by an unpowered peripheral
    if (uartPort->checkUsartTxOutput && !uartPort->checkUsartTxOutput(uartPort)) {
        // TX line is being pulled low, so don't transmit
        return;
    }

    uartPort->txBuffer[uartPort->txBufferHead] = ch;

    if (uartPort->txBufferHead + 1 >= uartPort->txBufferSize) {
        uartPort->txBufferHead = 0;
    } else {
        uartPort->txBufferHead++;
    }

    __HAL_UART_ENABLE_IT(&uartPort->Handle, UART_IT_TXE);
}

void uartBeginWrite(uartPort_t *uartPort)
{
    // Check if the TX line is being pulled low by an unpowered peripheral
    if (uartPort->checkUsartTxOutput) {
        uartPort->checkUsartTxOutput(uartPort);
    }
}

void uartWriteBufNoFlush(uartPort_t *uartPort, const uint8_t *data, int count)
{
    const uint8_t *bytePtr = (const uint8_t*)data;

    // Test if checkUsartTxOutput() detected TX line being pulled low by an unpowered peripheral
    if (uartPort->txPinState == TX_PIN_MONITOR) {
        // TX line is being pulled low, so don't transmit
        return;
    }

    while (count > 0) {
        // Calculate the available space to the end of the buffer
        const int spaceToEnd = uartPort->txBufferSize - uartPort->txBufferHead;
        // Determine the amount to copy in this iteration
        const int chunkSize = MIN(spaceToEnd, count);
        // Copy the chunk
        memcpy((void *)&uartPort->txBuffer[uartPort->txBufferHead], bytePtr, chunkSize);
        // Advance source pointer
        bytePtr += chunkSize;
        // Advance head, wrapping if necessary
        uartPort->txBufferHead = (uartPort->txBufferHead + chunkSize) % uartPort->txBufferSize;
        // Decrease remaining count
        count -= chunkSize;
    }
}

void uartEndWrite(uartPort_t *uartPort)
{
    // Check if the TX line is being pulled low by an unpowered peripheral
    if (uartPort->txPinState == TX_PIN_MONITOR) {
        // TX line is being pulled low, so don't transmit
        return;
    }

    __HAL_UART_ENABLE_IT(&uartPort->Handle, UART_IT_TXE);
}

void uartWriteBuf(uartPort_t *uartPort, const uint8_t *data, int count)
{
    uartBeginWrite(uartPort);
    uartWriteBufNoFlush(uartPort, data, count);
    uartEndWrite(uartPort);
}

void uartPrint(uartPort_t *uartPort, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        uartWrite(uartPort, ch);
    }
}

uint32_t uartGetBaudRate(uartPort_t *uartPort)
{
    return uartPort->config.baudRate;
}