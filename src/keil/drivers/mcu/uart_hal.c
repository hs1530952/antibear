#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/uart.h"
#include "drivers/uart_impl.h"

static void uartConfigurePinInversion(uartPort_t *uartPort)
{
    bool inverted = uartPort->config.options & SERIAL_INVERTED;

    if (inverted)
    {
        if (uartPort->config.mode & MODE_RX)
        {
            uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
            uartPort->Handle.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
        }
        if (uartPort->config.mode & MODE_TX)
        {
            uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_TXINVERT_INIT;
            uartPort->Handle.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
        }
    }
}

static void uartConfigurePinSwap(uartPort_t *uartPort)
{
    if (uartPort->hardware.pinSwap) {
        uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_SWAP_INIT;
        uartPort->Handle.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    }
}

void uartReconfigure(uartPort_t *uartPort)
{
    HAL_UART_DeInit(&uartPort->Handle);
    uartPort->Handle.Init.BaudRate = uartPort->config.baudRate;
    uartPort->Handle.Init.WordLength = (uartPort->config.options & SERIAL_PARITY_EVEN) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    uartPort->Handle.Init.StopBits = (uartPort->config.options & SERIAL_STOPBITS_2) ? UART_STOPBITS_2 : UART_STOPBITS_1;
    uartPort->Handle.Init.Parity = (uartPort->config.options & SERIAL_PARITY_EVEN) ? UART_PARITY_EVEN : UART_PARITY_NONE;
    uartPort->Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartPort->Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uartPort->Handle.Init.Mode = 0;
    uartPort->Handle.Init.ClockPrescaler = UART_PRESCALER_DIV8;

    if (uartPort->config.mode & MODE_RX)
        uartPort->Handle.Init.Mode |= UART_MODE_RX;
    if (uartPort->config.mode & MODE_TX)
        uartPort->Handle.Init.Mode |= UART_MODE_TX;

    uartConfigurePinInversion(uartPort);
    uartConfigurePinSwap(uartPort);

    if (uartPort->config.mode & SERIAL_BIDIR) {
        HAL_HalfDuplex_Init(&uartPort->Handle);
    } else {
        HAL_UART_Init(&uartPort->Handle);
    }

    if (uartPort->config.mode & MODE_RX)
    {
        /* Enable the UART Parity Error Interrupt */
        SET_BIT(uartPort->USARTx->CR1, USART_CR1_PEIE);

        /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        SET_BIT(uartPort->USARTx->CR3, USART_CR3_EIE);

        /* Enable the UART Data Register not empty Interrupt */
        SET_BIT(uartPort->USARTx->CR1, USART_CR1_RXNEIE);

        /* Enable Idle Line detection */
        SET_BIT(uartPort->USARTx->CR1, USART_CR1_IDLEIE);
    }

    if (uartPort->config.mode & MODE_TX)
    {
        /* Enable the UART Transmit Data Register Empty Interrupt */
        SET_BIT(uartPort->USARTx->CR1, USART_CR1_TXEIE);
        SET_BIT(uartPort->USARTx->CR1, USART_CR1_TCIE);
    }
}

bool checkUsartTxOutput(uartPort_t *uartPort)
{
    pinDef_t *txIO = &uartPort->hardware.txPin.pin;

    if ((uartPort->txPinState == TX_PIN_MONITOR) && txIO) {
        if (IORead(txIO)) {
            // TX is high so we're good to transmit

            // Enable USART TX output
            uartPort->txPinState = TX_PIN_ACTIVE;
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uartPort->hardware.txPin.af);

            // Enable the UART transmitter
            SET_BIT(uartPort->Handle.Instance->CR1, USART_CR1_TE);

            return true;
        } else {
            // TX line is pulled low so don't enable USART TX
            return false;
        }
    }

    return true;
}

void uartTxMonitor(uartPort_t *uartPort)
{
    if (uartPort->txPinState == TX_PIN_ACTIVE) {
        // Disable the UART transmitter
        CLEAR_BIT(uartPort->Handle.Instance->CR1, USART_CR1_TE);

        // Switch TX to an input with pullup so it's state can be monitored
        uartPort->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(&uartPort->hardware.txPin.pin, IOCFG_IPU);
    }
}

FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *uartPort)
{
    UART_HandleTypeDef *huart = &uartPort->Handle;
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET)) {
        uint8_t rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t)0xff);

        if (uartPort->config.rxCallback) {
            uartPort->config.rxCallback(rbyte, uartPort->config.rxCallbackData);
        } else {
            uartPort->rxBuffer[uartPort->rxBufferHead] = rbyte;
            uartPort->rxBufferHead = (uartPort->rxBufferHead + 1) % uartPort->rxBufferSize;
        }
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }

    /* UART parity error interrupt occurred -------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    // UART transmission completed
    if ((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_TCF);

        // Switch TX to an input with pull-up so it's state can be monitored
        uartTxMonitor(uartPort);
    }

    if ((__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET)) {
        /* Check that a Tx process is ongoing */
        if (uartPort->txBufferTail == uartPort->txBufferHead) {
            /* Disable the UART Transmit Data Register Empty Interrupt */
            CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
        } else {
            if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE)) {
                huart->Instance->TDR = (((uint16_t) uartPort->txBuffer[uartPort->txBufferTail]) & (uint16_t)0x01FFU);
            } else {
                huart->Instance->TDR = (uint8_t)(uartPort->txBuffer[uartPort->txBufferTail]);
            }
            uartPort->txBufferTail = (uartPort->txBufferTail + 1) % uartPort->txBufferSize;
        }
    }

    // UART reception idle detected

    if (__HAL_UART_GET_IT(huart, UART_IT_IDLE)) {
        if (uartPort->config.idleCallback) {
            uartPort->config.idleCallback();
        }

        __HAL_UART_CLEAR_IDLEFLAG(huart);
    }

}

static void uartSetRCC(uartPort_t *uartPort)
{
    if (!uartPort) {
        return;
    }

    switch ((uint32_t)uartPort->USARTx) {
        case (uint32_t)USART1: {
            __HAL_RCC_USART1_CLK_ENABLE();
        } break;

        case (uint32_t)USART2: {
            __HAL_RCC_USART2_CLK_ENABLE();
        } break;

        case (uint32_t)USART3: {
            __HAL_RCC_USART3_CLK_ENABLE();
        } break;

        case (uint32_t)UART4: {
            __HAL_RCC_UART4_CLK_ENABLE();
        } break;

        case (uint32_t)UART5: {
            __HAL_RCC_UART5_CLK_ENABLE();
        } break;

        case (uint32_t)USART6: {
            __HAL_RCC_USART6_CLK_ENABLE();
        } break;

        case (uint32_t)UART7: {
            __HAL_RCC_UART7_CLK_ENABLE();
        } break;

        case (uint32_t)UART8: {
            __HAL_RCC_UART8_CLK_ENABLE();
        } break;

        case (uint32_t)LPUART1: {
            __HAL_RCC_LPUART1_CLK_ENABLE();
        } break;
    }
}

void uartPreInit(uartPort_t *uartPort, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    if (!uartPort) {
        return;
    }

    uartPort->config.baudRate = baudRate;

    uartPort->USARTx = uartPort->hardware.reg;

    uartPort->rxBuffer = uartPort->hardware.rxBuffer;
    uartPort->txBuffer = uartPort->hardware.txBuffer;
    uartPort->rxBufferSize = uartPort->hardware.rxBufferSize;
    uartPort->txBufferSize = uartPort->hardware.txBufferSize;

    uartPort->checkUsartTxOutput = checkUsartTxOutput;

    uartPort->Handle.Instance = uartPort->hardware.reg;

    uartSetRCC(uartPort);

    pinDef_t *txIO = &uartPort->hardware.txPin.pin;
    pinDef_t *rxIO = &uartPort->hardware.rxPin.pin;

    uartPort->txPinState = TX_PIN_IGNORE;

    if ((options & SERIAL_BIDIR) && txIO) {
        ioConfig_t ioCfg = IO_CONFIG(
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP) || (options & SERIAL_BIDIR_PP_PD)) ? GPIO_MODE_AF_PP : GPIO_MODE_AF_OD,
            GPIO_SPEED_FREQ_HIGH,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP_PD)) ? GPIO_PULLDOWN : GPIO_PULLUP
        );
        IOConfigGPIOAF(txIO, ioCfg, uartPort->hardware.rxPin.af);
    } else {
        if ((mode & MODE_TX) && txIO) {
            if (options & SERIAL_CHECK_TX) {
                uartPort->txPinState = TX_PIN_ACTIVE;
                // Switch TX to an input with pullup so it's state can be monitored
                uartTxMonitor(uartPort);
            } else {
                IOConfigGPIOAF(txIO, IOCFG_AF_PP, uartPort->hardware.rxPin.af);
            }
        }

        if ((mode & MODE_RX) && rxIO) {
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP,uartPort->hardware.rxPin.af);
        }
    }

    HAL_NVIC_SetPriority(uartPort->hardware.rxIrq, NVIC_PRIORITY_BASE(uartPort->hardware.rxPriority), NVIC_PRIORITY_SUB(uartPort->hardware.rxPriority));
    HAL_NVIC_EnableIRQ(uartPort->hardware.rxIrq);
}