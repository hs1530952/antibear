#pragma once

void uartPreInit(uartPort_t *uartPort, uint32_t baudRate, portMode_e mode, portOptions_e options);

void uartIrqHandler(uartPort_t *uartPort);

void uartReconfigure(uartPort_t *uartPort);

bool checkUsartTxOutput(uartPort_t *uartPort);
void uartTxMonitor(uartPort_t *uartPort);