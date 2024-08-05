#pragma once

typedef enum {
    MODE_RX = 1 << 0,
    MODE_TX = 1 << 1,
    MODE_RXTX = MODE_RX | MODE_TX
} portMode_e;

typedef enum {
    SERIAL_NOT_INVERTED  = 0 << 0,
    SERIAL_INVERTED      = 1 << 0,
    SERIAL_STOPBITS_1    = 0 << 1,
    SERIAL_STOPBITS_2    = 1 << 1,
    SERIAL_PARITY_NO     = 0 << 2,
    SERIAL_PARITY_EVEN   = 1 << 2,
    SERIAL_UNIDIR        = 0 << 3,
    SERIAL_BIDIR         = 1 << 3,

    /*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
    SERIAL_BIDIR_OD      = 0 << 4,
    SERIAL_BIDIR_PP      = 1 << 4,
    SERIAL_BIDIR_NOPULL  = 1 << 5, // disable pulls in BIDIR RX mode
    SERIAL_BIDIR_PP_PD   = 1 << 6, // PP mode, normall inverted, but with PullDowns

    // If this option is set then switch the TX line to input when not in use to detect it being pulled low
    SERIAL_CHECK_TX      = 1 << 7,
} portOptions_e;

typedef enum {
    TX_PIN_ACTIVE,
    TX_PIN_MONITOR,
    TX_PIN_IGNORE
} txPinState_t;

typedef void (*serialReceiveCallbackPtr)(uint16_t data, void *rxCallbackData);
typedef void (*serialIdleCallbackPtr)();

typedef struct uartPinDef_s {
    pinDef_t pin;
    uint8_t af;
} uartPinDef_t;

typedef struct uartHardware_s {
    USART_TypeDef *reg;

    uartPinDef_t rxPin;
    uartPinDef_t txPin;
    bool pinSwap;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;

    uint8_t txIrq;
    uint8_t rxIrq;

    uint8_t txPriority;
    uint8_t rxPriority;
} uartHardware_t;

typedef struct uartConfig_s {
    portMode_e mode;
    portOptions_e options;

    uint32_t baudRate;

    serialReceiveCallbackPtr rxCallback;
    void *rxCallbackData;

    serialIdleCallbackPtr idleCallback;
} uartConfig_t;

typedef struct uartPort_s {
    uartHardware_t hardware;
    uartConfig_t config;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    // All USARTs can also be used as UART, and we use them only as UART.
    UART_HandleTypeDef Handle;
    USART_TypeDef *USARTx;

    bool (* checkUsartTxOutput)(struct uartPort_s *s);

    txPinState_t txPinState;
} uartPort_t;

void uartOpen(uartPort_t *uartPort, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options);

void uartSetBaudRate(uartPort_t *uartPort, uint32_t baudRate);
void uartSetMode(uartPort_t *uartPort, portMode_e mode);
uint32_t uartTotalRxBytesWaiting(const uartPort_t *uartPort);
uint32_t uartTotalTxBytesFree(const uartPort_t *uartPort);
bool isUartTransmitBufferEmpty(const uartPort_t *uartPort);
uint8_t uartRead(uartPort_t *uartPort);
void uartWrite(uartPort_t *uartPort, uint8_t ch);

void uartWriteBufNoFlush(uartPort_t *uartPort, const uint8_t *data, int count);
void uartBeginWrite(uartPort_t *uartPort);
void uartEndWrite(uartPort_t *uartPort);

void uartWriteBuf(uartPort_t *uartPort, const uint8_t *data, int count);
void uartPrint(uartPort_t *uartPort, const char *str);
uint32_t uartGetBaudRate(uartPort_t *uartPort);
