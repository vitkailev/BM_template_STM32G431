#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

enum UART_Constants {
    UART_BUFFER_SIZE = 256U,
    UART_QUEUE_SIZE = 1024U,

    UART_TIMEDELAY_AFTER_LAST_SYMBOL = 5U // msec.
};

enum UART_Errors {
    UART_SUCCESS,
    UART_NOT_INIT = -1,
    UART_WRONG_DATA = -2,
    UART_QUEUE_FULL = -3,
    UART_NOT_ALL_DATA = -4
};

typedef struct {
    bool isInit;
    bool isHaveData;
    volatile bool isReading;
    volatile bool isWriting;
    volatile uint32_t errType;
    volatile uint32_t errors;

    // rx
    volatile uint32_t time; // msec.
    volatile uint16_t size; // bytes
    uint8_t rxByte; // bytes
    uint8_t buffer[UART_BUFFER_SIZE];

    // tx
    uint16_t nSent; // bytes
    uint8_t *volatile head;
    uint8_t *tail;
    uint8_t queue[UART_QUEUE_SIZE];

    void *handle;
} UARTDef;

int UART_init(UARTDef *uart);

bool UART_isHaveData(const UARTDef *uart);

int UART_writeData(UARTDef *uart, const void *data, uint16_t size);

void UART_update(UARTDef *uart, uint32_t counter);

#ifdef __cplusplus
}
#endif

#endif // UART_H
