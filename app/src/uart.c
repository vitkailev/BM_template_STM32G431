#include <stdlib.h>

#include "stm32g4xx_hal.h"

#include "uart.h"

static bool isInit(const UARTDef *uart) {
    return uart->isInit;
}

static bool isReading(const UARTDef *uart) {
    return uart->isReading;
}

static bool isWriting(const UARTDef *uart) {
    return uart->isWriting;
}

static bool isQueueEmpty(const UARTDef *uart) {
    return (uart->tail == uart->head);
}

static bool isQueueFull(const UARTDef *uart) {
    if (uart->head == uart->queue && uart->tail == uart->queue + UART_QUEUE_SIZE)
        return true;
    else
        return (uart->tail + 1 == uart->head);
}

static void sendData(UARTDef *uart) {
    size_t size = 0;
    if (uart->tail > uart->head)
        size = uart->tail - uart->head;
    else
        size = uart->queue + UART_QUEUE_SIZE - uart->head;

    uart->nSent = (uint16_t) size;

    uart->isWriting = true;
    HAL_UART_Transmit_IT((UART_HandleTypeDef *) uart->obj, uart->head, uart->nSent);
}

int UART_init(UARTDef *uart) {
    uart->head = uart->queue;
    uart->tail = uart->queue;
    uart->isInit = true;
    return UART_SUCCESS;
}

bool UART_isHaveData(const UARTDef *uart) {
    return uart->isHaveData;
}

int UART_writeData(UARTDef *uart, const void *data, uint16_t size) {
    if (!isInit(uart))
        return UART_NOT_INIT;
    if (data == NULL || size == 0 || size > UART_QUEUE_SIZE)
        return UART_WRONG_DATA;

    if (uart->tail == (uart->queue + UART_QUEUE_SIZE) && uart->head != uart->queue)
        uart->tail = uart->queue;

    if (isQueueFull(uart))
        return UART_QUEUE_FULL;

    for (size_t i = 0; i < size; ++i) {
        *uart->tail++ = *((uint8_t *) data + i);

        if (uart->tail == (uart->queue + UART_QUEUE_SIZE) && uart->head != uart->queue)
            uart->tail = uart->queue;

        if (isQueueFull(uart))
            return UART_NOT_ALL_DATA;
    }
    return UART_SUCCESS;
}

void UART_update(UARTDef *uart, uint32_t currentTime) {
    if (!isInit(uart))
        return;

    if (isReading(uart) && abs(currentTime - uart->time) >= UART_TIMEDELAY_AFTER_LAST_SYMBOL) {
        uart->isReading = false;
        uart->isHaveData = true;
    }

    if (!isWriting(uart) && !isQueueEmpty(uart))
        sendData(uart);
}
