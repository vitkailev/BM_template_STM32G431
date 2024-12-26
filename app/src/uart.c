#include "stm32g4xx_hal.h"

#include "uart.h"

/**
 * @brief Check, that the UART interface is initialized
 * @param uart is the base UART data structure
 * @return True - UART interface has been initialized, otherwise - False
 */
static bool isInit(const UARTDef *uart) {
    return uart->isInit;
}

/**
 * @brief Check, that the UART interface is reading a new package
 * @param uart is the base UART data structure
 * @return True - is reading, otherwise - False
 */
static bool isReading(const UARTDef *uart) {
    return uart->isReading;
}

/**
 * @brief Check, that the UART interface is sending data
 * @param uart is the base UART data structure
 * @return True - is writing, otherwise - False
 */
static bool isWriting(const UARTDef *uart) {
    return uart->isWriting;
}

/**
 * @brief Check, that the output queue is empty
 * @param uart is the base UART data structure
 * @return True - is empty, otherwise - False
 */
static bool isQueueEmpty(const UARTDef *uart) {
    return (uart->tail == uart->head);
}

/**
 * @brief Check, that the output queue is full
 * @param uart is the base UART data structure
 * @return True - is full, otherwise - False
 */
static bool isQueueFull(const UARTDef *uart) {
    if (uart->head == uart->queue && uart->tail == uart->queue + UART_QUEUE_SIZE)
        return true;
    else
        return (uart->tail + 1 == uart->head);
}

/**
 * @brief Send a new data chunk from the output queue to the UART module
 * @param uart is the base UART data structure
 */
static void sendData(UARTDef *uart) {
    size_t size = 0;
    if (uart->tail > uart->head)
        size = uart->tail - uart->head;
    else
        size = uart->queue + UART_QUEUE_SIZE - uart->head;

    uart->nSent = (uint16_t) size;

    uart->isWriting = true;
    HAL_UART_Transmit_IT((UART_HandleTypeDef *) uart->handle, uart->head, uart->nSent);
}

/**
 * @brief UART interface initialization
 * @param uart is the base UART data structure
 * @return UART_SUCCESS
 */
int UART_init(UARTDef *uart) {
    uart->head = uart->queue;
    uart->tail = uart->queue;
    uart->isInit = true;
    return UART_SUCCESS;
}

/**
 * @brief Check, that UART interface has a new package
 * @param uart is the base UART data structure
 * @return True - UART interface has received a package, otherwise - False
 */
bool UART_isHaveData(const UARTDef *uart) {
    return uart->isHaveData;
}

/**
 * @brief Put the target data to the UART interface output queue (for sending)
 * @param uart is the base UART data structure
 * @param data is the target data
 * @param size is the target data size (bytes)
 * @return UART_Errors value
 */
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

/**
 * @brief Update the UART interface current state
 * @param uart is the base UART data structure
 * @param currentTime is the runtime value (msec.)
 */
void UART_update(UARTDef *uart, uint32_t currentTime) {
    if (!isInit(uart))
        return;

    if (isReading(uart) && (currentTime - uart->time) >= UART_TIMEDELAY_AFTER_LAST_SYMBOL) {
        uart->isReading = false;
        uart->isHaveData = true;
    }

    if (!isWriting(uart) && !isQueueEmpty(uart))
        sendData(uart);
}
