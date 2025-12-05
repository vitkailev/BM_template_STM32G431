#include "stm32g4xx_hal.h"

#include "functions.h"

/**
 * @brief A timer interrupt callback function
 * @param htim is the timer handle structure (HAL)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == ((TIM_HandleTypeDef *) Mcu.measTimer.handle)->Instance) {
        Mcu.measTimer.isTriggered = true;
    }
}

/**
 * @brief ADC interrupt callback function
 * @param hadc is the ADC handle structure (HAL)
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ((ADC_HandleTypeDef *) Mcu.adc.handle)->Instance) {
        uint32_t value = HAL_ADC_GetValue((ADC_HandleTypeDef *) Mcu.adc.handle);

        if (Mcu.adc.idx >= NUMBER_ADC_CHANNELS)
            Mcu.adc.idx = 0;

        Mcu.adc.rawValues[Mcu.adc.idx++] = (uint16_t) value;

        if (Mcu.adc.idx == NUMBER_ADC_CHANNELS) {
            Mcu.adc.isProcessing = false;
            Mcu.adc.isFinished = true;
        } else
            HAL_ADC_Start_IT((ADC_HandleTypeDef *) Mcu.adc.handle);
    }
}

/**
 * @brief ADC interrupt error callback function
 * @param hadc is the ADC handle structure (HAL)
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ((ADC_HandleTypeDef *) Mcu.adc.handle)->Instance) {
        Mcu.adc.isProcessing = false;
        Mcu.adc.errType = HAL_ADC_GetError((ADC_HandleTypeDef *) Mcu.adc.handle);
        Mcu.adc.errors++;
    }
}

/**
 * @brief A comparator interrupt callback function
 * @param hcomp is the comparator handle structure (HAL)
 */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
    if (hcomp->Instance == ((COMP_HandleTypeDef *) Mcu.comp.handle)->Instance) {
        Mcu.comp.isTriggered = true;
        Mcu.comp.state = (bool) HAL_COMP_GetOutputLevel((COMP_HandleTypeDef *) Mcu.comp.handle);
        Mcu.comp.errType = HAL_COMP_GetError((COMP_HandleTypeDef *) Mcu.comp.handle);
        Mcu.comp.errors += (bool) Mcu.comp.errType;
    }
}

/**
 * @brief Update the current state of the UART queue (change the "HEAD" position)
 * @param uart is the base UART data structure
 */
static void UART_txHandler(UARTDef *uart) {
    uart->isWriting = false;

    uart->head += uart->nSent;
    if (uart->head == (uart->queue + UART_QUEUE_SIZE))
        uart->head = uart->queue;
}

/**
 * @brief UART data transfer callback function
 * @param huart is the UART handle structure (HAL)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart.handle)->Instance) {
        UART_txHandler(&Mcu.uart);
    }
}

/**
 * @brief Saving received data in the RX buffer (byte-to-byte)
 * @param uart is the base UART data structure
 * @param time is the runtime value
 */
static void UART_rxHandler(UARTDef *uart, uint32_t time) {
    if (!uart->isReading)
        uart->size = 0;

    uart->isReading = true;
    uart->time = time;
    if (uart->size >= UART_BUFFER_SIZE)
        uart->size = 0;

    *(uart->buffer + uart->size++) = uart->rxByte;

    HAL_UART_Receive_IT((UART_HandleTypeDef *) uart->handle, &uart->rxByte, 1U);
}

/**
 * @brief UART data receive callback function
 * @param huart is the UART handle structure (HAL)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart.handle)->Instance) {
        UART_rxHandler(&Mcu.uart, HAL_GetTick());
    }
}

/**
 * @brief UART errors callback function
 * @param huart is the UART handle structure (HAL)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart.handle)->Instance) {
        Mcu.uart.isWriting = false;
        Mcu.uart.isReading = false;
        Mcu.uart.errType = HAL_UART_GetError((UART_HandleTypeDef *) Mcu.uart.handle);
        Mcu.uart.errors++;
    }
}
