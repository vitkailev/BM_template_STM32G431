#include "stm32g4xx_hal.h"

#include "variables.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == ((TIM_HandleTypeDef *) Mcu.timer_8kHz.obj)->Instance) {
        Mcu.timer_8kHz.isTriggered = true;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ((ADC_HandleTypeDef *) Mcu.adc.obj)->Instance) {
        uint32_t value = HAL_ADC_GetValue((ADC_HandleTypeDef *) Mcu.adc.obj);

        if (Mcu.adc.idx >= NUMBER_ADC_CHANNELS)
            Mcu.adc.idx = 0;

        Mcu.adc.rawValues[Mcu.adc.idx++] = (uint16_t) value;

        if (Mcu.adc.idx == NUMBER_ADC_CHANNELS) {
            Mcu.adc.isProcessing = false;
            Mcu.adc.isFinished = true;
        } else
            HAL_ADC_Start_IT((ADC_HandleTypeDef *) Mcu.adc.obj);
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ((ADC_HandleTypeDef *) Mcu.adc.obj)->Instance) {
        Mcu.adc.isProcessing = false;
        Mcu.adc.errType = HAL_ADC_GetError((ADC_HandleTypeDef *) Mcu.adc.obj);
        Mcu.adc.errors++;
    }
}

static void UART_txHandler(UARTDef *uart) {
    uart->isWriting = false;

    uart->head += uart->nSent;
    if (uart->head == (uart->queue + UART_QUEUE_SIZE))
        uart->head = uart->queue;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart1.obj)->Instance) {
        UART_txHandler(&Mcu.uart1);
    } else if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart2.obj)->Instance) {
        UART_txHandler(&Mcu.uart2);
    }
}

static void UART_rxHandler(UARTDef *uart, uint32_t time) {
    if (!uart->isReading)
        uart->size = 0;

    uart->isReading = true;
    uart->time = time;
    if (uart->size >= UART_BUFFER_SIZE)
        uart->size = 0;

    *(uart->buffer + uart->size++) = uart->rxByte;

    HAL_UART_Receive_IT((UART_HandleTypeDef *) uart->obj, &uart->rxByte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart1.obj)->Instance) {
        UART_rxHandler(&Mcu.uart1, HAL_GetTick());
    } else if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart2.obj)->Instance) {
        UART_rxHandler(&Mcu.uart2, HAL_GetTick());
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart1.obj)->Instance) {
        Mcu.uart1.isWriting = false;
        Mcu.uart1.isReading = false;
        Mcu.uart1.errType = HAL_UART_GetError((UART_HandleTypeDef *) Mcu.uart1.obj);
        Mcu.uart1.errors++;
    } else if (huart->Instance == ((UART_HandleTypeDef *) Mcu.uart2.obj)->Instance) {
        Mcu.uart2.isWriting = false;
        Mcu.uart2.isReading = false;
        Mcu.uart2.errType = HAL_UART_GetError((UART_HandleTypeDef *) Mcu.uart2.obj);
        Mcu.uart2.errors++;
    }
}
