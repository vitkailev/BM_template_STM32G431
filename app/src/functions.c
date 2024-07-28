#include <string.h>

#include "stm32g4xx_hal.h"

#include "functions.h"

void changePinState(PortDef *port, bool state) {
    port->state = state;
    HAL_GPIO_WritePin((GPIO_TypeDef *) port->obj, port->pin, state);
}

bool isPinTriggered(const PortDef *port) {
    return port->isTriggered;
}

bool getPinState(const PortDef *port) {
    return port->state;
}

void checkPinState(PortDef *port) {
    bool isSet = HAL_GPIO_ReadPin((GPIO_TypeDef *) port->obj, port->pin);
    if (isSet) {
        port->delay++;
    } else {
        port->delay = 0;

        // if you need to control the moment of release
//        port->isTriggered = port->state;

        port->state = false;
    }

    if (port->delay == port->duration) {
        port->isTriggered = !port->state;
        port->state = true;
    }
}

bool isADCFinished(const ADCDef *adc) {
    return adc->isFinished;
}

int readAnalogValues(ADCDef *adc) {
    if (adc->isProcessing)
        return HAL_BUSY;

    adc->isProcessing = true;
    adc->errType = 0;
    adc->idx = 0;
    return HAL_ADC_Start_IT((ADC_HandleTypeDef *) adc->obj);
}

int setDACOutput(DACDef *dac, uint8_t channel, uint32_t voltage) {
    if (channel != DAC_CHANNEL_1 && channel != DAC_CHANNEL_2)
        return -1;
    if (voltage > 3000)
        return -2;

    voltage *= 4095;
    dac->value = voltage / 3000;
    HAL_DAC_SetValue((DAC_HandleTypeDef *) dac->obj, channel, DAC_ALIGN_12B_R, dac->value);
    return HAL_DAC_Start((DAC_HandleTypeDef *) dac->obj, channel);
}

// Datasheet, DS12589 Rev. 6, Analog-to-digital converter (ADC), Internal voltage reference (Vrefint), page 30
uint32_t getRealVrefint(void) {
    const uint16_t *calValue = (const uint16_t *) 0x1FFF75AA;
    uint32_t result = *calValue;
    result *= 3000;
    result /= 4095;
    return result;
}

const void *getUniqueID(void) {
    // RM0440 Reference manual, 48.1 Unique device ID, page 2108
    return (const void *) 0x1FFF7590;
}

uint32_t getCRC(const void *data, uint32_t size) {
    // https://crccalc.com/
    // CRC-32

    uint32_t result = HAL_CRC_Calculate(Mcu.handlers.crc, (uint32_t *) data, size);
    HAL_CRC_StateTypeDef state = HAL_CRC_GetState(Mcu.handlers.crc);
    return (state == HAL_CRC_STATE_READY) ? (result ^ 0xFFFFFFFFU) : 0;
}

uint16_t generateRandomNumbers(void *rngObj, uint32_t *dst, uint16_t n) {
    if (dst == NULL || n == 0)
        return 0;

    uint16_t i = 0;
    uint32_t value = 0;
    for (i = 0; i < n; ++i) {
        if (HAL_RNG_GenerateRandomNumber((RNG_HandleTypeDef *) rngObj, &value) == HAL_OK)
            *(dst + i) = value;
        else
            break;
    }
    return i;
}
