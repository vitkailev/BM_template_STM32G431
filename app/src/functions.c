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

void setTimerPrescaler(TimerDef *tim, uint16_t value) {
    tim->prescaler = value;
    __HAL_TIM_SET_PRESCALER((TIM_HandleTypeDef *) tim->handler, tim->prescaler);
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
    return HAL_ADC_Start_IT((ADC_HandleTypeDef *) adc->handler);
}

int setCompThresholdLevel(CompDef *comp, uint32_t voltage) {
    return setDACOutput(&comp->dac, DAC_CHANNEL_2, voltage);
}

int setDACOutput(DACDef *dac, uint8_t channel, uint32_t voltage) {
    if (channel != DAC_CHANNEL_1 && channel != DAC_CHANNEL_2)
        return -1;
    if (voltage > VREFP)
        return -2;

    voltage *= 4095;
    dac->value = voltage / VREFP;
    HAL_DAC_SetValue((DAC_HandleTypeDef *) dac->handler, channel, DAC_ALIGN_12B_R, dac->value);
    return HAL_DAC_Start((DAC_HandleTypeDef *) dac->handler, channel);
}

int changeGeneratorMode(GeneratorDef *gen, uint8_t mode) {
    if (mode >= NUMBER_GEN_MODES)
        return -1;

    gen->mode = mode;
    uint16_t baseLevel = 0; // mV
    uint32_t chNum = DAC_CHANNEL_2;
    DAC_HandleTypeDef *dacHandler = (DAC_HandleTypeDef *) gen->dac.handler;
    HAL_TIM_Base_Stop((TIM_HandleTypeDef *) gen->timer_1.handler);
    HAL_TIM_Base_Stop((TIM_HandleTypeDef *) gen->timer_2.handler);
    HAL_DAC_Stop(dacHandler, chNum);

    switch (gen->mode) {
        case GENERATE_TRIANGLE: {
            baseLevel = 0;
            uint32_t amplitude = DAC_TRIANGLEAMPLITUDE_4095;
            gen->dac.channel.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
            gen->dac.channel.DAC_Trigger2 = DAC_TRIGGER_NONE;
            if (HAL_DAC_ConfigChannel(dacHandler, &gen->dac.channel, chNum) != HAL_OK) {

            } else if (HAL_DACEx_TriangleWaveGenerate(dacHandler, chNum, amplitude) != HAL_OK) {

            } else {
                setDACOutput(&gen->dac, chNum, baseLevel);
                HAL_TIM_Base_Start((TIM_HandleTypeDef *) gen->timer_1.handler);
            }
            break;
        }
        case GENERATE_NOISE: {
            baseLevel = 1000;
            uint32_t amplitude = DAC_LFSRUNMASK_BITS10_0;
            gen->dac.channel.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
            gen->dac.channel.DAC_Trigger2 = DAC_TRIGGER_NONE;
            if (HAL_DAC_ConfigChannel(dacHandler, &gen->dac.channel, chNum) != HAL_OK) {

            } else if (HAL_DACEx_NoiseWaveGenerate(dacHandler, chNum, amplitude) != HAL_OK) {

            } else {
                setDACOutput(&gen->dac, chNum, baseLevel);
                HAL_TIM_Base_Start((TIM_HandleTypeDef *) gen->timer_1.handler);
            }
            break;
        }
        case GENERATE_SAWTOOTH: {
            uint32_t direction = DAC_SAWTOOTH_POLARITY_INCREMENT;
            uint16_t resetValue = 0;// 0x0FFF - DAC_SAWTOOTH_POLARITY_DECREMENT
            // 1 step == 0.0625 -> 32 * 0.0625 = 2.0 -> you need to generate 2048 events of the T6 to achieve VREFP
            uint16_t step = 32;
            gen->dac.channel.DAC_Trigger = DAC_TRIGGER_T7_TRGO; // reset trigger
            gen->dac.channel.DAC_Trigger2 = DAC_TRIGGER_T6_TRGO; // step trigger
            if (HAL_DAC_ConfigChannel(dacHandler, &gen->dac.channel, chNum) != HAL_OK) {

            } else if (HAL_DACEx_SawtoothWaveGenerate(dacHandler, chNum, direction, resetValue, step) != HAL_OK) {

            } else {
                setDACOutput(&gen->dac, chNum, baseLevel);
                HAL_TIM_Base_Start((TIM_HandleTypeDef *) gen->timer_1.handler);
                HAL_TIM_Base_Start((TIM_HandleTypeDef *) gen->timer_2.handler);
            }
            break;
        }
        default:
            break;
    }

    return 0;
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
