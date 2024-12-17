#include "stm32g4xx_hal.h"

#include "functions.h"

/**
 * @brief Set the state/level of the output pin
 * @param port is the base GPIO port data structure
 * @param state is a pin state (True - HIGH or False - LOW)
 */
void changePinState(PortDef *port, bool state) {
    port->state = state;
    HAL_GPIO_WritePin((GPIO_TypeDef *) port->handle, port->pin, state);
}

/**
 * @brief Check if the input pin has changed state (detect this event)
 * @param port is the base GPIO port data structure
 * @return True - a pin changed state from LOW to HIGH, otherwise - False
 */
bool isPinTriggered(const PortDef *port) {
    return port->isTriggered;
}

/**
 * @brief Get/read the current state of the input pin
 * @param port is the base GPIO port data structure
 * @return True - a pin is HIGH, otherwise - LOW
 */
bool getPinState(const PortDef *port) {
    return port->state;
}

/**
 * @brief Update the GPIO pin data structure state
 * @param port is the base GPIO port data structure
 */
void checkPinState(PortDef *port) {
    bool isSet = HAL_GPIO_ReadPin((GPIO_TypeDef *) port->handle, port->pin);
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

/**
 * @brief Change the timer frequency via updating the prescaler value
 * @param tim is the base timer data structure
 * @param value is the prescaler value
 */
void setTimerPrescaler(TimerDef *tim, uint16_t value) {
    tim->prescaler = value;
    __HAL_TIM_SET_PRESCALER((TIM_HandleTypeDef *) tim->handle, tim->prescaler);
}

/**
 * @brief Set the duty value of the PWM signal
 * @param tim is the base timer data structure
 * @param channel is the timer output channel (TIM_CHANNEL_x (x=1..6) or TIM_CHANNEL_ALL)
 * @param value is the duty value (from 0 to 100)
 */
void setPWMDutyCycle(TimerDef *tim, uint16_t channel, uint8_t value) {
    value = (value > 100) ? 100 : value;
    __HAL_TIM_SET_COMPARE((TIM_HandleTypeDef *) tim->handle, channel, value);
}

/**
 * @brief Release/unlock PWM output pins after a break event
 * @param tim is the base timer data structure
 */
void releasePWMBreakState(TimerDef *tim) {
    __HAL_TIM_MOE_ENABLE((TIM_HandleTypeDef *) tim->handle);
}

/**
 * @brief Check if the ADC module has finished conversion process (all pins/channels)
 * @param adc is the base ADC data structure
 * @return True - conversion completed, otherwise - False
 */
bool isADCFinished(const ADCDef *adc) {
    return adc->isFinished;
}

/**
 * @brief Start/run analog-to-digital conversion process
 * @param adc is the base ADC data structure
 * @return HAL_OK - successfully, otherwise - error code
 */
int readAnalogValues(ADCDef *adc) {
    if (adc->isProcessing)
        return HAL_BUSY;

    adc->isProcessing = true;
    adc->errType = 0;
    adc->idx = 0;
    return HAL_ADC_Start_IT((ADC_HandleTypeDef *) adc->handle);
}

/**
 * @brief Set the comparator threshold voltage
 * @param comp is the base comparator data structure
 * @param voltage is the threshold level (mV)
 * @return HAL_OK - successfully, otherwise - error code
 */
int setCompThresholdLevel(CompDef *comp, uint32_t voltage) {
    return setDACOutput(&comp->dac, DAC_CHANNEL_2, voltage);
}

/**
 * @brief Set the DAC output DC voltage
 * @param dac is the base DAC data structure
 * @param channel is the DAC output channel (DAC_CHANNEL_1 or DAC_CHANNEL_2)
 * @param voltage is the DAC output DC level (mV)
 * @return HAL_OK - successfully, otherwise - error code
 */
int setDACOutput(DACDef *dac, uint8_t channel, uint32_t voltage) {
    if (channel != DAC_CHANNEL_1 && channel != DAC_CHANNEL_2)
        return -1;
    if (voltage > VREFP)
        return -2;

    voltage *= 4095;
    dac->value = voltage / VREFP;
    HAL_DAC_SetValue((DAC_HandleTypeDef *) dac->handle, channel, DAC_ALIGN_12B_R, dac->value);
    return HAL_DAC_Start((DAC_HandleTypeDef *) dac->handle, channel);
}

/**
 * @brief Select/set the signal generator mode
 * @param gen is the base generator data structure
 * @param mode is the mode value (enum GeneratorModes)
 * @return 0 - successfully, -1 - the generator mode is invalid
 */
int changeGeneratorMode(GeneratorDef *gen, uint8_t mode) {
    if (mode >= NUMBER_GEN_MODES)
        return -1;

    gen->mode = mode;
    uint16_t baseLevel = 0; // mV
    uint32_t chNum = DAC_CHANNEL_2;
    DAC_HandleTypeDef *dacHandler = (DAC_HandleTypeDef *) gen->dac.handle;
    HAL_TIM_Base_Stop((TIM_HandleTypeDef *) gen->timer_1.handle);
    HAL_TIM_Base_Stop((TIM_HandleTypeDef *) gen->timer_2.handle);
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
                HAL_TIM_Base_Start((TIM_HandleTypeDef *) gen->timer_1.handle);
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
                HAL_TIM_Base_Start((TIM_HandleTypeDef *) gen->timer_1.handle);
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
                HAL_TIM_Base_Start((TIM_HandleTypeDef *) gen->timer_1.handle);
                HAL_TIM_Base_Start((TIM_HandleTypeDef *) gen->timer_2.handle);
            }
            break;
        }
        default:
            break;
    }

    return 0;
}

/**
 * @brief Get the precise voltage of Vrefint
 * @return mV value
 */
uint32_t getRealVrefint(void) {
    // Datasheet, DS12589 Rev. 6, Analog-to-digital converter (ADC),
    // Internal voltage reference (Vrefint), page 30

    const uint16_t *calValue = (const uint16_t *) 0x1FFF75AA;
    uint32_t result = *calValue;
    result = result * 3000 / 4095;
    return result;
}

/**
 * @brief Get the MCU Unique ID
 * @return pointer to a unique id value
 */
const void *getUniqueID(void) {
    // RM0440 Reference manual, 48.1 Unique device ID, page 2108
    return (const void *) 0x1FFF7590;
}

/**
 * @brief Calculate CRC-32
 * @param data is target data
 * @param size is target data size (bytes)
 * @return if successfully - CRC value, otherwise - 0
 */
uint32_t getCRC(const void *data, uint16_t size) {
    // https://crccalc.com/
    // CRC-32

    uint32_t result = HAL_CRC_Calculate((CRC_HandleTypeDef *) Mcu.handles.crc, (uint32_t *) data, size);
    HAL_CRC_StateTypeDef state = HAL_CRC_GetState((CRC_HandleTypeDef *) Mcu.handles.crc);
    return (state == HAL_CRC_STATE_READY) ? (result ^ 0xFFFFFFFFU) : 0;
}

/**
 * @brief Generate random numbers (uint32_t)
 * @param dst is the destination address(array)
 * @param n is the value of how many random numbers you need
 * @return 0 - wrong input data, i - number of generated values
 */
uint16_t generateRandomNumbers(uint32_t *dst, uint16_t n) {
    if (dst == NULL || n == 0)
        return 0;

    uint16_t i = 0;
    uint32_t value = 0;
    for (i = 0; i < n; ++i) {
        if (HAL_RNG_GenerateRandomNumber((RNG_HandleTypeDef *) Mcu.handles.rng, &value) == HAL_OK)
            *(dst + i) = value;
        else
            break;
    }
    return i;
}
