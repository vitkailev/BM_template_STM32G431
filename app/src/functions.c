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

    voltage <<= 12;
    dac->value = voltage / VREFP;
    HAL_DAC_SetValue((DAC_HandleTypeDef *) dac->handle, channel, DAC_ALIGN_12B_R, dac->value);
    return HAL_DAC_Start((DAC_HandleTypeDef *) dac->handle, channel);
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
