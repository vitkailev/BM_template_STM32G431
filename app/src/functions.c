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

void readUniqueID(MCUDef *mcu) {
    // RM0440 Reference manual, 48.1 Unique device ID, page 2108
    const void *baseAddr = (const void *) 0x1FFF7590;
    memcpy((void *) mcu->uniqueID, (const void *) baseAddr, UNIQUE_ID_SIZE);
}

uint32_t getCRC(const void *data, uint32_t size) {
    // https://crccalc.com/
    // CRC-32

    uint32_t result = HAL_CRC_Calculate(Mcu.crcHandler, (uint32_t *) data, size);
    HAL_CRC_StateTypeDef state = HAL_CRC_GetState(Mcu.crcHandler);
    return (state == HAL_CRC_STATE_READY) ? (result ^ 0xFFFFFFFFU) : 0;
}

uint16_t generateRandomNumbers(uint32_t *dst, uint16_t n) {
    if (dst == NULL || n == 0)
        return 0;

    uint16_t i = 0;
    uint32_t value = 0;
    for (i = 0; i < n; ++i) {
        if (HAL_RNG_GenerateRandomNumber(Mcu.rngHandler, &value) == HAL_OK)
            *(dst + i) = value;
        else
            break;
    }
    return i;
}
