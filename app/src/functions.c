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
