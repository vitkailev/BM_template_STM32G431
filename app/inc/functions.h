#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "variables.h"

void changePinState(PortDef *port, bool state);

bool isPinTriggered(const PortDef *port);

bool getPinState(const PortDef *port);

void checkPinState(PortDef *port);

void readUniqueID(MCUDef *mcu);

uint32_t getCRC(const void *data, uint32_t size);

uint16_t generateRandomNumbers(uint32_t *dst, uint16_t n);

#ifdef __cplusplus
}
#endif

#endif // FUNCTIONS_H
