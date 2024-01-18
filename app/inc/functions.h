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


#ifdef __cplusplus
}
#endif

#endif // FUNCTIONS_H
