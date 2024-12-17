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

void setTimerPrescaler(TimerDef *tim, uint16_t value);

void setPWMDutyCycle(TimerDef *tim, uint16_t channel, uint8_t value);

void releasePWMBreakState(TimerDef *tim);

bool isADCFinished(const ADCDef *adc);

int readAnalogValues(ADCDef *adc);

int setCompThresholdLevel(CompDef *comp, uint32_t voltage);

int setDACOutput(DACDef *dac, uint8_t channel, uint32_t voltage);

int changeGeneratorMode(GeneratorDef *gen, uint8_t mode);

uint32_t getRealVrefint(void);

const void *getUniqueID(void);

uint32_t getCRC(const void *data, uint16_t size);

uint16_t generateRandomNumbers(uint32_t *dst, uint16_t n);

#ifdef __cplusplus
}
#endif

#endif // FUNCTIONS_H
