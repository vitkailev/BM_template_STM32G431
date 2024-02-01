#ifndef VARIABLES_H
#define VARIABLES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "uart.h"

enum Constants {
    UNIQUE_ID_SIZE = 12U,

    LED_GREEN = 0,
    NUMBER_LEDS,

    BUTTON_1 = 0,
    NUMBER_BUTTONS,

    // Oscilloscope measurement
    OSC_CHANNEL_1 = 0,
    OSC_CHANNEL_2,
    OSC_CHANNEL_3,
    OSC_CHANNEL_4,
    NUMBER_OSC_CHANNELS,

    NUMBER_ADC_CHANNELS = 4U // 3 input pins + Vref or Vtemp
};

typedef struct {
    volatile bool isSysTickTriggered;
    bool isWDTTriggered;
} FlagsDef;

typedef struct {
    bool isTriggered;
    bool state;

    uint16_t delay;
    uint16_t duration;

    uint16_t pin;
    void *obj;
} PortDef;

typedef struct {
    volatile bool isTriggered;
    uint16_t freq;
    void *obj;
} TimerDef;

typedef struct {
    volatile bool isProcessing;
    volatile bool isFinished;
    volatile uint32_t errType;
    volatile uint32_t errors;

    uint8_t idx;
    uint16_t rawValues[NUMBER_ADC_CHANNELS]; // relative values
    uint16_t value[NUMBER_ADC_CHANNELS]; // mV

    void *obj;
} ADCDef;

typedef struct {
    void *crc;
    void *rng;
    void *wdt;
} HandlersDef;

typedef struct {
    FlagsDef flags;

    uint32_t runtime;
    uint8_t uniqueID[UNIQUE_ID_SIZE];

    PortDef led;
    PortDef button;
    PortDef oscPins[NUMBER_OSC_CHANNELS];

    HandlersDef handlers;
    TimerDef timer_8kHz;
    ADCDef adc;
    UARTDef uart1;
    UARTDef uart2;
} MCUDef;

extern MCUDef Mcu;

#ifdef __cplusplus
}
#endif

#endif // VARIABLES_H
