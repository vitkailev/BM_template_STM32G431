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
    VREFP = 3250, // mV
    UNIQUE_ID_SIZE = 12, // 96 bits

    LED_GREEN = 0,
    NUMBER_LEDS,

    BUTTON_1 = 0,
    NUMBER_BUTTONS,

    ANALOG_IN_1 = 0,
    ANALOG_IN_2,
    ANALOG_TEMP_VREF,
    NUMBER_ADC_CHANNELS
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
    void *handle;
} PortDef;

typedef struct {
    volatile bool isTriggered;
    uint16_t prescaler;
    uint32_t freq;

    void *handle;
} TimerDef;

typedef struct {
    volatile bool isProcessing;
    volatile bool isFinished;
    volatile uint32_t errType;
    volatile uint32_t errors;

    uint8_t idx;
    uint16_t rawValues[NUMBER_ADC_CHANNELS]; // relative values
    uint16_t value[NUMBER_ADC_CHANNELS]; // mV

    void *handle;
} ADCDef;

typedef struct {
    uint8_t mode;
    uint32_t value;
    DAC_ChannelConfTypeDef channel;

    void *handle;
} DACDef;

typedef struct {
    volatile bool isTriggered;
    volatile bool state;
    volatile uint32_t errType;
    volatile uint32_t errors;

    DACDef dac;
    void *handle;
} CompDef;

typedef struct {
    void *crc;
    void *wdt;
} HandlesDef;

typedef struct {
    FlagsDef flags;

    volatile uint32_t runtime;
    int16_t temp; // C

    PortDef led[NUMBER_LEDS];
    PortDef button[NUMBER_BUTTONS];

    HandlesDef handles;
    TimerDef measTimer;
    ADCDef adc;
    CompDef comp;
    DACDef dac;
    TimerDef pwmTimer;
    UARTDef uart;
} MCUDef;

extern MCUDef Mcu;

#ifdef __cplusplus
}
#endif

#endif // VARIABLES_H
