#ifndef VARIABLES_H
#define VARIABLES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

enum Constants {
    LED_GREEN = 0,
    NUMBER_LEDS,

    BUTTON_1 = 0,
    NUMBER_BUTTONS,

    // Oscilloscope measurement
    OSC_CHANNEL_1 = 0,
    OSC_CHANNEL_2,
    OSC_CHANNEL_3,
    OSC_CHANNEL_4,
    NUMBER_OSC_CHANNELS
};

typedef struct {
    volatile bool isSysTickTriggered;
} FlagsDef;

typedef struct {
    volatile bool isTriggered;
    uint16_t freq;
    void *obj;
} TimerDef;

typedef struct {
    FlagsDef flags;

    uint32_t runtime;

    TimerDef timer_8kHz;
} MCUDef;

extern MCUDef Mcu;

#ifdef __cplusplus
}
#endif

#endif // VARIABLES_H
