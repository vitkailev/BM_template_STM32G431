#include "stm32g4xx_hal.h"

#include "settings.h"
#include "functions.h"

MCUDef Mcu;

static int initPeripheral(void);

static void application(void);

int main(void) {
    HAL_Init();
    initPeripheral();

    while (true) {
        application();
    }
    return 1;
}

static int initPeripheral(void) {

    Mcu.button.duration = 50; // ms
    Mcu.button.pin = GPIO_PIN_13;
    Mcu.button.obj = GPIOC;
    Mcu.led.pin = GPIO_PIN_5;
    Mcu.led.obj = GPIOA;
    Mcu.oscPins[OSC_CHANNEL_1].pin = GPIO_PIN_10; // D2
    Mcu.oscPins[OSC_CHANNEL_1].obj = GPIOA;
    Mcu.oscPins[OSC_CHANNEL_2].pin = GPIO_PIN_5; // D4
    Mcu.oscPins[OSC_CHANNEL_2].obj = GPIOB;
    Mcu.oscPins[OSC_CHANNEL_3].pin = GPIO_PIN_8; // D7
    Mcu.oscPins[OSC_CHANNEL_3].obj = GPIOA;
    Mcu.oscPins[OSC_CHANNEL_4].pin = GPIO_PIN_9; // D8
    Mcu.oscPins[OSC_CHANNEL_4].obj = GPIOA;
    Mcu.timer_8kHz.freq = 8000; // Hz

    initialization(&Mcu);
    return 0;
}

static void application(void) {
    if (Mcu.flags.isSysTickTriggered) {
        Mcu.flags.isSysTickTriggered = false;

        Mcu.runtime++;
        checkPinState(&Mcu.button);

        changePinState(&Mcu.oscPins[OSC_CHANNEL_1], !getPinState(&Mcu.oscPins[OSC_CHANNEL_1]));
    }

    if (Mcu.timer_8kHz.isTriggered) {
        Mcu.timer_8kHz.isTriggered = false;

        changePinState(&Mcu.oscPins[OSC_CHANNEL_2], !getPinState(&Mcu.oscPins[OSC_CHANNEL_2]));
    }

    if (isPinTriggered(&Mcu.button)) {
        Mcu.button.isTriggered = false;

        changePinState(&Mcu.led, !getPinState(&Mcu.led));
    }
}
