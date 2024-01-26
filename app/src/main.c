#include "stm32g4xx_hal.h"

#include "settings.h"
#include "functions.h"

MCUDef Mcu;

static int initPeripheral(void);

static void application(void);

int main(void) {
    HAL_Init();
    initPeripheral();

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
        Mcu.flags.isWDTTriggered = true;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    while (true) {
        HAL_IWDG_Refresh((IWDG_HandleTypeDef *) Mcu.wdtHandler);

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

    if (initialization(&Mcu) == SETTING_SUCCESS) {
        readUniqueID(&Mcu);
        UART_init(&Mcu.uart1);
        UART_init(&Mcu.uart2);
    }

    return 0;
}

static void application(void) {
    if (Mcu.flags.isWDTTriggered) {
        Mcu.flags.isWDTTriggered = false;

        changePinState(&Mcu.led, true);
    }

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
    }

    UART_update(&Mcu.uart1, HAL_GetTick());
    if (UART_isHaveData(&Mcu.uart1)) {
        Mcu.uart1.isHaveData = false;
    }

    UART_update(&Mcu.uart2, HAL_GetTick());
    if (UART_isHaveData(&Mcu.uart2)) {
        Mcu.uart2.isHaveData = false;
    }
}
