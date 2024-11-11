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
        HAL_IWDG_Refresh((IWDG_HandleTypeDef *) Mcu.handlers.wdt);

        application();
    }
    return 1;
}

static void application(void) {
    if (Mcu.flags.isWDTTriggered) {
        Mcu.flags.isWDTTriggered = false;
    }

    if (Mcu.flags.isSysTickTriggered) {
        Mcu.flags.isSysTickTriggered = false;

        checkPinState(&Mcu.button[BUTTON_1]);
        changePinState(&Mcu.oscPins[OSC_CHANNEL_1], !getPinState(&Mcu.oscPins[OSC_CHANNEL_1]));
    }

    if (Mcu.measTimer.isTriggered) {
        Mcu.measTimer.isTriggered = false;

        readAnalogValues(&Mcu.adc);
        changePinState(&Mcu.oscPins[OSC_CHANNEL_2], !getPinState(&Mcu.oscPins[OSC_CHANNEL_2]));
    }

    if (isPinTriggered(&Mcu.button[BUTTON_1])) {
        Mcu.button[BUTTON_1].isTriggered = false;

    }

    if (isADCFinished(&Mcu.adc)) {
        Mcu.adc.isFinished = false;

        int32_t value = 0;
        for (size_t i = 0; i < NUMBER_ADC_CHANNELS; ++i) {
            value = Mcu.adc.rawValues[i];
            if (i == ANALOG_TEMP_VREF) {
                Mcu.temp = (int16_t) __LL_ADC_CALC_TEMPERATURE(VREFP, value, LL_ADC_RESOLUTION_12B);
                value = 0;
            } else {
                value *= VREFP;
                value /= 4095;
            }
            Mcu.adc.value[i] = value;
        }
    }

    if (Mcu.comp.isTriggered) {
        Mcu.comp.isTriggered = false;

        if (Mcu.comp.errType == 0)
            changePinState(&Mcu.oscPins[OSC_CHANNEL_3], Mcu.comp.state);
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

static int initPeripheral(void) {

    Mcu.led[LED_GREEN].pin = GPIO_PIN_5;
    Mcu.led[LED_GREEN].obj = GPIOA;
    Mcu.button[BUTTON_1].duration = 50; // ms
    Mcu.button[BUTTON_1].pin = GPIO_PIN_13;
    Mcu.button[BUTTON_1].obj = GPIOC;
    Mcu.oscPins[OSC_CHANNEL_1].pin = GPIO_PIN_10; // D2
    Mcu.oscPins[OSC_CHANNEL_1].obj = GPIOA;
    Mcu.oscPins[OSC_CHANNEL_2].pin = GPIO_PIN_5; // D4
    Mcu.oscPins[OSC_CHANNEL_2].obj = GPIOB;
    Mcu.oscPins[OSC_CHANNEL_3].pin = GPIO_PIN_8; // D7
    Mcu.oscPins[OSC_CHANNEL_3].obj = GPIOA;
    Mcu.oscPins[OSC_CHANNEL_4].pin = GPIO_PIN_9; // D8
    Mcu.oscPins[OSC_CHANNEL_4].obj = GPIOA;

    if (initialization(&Mcu) == SETTING_SUCCESS) {
        setCompThresholdLevel(&Mcu.comp, 2137);
        HAL_COMP_Start((COMP_HandleTypeDef *) Mcu.comp.handler);

        UART_init(&Mcu.uart1);
        UART_init(&Mcu.uart2);
    }
    return 0;
}
