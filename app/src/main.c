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

    while (1) {
        HAL_IWDG_Refresh((IWDG_HandleTypeDef *) Mcu.handles.wdt);

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
    }

    if (Mcu.measTimer.isTriggered) {
        Mcu.measTimer.isTriggered = false;

        readAnalogValues(&Mcu.adc);
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
                value >>= 12;
            }
            Mcu.adc.value[i] = (uint16_t) value;
        }
    }

    if (Mcu.comp.isTriggered) {
        Mcu.comp.isTriggered = false;

        if (Mcu.comp.errType == 0) {
        }
    }

    UART_update(&Mcu.uart, HAL_GetTick());
    if (UART_isHaveData(&Mcu.uart)) {
        Mcu.uart.isHaveData = false;
    }
}

static int initPeripheral(void) {
    Mcu.led[LED_GREEN].pin = GPIO_PIN_5;
    Mcu.led[LED_GREEN].handle = GPIOA;
    Mcu.button[BUTTON_1].duration = 50; // ms
    Mcu.button[BUTTON_1].pin = GPIO_PIN_13;
    Mcu.button[BUTTON_1].handle = GPIOC;

    UART_init(&Mcu.uart);

    if (SETTING_SUCCESS == initialization(&Mcu)) {
        setCompThresholdLevel(&Mcu.comp, 2137);
        HAL_COMP_Start((COMP_HandleTypeDef *) Mcu.comp.handle);

        HAL_TIM_PWM_Start((TIM_HandleTypeDef *) Mcu.pwmTimer.handle, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start((TIM_HandleTypeDef *) Mcu.pwmTimer.handle, TIM_CHANNEL_1);
        setPWMDutyCycle(&Mcu.pwmTimer, TIM_CHANNEL_1, 15);

        HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *) Mcu.measTimer.handle);
    }
    return 0;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1) {
    }
}
#endif
