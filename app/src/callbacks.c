#include "stm32g4xx_hal.h"

#include "variables.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == ((TIM_HandleTypeDef *) Mcu.timer_8kHz.obj)->Instance) {
        Mcu.timer_8kHz.isTriggered = true;
    }
}
