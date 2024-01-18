#include "stm32g4xx_hal.h"

#include "settings.h"

static TIM_HandleTypeDef timer7Handler;

static int settingSystemClock(void) {
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitTypeDef oscInit = {0};
    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    oscInit.LSEState = RCC_LSE_OFF;
    oscInit.HSEState = RCC_HSE_OFF;
    oscInit.HSI48State = RCC_HSI48_OFF;
    oscInit.LSIState = RCC_LSI_ON;
    oscInit.HSIState = RCC_HSI_ON;
    oscInit.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    // Datasheet, DS12589 Rev. 6, Electrical characteristics, PLL characteristics, page 106
    // Voltage scaling Range 1: 150MHz
    // PLL VCO: 288MHz
    oscInit.PLL.PLLState = RCC_PLL_ON;
    oscInit.PLL.PLLSource = RCC_PLLSOURCE_HSI; // 16MHz
    oscInit.PLL.PLLM = RCC_PLLM_DIV1;
    oscInit.PLL.PLLN = 18;
    oscInit.PLL.PLLP = RCC_PLLP_DIV2;
    oscInit.PLL.PLLQ = RCC_PLLQ_DIV6;
    oscInit.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)
        return SETTING_ERROR;

    RCC_ClkInitTypeDef clkInit = {0};
    clkInit.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1; // 144MHz
    clkInit.APB1CLKDivider = RCC_HCLK_DIV4; // 36MHz
    clkInit.APB2CLKDivider = RCC_HCLK_DIV4; // 36MHz

    if (HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_4) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingGPIO(void) {
    GPIO_InitTypeDef gpioInit = {0};

    // Button
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpioInit.Pin = GPIO_PIN_13;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_PULLDOWN;
    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpioInit);

    // LED
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpioInit.Pin = GPIO_PIN_5;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_PULLDOWN;
    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    // GPIO: D2, D7, D8
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpioInit.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_PULLDOWN;
    gpioInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    // GPIO: D4
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpioInit.Pin = GPIO_PIN_5;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_PULLDOWN;
    gpioInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    return SETTING_SUCCESS;
}

static int settingTimer(TimerDef *timer) {
    timer->obj = (void *) &timer7Handler;
    TIM_HandleTypeDef *tim = (TIM_HandleTypeDef *) timer->obj;
    tim->Instance = TIM7;
    tim->Init.Period = timer->freq / 1000U - 1U;
    tim->Init.Prescaler = HAL_RCC_GetPCLK1Freq() / timer->freq - 1U;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    tim->Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(tim) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int turnOnInterrupts(MCUDef *mcu) {
    if (HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *) mcu->timer_8kHz.obj) == HAL_OK)
        return SETTING_SUCCESS;

    return SETTING_ERROR;
}

int initialization(MCUDef *mcu) {
    if (settingSystemClock() == SETTING_SUCCESS &&
        settingGPIO() == SETTING_SUCCESS &&
        settingTimer(&mcu->timer_8kHz) == SETTING_SUCCESS)
        return turnOnInterrupts(mcu);

    return SETTING_ERROR;
}
