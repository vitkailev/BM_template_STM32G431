#include "stm32g4xx_hal.h"

#include "settings.h"

static TIM_HandleTypeDef timer6Handler;
static TIM_HandleTypeDef timer7Handler;
static TIM_HandleTypeDef timer8Handler;
static TIM_HandleTypeDef timer15Handler;
static ADC_HandleTypeDef adcHandler;
static DAC_HandleTypeDef dac1Handler;
static DAC_HandleTypeDef dac3Handler;
static COMP_HandleTypeDef compHandler;
static UART_HandleTypeDef usart1Handler;
static UART_HandleTypeDef usart2Handler;
static CRC_HandleTypeDef crcHandler;
static RNG_HandleTypeDef rngHandler;
static IWDG_HandleTypeDef wdtHandler;

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

    // Datasheet, DS12589 Rev. 6, Electrical characteristics, Analog-to-digital converter characteristics, page 118
    // Range 1, Vdda >= 2.7V, max 52MHz
    oscInit.PLL.PLLP = RCC_PLLP_DIV6; // 48MHz

    oscInit.PLL.PLLQ = RCC_PLLQ_DIV6;
    oscInit.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)
        return SETTING_ERROR;

    RCC_ClkInitTypeDef clkInit = {0};
    clkInit.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1; // HCLK = 144MHz
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
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    gpioInit.Pin = GPIO_PIN_5;
//    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
//    gpioInit.Pull = GPIO_PULLDOWN;
//    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(GPIOA, &gpioInit);

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
    uint32_t sourceClock = HAL_RCC_GetPCLK2Freq();
    // APB2Divider != 1
    sourceClock *= 2;

    timer->handler = (void *) &timer15Handler;
    timer->freq = 4000U; // Hz
    timer->prescaler = sourceClock / timer->freq / 100U - 1U;

    TIM_HandleTypeDef *tim = (TIM_HandleTypeDef *) timer->handler;
    tim->Instance = TIM15;
    tim->Init.Period = 100U - 1U;
    tim->Init.Prescaler = timer->prescaler;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    tim->Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(tim) != HAL_OK)
        return SETTING_ERROR;

    TIM_MasterConfigTypeDef masterConf = {0};
    masterConf.MasterOutputTrigger = TIM_TRGO_UPDATE;
    masterConf.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
    masterConf.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(tim, &masterConf) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingPWM(TimerDef *timer) {
    uint32_t sourceClock = HAL_RCC_GetPCLK2Freq();
    // APB2Divider != 1
    sourceClock *= 2;

    timer->handler = (void *) &timer8Handler;
    timer->freq = 10000U; // Hz
    timer->prescaler = sourceClock / timer->freq / 100U - 1U;

    TIM_HandleTypeDef *tim = (TIM_HandleTypeDef *) timer->handler;
    tim->Instance = TIM8;
    tim->Init.Period = 100U - 1U;
    tim->Init.Prescaler = timer->prescaler;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    tim->Init.RepetitionCounter = 0;

    if (HAL_TIM_PWM_Init(tim) != HAL_OK)
        return SETTING_ERROR;

    TIM_MasterConfigTypeDef masterConf = {0};
    masterConf.MasterOutputTrigger = TIM_TRGO_UPDATE;
    masterConf.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
    masterConf.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(tim, &masterConf) != HAL_OK)
        return SETTING_ERROR;

    TIM_OC_InitTypeDef pwm = {0};
    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    pwm.OCIdleState = TIM_OCIDLESTATE_RESET;
    pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    pwm.Pulse = 5;
    if (HAL_TIM_PWM_ConfigChannel(tim, &pwm, TIM_CHANNEL_1) != HAL_OK)
        return SETTING_ERROR;
    pwm.Pulse = 5;
    if (HAL_TIM_PWM_ConfigChannel(tim, &pwm, TIM_CHANNEL_2) != HAL_OK)
        return SETTING_ERROR;

    if (HAL_TIMEx_ConfigDeadTime(tim, 64) != HAL_OK)
        return SETTING_ERROR;

    TIMEx_BreakInputConfigTypeDef breakInit = {0};
    breakInit.Source = TIM_BREAKINPUTSOURCE_BKIN;
    breakInit.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
    breakInit.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
    if (HAL_TIMEx_ConfigBreakInput(tim, TIM_BREAKINPUT_BRK, &breakInit) != HAL_OK)
        return SETTING_ERROR;

    TIM_BreakDeadTimeConfigTypeDef breakDT = {0};
    breakDT.OffStateRunMode = TIM_OSSR_ENABLE;
    breakDT.OffStateIDLEMode = TIM_OSSI_ENABLE;
    breakDT.LockLevel = TIM_LOCKLEVEL_OFF;
    breakDT.DeadTime = 0;
    breakDT.BreakState = TIM_BREAK_ENABLE;
    breakDT.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    breakDT.BreakFilter = 0;
    breakDT.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    breakDT.Break2State = TIM_BREAK2_DISABLE;
    breakDT.Break2Filter = 0;
    breakDT.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    breakDT.Break2AFMode = TIM_BREAK2_AFMODE_INPUT;
    breakDT.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(tim, &breakDT) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingADC(ADCDef *adc) {
    adc->handler = (void *) &adcHandler;
    ADC_HandleTypeDef *adcInit = (ADC_HandleTypeDef *) adc->handler;
    adcInit->Instance = ADC1;
    adcInit->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    adcInit->Init.Resolution = ADC_RESOLUTION_12B;
    adcInit->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adcInit->Init.GainCompensation = 0;
    adcInit->Init.ScanConvMode = ADC_SCAN_ENABLE;
    adcInit->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    adcInit->Init.LowPowerAutoWait = DISABLE;
    adcInit->Init.ContinuousConvMode = DISABLE;
    adcInit->Init.NbrOfConversion = 4;
    adcInit->Init.DiscontinuousConvMode = ENABLE;
    adcInit->Init.NbrOfDiscConversion = 1;
    adcInit->Init.ExternalTrigConv = ADC_SOFTWARE_START;
//    adcInit->Init.ExternalTrigConvEdge =;
    adcInit->Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    adcInit->Init.DMAContinuousRequests = DISABLE;
    adcInit->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    adcInit->Init.OversamplingMode = DISABLE;
//    adcInit->Init.Oversampling =;

    if (HAL_ADC_Init(adcInit) != HAL_OK)
        return SETTING_ERROR;

    if (HAL_ADCEx_Calibration_Start(adcInit, ADC_SINGLE_ENDED) != HAL_OK)
        return SETTING_ERROR;

    ADC_ChannelConfTypeDef chInit = {0};

    chInit.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    chInit.SingleDiff = ADC_SINGLE_ENDED;
    chInit.OffsetNumber = ADC_OFFSET_NONE;
    chInit.Offset = 0;
    chInit.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
    chInit.OffsetSaturation = DISABLE;

    // PA0, ADC12
    chInit.Channel = ADC_CHANNEL_1;
    chInit.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(adcInit, &chInit) != HAL_OK)
        return SETTING_ERROR;

    // PA1, ADC12
    chInit.Channel = ADC_CHANNEL_2;
    chInit.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(adcInit, &chInit) != HAL_OK)
        return SETTING_ERROR;

    // PB0, ADC1
    chInit.Channel = ADC_CHANNEL_15;
    chInit.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(adcInit, &chInit) != HAL_OK)
        return SETTING_ERROR;

    // only ADC1
    // ADC_CHANNEL_TEMPSENSOR_ADC1 or ADC_CHANNEL_VREFINT
    chInit.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
    chInit.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(adcInit, &chInit) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingGenerator(GeneratorDef *gen) {
    gen->dac.handler = (void *) &dac1Handler;
    DAC_HandleTypeDef *dacInit = (DAC_HandleTypeDef *) gen->dac.handler;
    dacInit->Instance = DAC1;
    if (HAL_DAC_Init(dacInit) != HAL_OK)
        return SETTING_ERROR;

    gen->dac.channel.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    gen->dac.channel.DAC_DMADoubleDataMode = DISABLE;
    gen->dac.channel.DAC_SignedFormat = DISABLE;
    gen->dac.channel.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    gen->dac.channel.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
    gen->dac.channel.DAC_Trigger2 = DAC_TRIGGER_NONE;
    gen->dac.channel.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    gen->dac.channel.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    gen->dac.channel.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
//    chInit.DAC_TrimmingValue =;
//    chInit.DAC_SampleAndHoldConfig =;

    // channel 1 - DC - manual changed voltage
    if (HAL_DAC_ConfigChannel(dacInit, &gen->dac.channel, DAC_CHANNEL_1) != HAL_OK)
        return SETTING_ERROR;

    if (HAL_DACEx_SelfCalibrate(dacInit, &gen->dac.channel, DAC_CHANNEL_1) != HAL_OK)
        return SETTING_ERROR;

    // channel 2 - Timer triggered signal generation, default values
    gen->dac.channel.DAC_Trigger = DAC_TRIGGER_NONE;
    gen->dac.channel.DAC_Trigger2 = DAC_TRIGGER_NONE;

    uint32_t sourceClock = HAL_RCC_GetPCLK1Freq();
    // APB1Divider != 1
    sourceClock *= 2;

    TIM_HandleTypeDef *tim = NULL;
    TIM_MasterConfigTypeDef masterConf = {0};

    gen->timer_1.handler = (void *) &timer6Handler;
    gen->timer_1.freq = 0; // Hz, dynamic
    gen->timer_1.prescaler = 0;

    tim = (TIM_HandleTypeDef *) gen->timer_1.handler;
    tim->Instance = TIM6;
    tim->Init.Period = 4U - 1U;
    tim->Init.Prescaler = gen->timer_1.prescaler;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    tim->Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(tim) != HAL_OK)
        return SETTING_ERROR;

    masterConf.MasterOutputTrigger = TIM_TRGO_UPDATE;
    masterConf.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
    masterConf.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(tim, &masterConf) != HAL_OK)
        return SETTING_ERROR;

    gen->timer_2.handler = (void *) &timer7Handler;
    gen->timer_2.freq = 9000; // Hz, dynamic
    gen->timer_2.prescaler = sourceClock / gen->timer_2.freq / 100U - 1U;

    tim = (TIM_HandleTypeDef *) gen->timer_2.handler;
    tim->Instance = TIM7;
    tim->Init.Period = 100U - 1U;
    tim->Init.Prescaler = gen->timer_2.prescaler;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    tim->Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(tim) != HAL_OK)
        return SETTING_ERROR;

    masterConf.MasterOutputTrigger = TIM_TRGO_UPDATE;
    masterConf.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
    masterConf.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(tim, &masterConf) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingComp(CompDef *comp) {
    comp->dac.handler = (void *) &dac3Handler;
    DAC_HandleTypeDef *dacInit = (DAC_HandleTypeDef *) comp->dac.handler;
    dacInit->Instance = DAC3;
    if (HAL_DAC_Init(dacInit) != HAL_OK)
        return SETTING_ERROR;

    comp->dac.channel.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    comp->dac.channel.DAC_DMADoubleDataMode = DISABLE;
    comp->dac.channel.DAC_SignedFormat = DISABLE;
    comp->dac.channel.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    comp->dac.channel.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
    comp->dac.channel.DAC_Trigger2 = DAC_TRIGGER_NONE;
    comp->dac.channel.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    comp->dac.channel.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
    comp->dac.channel.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
//    chInit.DAC_TrimmingValue =;
//    chInit.DAC_SampleAndHoldConfig =;

    if (HAL_DAC_ConfigChannel(dacInit, &comp->dac.channel, DAC_CHANNEL_2) != HAL_OK)
        return SETTING_ERROR;

    if (HAL_DACEx_SelfCalibrate(dacInit, &comp->dac.channel, DAC_CHANNEL_2) != HAL_OK)
        return SETTING_ERROR;

    comp->handler = (void *) &compHandler;
    COMP_HandleTypeDef *compInit = (COMP_HandleTypeDef *) comp->handler;
    compInit->Instance = COMP2;
    compInit->Init.InputPlus = COMP_INPUT_PLUS_IO1; // PA7
    compInit->Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
    compInit->Init.Hysteresis = COMP_HYSTERESIS_40MV;
    compInit->Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    compInit->Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
    compInit->Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;

    if (HAL_COMP_Init(compInit) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingUART(MCUDef *mcu) {
    UART_HandleTypeDef *uartInit = NULL;

    mcu->uart1.obj = &usart1Handler;
    uartInit = (UART_HandleTypeDef *) mcu->uart1.obj;
    uartInit->Instance = USART1;
    uartInit->FifoMode = UART_FIFOMODE_DISABLE;
    uartInit->Init.BaudRate = 115200;
    uartInit->Init.WordLength = UART_WORDLENGTH_8B;
    uartInit->Init.StopBits = UART_STOPBITS_1;
    uartInit->Init.Parity = UART_PARITY_NONE;
    uartInit->Init.Mode = UART_MODE_TX_RX;
    uartInit->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartInit->Init.OverSampling = UART_OVERSAMPLING_16;
    uartInit->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uartInit->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uartInit->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(uartInit) != HAL_OK)
        return SETTING_ERROR;

    mcu->uart2.obj = &usart2Handler;
    uartInit = (UART_HandleTypeDef *) mcu->uart2.obj;
    uartInit->Instance = USART2;
    uartInit->FifoMode = UART_FIFOMODE_DISABLE;
    uartInit->Init.BaudRate = 115200;
    uartInit->Init.WordLength = UART_WORDLENGTH_8B;
    uartInit->Init.StopBits = UART_STOPBITS_1;
    uartInit->Init.Parity = UART_PARITY_NONE;
    uartInit->Init.Mode = UART_MODE_TX_RX;
    uartInit->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartInit->Init.OverSampling = UART_OVERSAMPLING_16;
    uartInit->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uartInit->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uartInit->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(uartInit) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingCRC(MCUDef *mcu) {
    mcu->handlers.crc = &crcHandler;
    CRC_HandleTypeDef *crcInit = (CRC_HandleTypeDef *) mcu->handlers.crc;
    crcInit->Instance = CRC;
    crcInit->InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES; // uint8_t
    crcInit->Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    crcInit->Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    crcInit->Init.CRCLength = CRC_POLYLENGTH_32B;
    crcInit->Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
    crcInit->Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;

    if (HAL_CRC_Init(&crcHandler) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingRNG(MCUDef *mcu) {
    mcu->handlers.rng = (void *) &rngHandler;
    RNG_HandleTypeDef *rngInit = (RNG_HandleTypeDef *) mcu->handlers.rng;
    rngInit->Instance = RNG;
    rngInit->Init.ClockErrorDetection = RNG_CED_ENABLE;

    if (HAL_RNG_Init(rngInit) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int settingWDT(MCUDef *mcu) {
    mcu->handlers.wdt = (void *) &wdtHandler;
    IWDG_HandleTypeDef *wdtInit = (IWDG_HandleTypeDef *) mcu->handlers.wdt;
    wdtInit->Instance = IWDG;
    wdtInit->Init.Prescaler = IWDG_PRESCALER_8;
    wdtInit->Init.Reload = 0x0FFF;
    wdtInit->Init.Window = 0x0FFF; // window mode is turn OFF

    if (HAL_IWDG_Init(wdtInit) != HAL_OK)
        return SETTING_ERROR;

    return SETTING_SUCCESS;
}

static int turnOnInterrupts(MCUDef *mcu) {
    if (HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *) mcu->measTimer.handler) == HAL_OK &&
        HAL_UART_Receive_IT((UART_HandleTypeDef *) mcu->uart1.obj, &mcu->uart1.rxByte, 1U) == HAL_OK &&
        HAL_UART_Receive_IT((UART_HandleTypeDef *) mcu->uart2.obj, &mcu->uart2.rxByte, 1U) == HAL_OK) {
        return SETTING_SUCCESS;
    }

    return SETTING_ERROR;
}

int initialization(MCUDef *mcu) {
    if (SETTING_SUCCESS != settingSystemClock()) {

    } else if (SETTING_SUCCESS != settingGPIO()) {

    } else if (SETTING_SUCCESS != settingTimer(&mcu->measTimer)) {

    } else if (SETTING_SUCCESS != settingPWM(&mcu->pwmTimer)) {

    } else if (SETTING_SUCCESS != settingADC(&mcu->adc)) {

    } else if (SETTING_SUCCESS != settingGenerator(&mcu->generator)) {

    } else if (SETTING_SUCCESS != settingComp(&mcu->comp)) {

    } else if (SETTING_SUCCESS != settingUART(mcu)) {

    } else if (SETTING_SUCCESS != settingCRC(mcu)) {

    } else if (SETTING_SUCCESS != settingRNG(mcu)) {

    } else if (SETTING_SUCCESS != settingWDT(mcu)) {

    } else {
        return turnOnInterrupts(mcu);
    }

    return SETTING_ERROR;
}
