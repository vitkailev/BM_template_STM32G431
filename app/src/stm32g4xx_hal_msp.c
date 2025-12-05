/**
  ******************************************************************************
  * @file    stm32g4xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
  *          This file template is located in the HAL folder and should be copied
  *          to the user folder.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the Global MSP.
  * @param  None
  */
void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_PWREx_DisableUCPDDeadBattery();
}

/**
  * @brief  DeInitialize the Global MSP.
  * @param  None
  */
void HAL_MspDeInit(void) {
}

/**
 * @brief Initialize the base timers, turn ON a clock source and setup interrupt vector
 * @param htim is the pointer to the data structure of the base timer handle (HAL).
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM15) {
        __HAL_RCC_TIM15_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    }
}

/**
 * @brief DeInitialize the base timers
 * @param htim is the pointer to the data structure of the base timer handle (HAL).
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM15) {
        __HAL_RCC_TIM15_FORCE_RESET();
        __HAL_RCC_TIM15_RELEASE_RESET();
        __HAL_RCC_TIM15_CLK_DISABLE();

        HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
    }
}

/**
 * @brief Initialize the PWM timer mode, turn ON a clock source and setup GPIOs
 * @param htim is the pointer to the data structure of the timer handle (HAL).
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
    GPIO_InitTypeDef gpioInit = {0};

    if (htim->Instance == TIM16) {
        __HAL_RCC_TIM16_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpioInit.Pin = GPIO_PIN_4 | GPIO_PIN_6;
        gpioInit.Mode = GPIO_MODE_AF_PP;
        gpioInit.Pull = GPIO_PULLDOWN;
        gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpioInit.Alternate = GPIO_AF1_TIM16;
        HAL_GPIO_Init(GPIOB, &gpioInit);
    }
}

/**
 * @brief DeInitialize the PWM timer mode
 * @param htim is the pointer to the data structure of the timer handle (HAL).
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM16) {
        __HAL_RCC_TIM16_FORCE_RESET();
        __HAL_RCC_TIM16_RELEASE_RESET();
        __HAL_RCC_TIM16_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
    }
}

/**
 * @brief Initialize the ADC module, turn ON a clock source, setup GPIO and interrupt vector
 * @param hadc is the pointer to the data structure of the ADC handle (HAL).
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
    GPIO_InitTypeDef gpioInit = {0};
    RCC_PeriphCLKInitTypeDef clockInit = {0};

    if (hadc->Instance == ADC1) {
        clockInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        clockInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;

        if (HAL_RCCEx_PeriphCLKConfig(&clockInit) == HAL_OK) {
            __HAL_RCC_ADC12_CLK_ENABLE();

            __HAL_RCC_GPIOA_CLK_ENABLE();
            gpioInit.Pin = GPIO_PIN_0 | GPIO_PIN_1;
            gpioInit.Mode = GPIO_MODE_ANALOG;
            gpioInit.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &gpioInit);

            HAL_NVIC_SetPriority(ADC1_2_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
        }
    }
}

/**
 * @brief DeInitialize the ADC module
 * @param hadc is the pointer to the data structure of the ADC handle (HAL).
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        __HAL_RCC_ADC12_FORCE_RESET();
        __HAL_RCC_ADC12_RELEASE_RESET();
        __HAL_RCC_ADC12_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

        HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    }
}

/**
 * @brief Initialize the DAC module, turn ON a clock source, setup GPIO and interrupt vector
 * @param hdac is the pointer to the data structure of the DAC handle (HAL).
 */
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac) {
    GPIO_InitTypeDef gpioInit = {0};

    if (hdac->Instance == DAC1) {
        __HAL_RCC_DAC1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpioInit.Pin = GPIO_PIN_4;
        gpioInit.Mode = GPIO_MODE_ANALOG;
        gpioInit.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpioInit);
    } else if (hdac->Instance == DAC3) {
        __HAL_RCC_DAC3_CLK_ENABLE();
    }
}

/**
 * @brief DeInitialize the DAC module
 * @param hdac is the pointer to the data structure of the DAC handle (HAL).
 */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac) {
    if (hdac->Instance == DAC1) {
        __HAL_RCC_DAC1_FORCE_RESET();
        __HAL_RCC_DAC1_RELEASE_RESET();
        __HAL_RCC_DAC1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
    } else if (hdac->Instance == DAC3) {
        __HAL_RCC_DAC3_FORCE_RESET();
        __HAL_RCC_DAC3_RELEASE_RESET();
        __HAL_RCC_DAC3_CLK_DISABLE();
    }
}

/**
 * @brief Initialize the COMP module, turn ON a clock source, setup GPIO and interrupt vector
 * @param hcomp is the pointer to the data structure of the COMP handle (HAL).
 */
void HAL_COMP_MspInit(COMP_HandleTypeDef *hcomp) {
    GPIO_InitTypeDef gpioInit = {0};

    if (hcomp->Instance == COMP2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpioInit.Pin = GPIO_PIN_7;
        gpioInit.Mode = GPIO_MODE_ANALOG;
        gpioInit.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpioInit);

        HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
    }
}

/**
 * @brief DeInitialize the COMP module
 * @param hcomp is the pointer to the data structure of the COMP handle (HAL).
 */
void HAL_COMP_MspDeInit(COMP_HandleTypeDef *hcomp) {
    if (hcomp->Instance == COMP2) {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

        HAL_NVIC_DisableIRQ(COMP1_2_3_IRQn);
    }
}

/**
 * @brief Initialize the UART interfaces, turn ON a clock source, setup GPIO and interrupt vector
 * @param huart is the pointer to the data structure of the UART handle (HAL).
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    GPIO_InitTypeDef gpioInit = {0};
    RCC_PeriphCLKInitTypeDef clockInit = {0};

    if (huart->Instance == USART1) {
        clockInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        clockInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;

        if (HAL_RCCEx_PeriphCLKConfig(&clockInit) == HAL_OK) {
            __HAL_RCC_USART1_CLK_ENABLE();

            __HAL_RCC_GPIOC_CLK_ENABLE();
            gpioInit.Pin = GPIO_PIN_4 | GPIO_PIN_5;
            gpioInit.Mode = GPIO_MODE_AF_PP;
            gpioInit.Pull = GPIO_PULLUP;
            gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
            gpioInit.Alternate = GPIO_AF7_USART1;
            HAL_GPIO_Init(GPIOC, &gpioInit);

            HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
            HAL_NVIC_EnableIRQ(USART1_IRQn);
        }
    }
}

/**
 * @brief DeInitialize the UART interfaces
 * @param huart is the pointer to the data structure of the UART handle (HAL).
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);

        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
}

/**
 * @brief Initialize the CRC module, turn ON a clock source
 * @param hcrc is the pointer to the data structure of the CRC handle (HAL).
 */
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc) {
    if (hcrc->Instance == CRC) {
        __HAL_RCC_CRC_CLK_ENABLE();
    }
}

/**
 * @brief DeInitialize the CRC module
 * @param hcrc is the pointer to the data structure of the CRC handle (HAL).
 */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc) {
    if (hcrc->Instance == CRC) {
        __HAL_RCC_CRC_FORCE_RESET();
        __HAL_RCC_CRC_RELEASE_RESET();
        __HAL_RCC_CRC_CLK_DISABLE();
    }
}
