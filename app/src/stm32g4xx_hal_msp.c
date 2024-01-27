/**
  ******************************************************************************
  * @file    stm32g4xx_hal_msp_template.c
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
  * @retval None
  */
void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_PWREx_DisableUCPDDeadBattery();
}

/**
  * @brief  DeInitialize the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspDeInit(void) {
}

/**
 * @brief Initialize the base timers, turn ON a clock source and setup interrupt vector
 * @param htim is the pointer to the data structure of the base timer handler.
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        __HAL_RCC_TIM7_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
    }
}

/**
 * @brief DeInitialize the base timers
 * @param htim is the pointer to the data structure of the base timer handler.
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        __HAL_RCC_TIM7_FORCE_RESET();
        __HAL_RCC_TIM7_RELEASE_RESET();
        __HAL_RCC_TIM7_CLK_DISABLE();

        HAL_NVIC_DisableIRQ(TIM7_IRQn);
    }
}

/**
 * @brief Initialize the UART interfaces, turn ON a clock source, setup GPIO and interrupt vector
 * @param huart is the pointer to the data structure of the UART interface handler.
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    GPIO_InitTypeDef gpioInit = {0};

    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        gpioInit.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        gpioInit.Mode = GPIO_MODE_AF_PP;
        gpioInit.Pull = GPIO_PULLUP;
        gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
        gpioInit.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOC, &gpioInit);

        HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpioInit.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        gpioInit.Mode = GPIO_MODE_AF_PP;
        gpioInit.Pull = GPIO_PULLUP;
        gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
        gpioInit.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &gpioInit);

        HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

/**
 * @brief DeInitialize the UART interfaces
 * @param huart is the pointer to the data structure of the UART interface handler.
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);

        HAL_NVIC_DisableIRQ(USART1_IRQn);
    } else if (huart->Instance == USART2) {
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        __HAL_RCC_USART2_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
}

/**
 * @brief Initialize the CRC module, turn ON a clock source
 * @param hcrc is the pointer to the data structure of the CRC module.
 */
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc) {
    if (hcrc->Instance == CRC) {
        __HAL_RCC_CRC_CLK_ENABLE();
    }
}

/**
 * @brief DeInitialize the CRC module
 * @param hcrc is the pointer to the data structure of the CRC module.
 */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc) {
    if (hcrc->Instance == CRC) {
        __HAL_RCC_CRC_FORCE_RESET();
        __HAL_RCC_CRC_RELEASE_RESET();
        __HAL_RCC_CRC_CLK_DISABLE();
    }
}

/**
 * @brief Initialize the RNG module, turn ON a clock source
 * @param hrng is the pointer to the data structure of the RNG module.
 */
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng) {
    if (hrng->Instance == RNG) {
        RCC_PeriphCLKInitTypeDef clockInit = {0};
        clockInit.PeriphClockSelection = RCC_PERIPHCLK_RNG;
        clockInit.RngClockSelection = RCC_RNGCLKSOURCE_PLL;

        if (HAL_RCCEx_PeriphCLKConfig(&clockInit) == HAL_OK)
            __HAL_RCC_RNG_CLK_ENABLE();
    }
}

/**
 * @brief DeInitialize the RNG module
 * @param hrng is the pointer to the data structure of the RNG module.
 */
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng) {
    if (hrng->Instance == RNG) {
        __HAL_RCC_RNG_FORCE_RESET();
        __HAL_RCC_RNG_RELEASE_RESET();
        __HAL_RCC_RNG_CLK_DISABLE();
    }
}
