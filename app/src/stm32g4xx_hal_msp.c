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
        HAL_NVIC_DisableIRQ(TIM7_IRQn);
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
    }
}
