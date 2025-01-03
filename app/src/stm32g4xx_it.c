/**
  ******************************************************************************
  * @file    Templates/Src/stm32g4xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "stm32g4xx_it.h"

#include "variables.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void) {
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void) {
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void) {
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void) {
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void) {
    Mcu.flags.isSysTickTriggered = true;
    Mcu.runtime++;

    HAL_IncTick();
}

/******************************************************************************/
/*                 STM32G4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32g4xxxx.s).                                             */
/******************************************************************************/

/**
 * @brief ADC interrupt handler function
 */
void ADC1_2_IRQHandler(void) {
    HAL_ADC_IRQHandler((ADC_HandleTypeDef *) Mcu.adc.handle);
}

/**
 * @brief Basic timer interrupt handler function
 */
void TIM6_DAC_IRQHandler(void) {
}

/**
 * @brief Basic timer interrupt handler function
 */
void TIM7_IRQHandler(void) {
}

/**
 * @brief General purpose timer interrupt handler function
 */
void TIM1_BRK_TIM15_IRQHandler(void) {
    HAL_TIM_IRQHandler((TIM_HandleTypeDef *) Mcu.measTimer.handle);
}

/**
 * @brief Comparator interrupt handler function
 */
void COMP1_2_3_IRQHandler(void) {
    HAL_COMP_IRQHandler((COMP_HandleTypeDef *) Mcu.comp.handle);
}

/**
 * @brief USART interrupt handler function
 */
void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler((UART_HandleTypeDef *) Mcu.uart1.handle);
}

/**
 * @brief USART interrupt handler function
 */
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler((UART_HandleTypeDef *) Mcu.uart2.handle);
}
