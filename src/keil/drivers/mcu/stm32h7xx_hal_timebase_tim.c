/**
  ******************************************************************************
  * @file    stm32h7xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef        htim7;
static uint32_t systemTIMClockFrequency;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static bool FindBestTIMParam(const uint32_t timClk, uint32_t *uwPrescalerValue, uint32_t *uwPeriodValue)
{
  for ((*uwPrescalerValue) = 0; *uwPrescalerValue <= 0xFFFF; (*uwPrescalerValue)++)
  {
    *uwPeriodValue = ((timClk / (*uwPrescalerValue + 1)) / 1000) - 1;
    if (((timClk / (*uwPrescalerValue + 1)) % 1000) == 0 && *uwPeriodValue <= 0xFFFF)
    {
      return true;
    }
  }

  return false;
}

/**
  * @brief  This function configures the TIM7 as a time base source.
  *         The time source is configured to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock, uwAPB1Prescaler;
  uint32_t              uwPrescalerValue, uwPeriodValue;
  uint32_t              pFLatency;
  HAL_StatusTypeDef     status;

  /* Enable TIM7 clock */
  __HAL_RCC_TIM7_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Get APB1 prescaler */
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;

  /* Compute TIM7 clock */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
  }

  if (!FindBestTIMParam(uwTimclock, &uwPrescalerValue, &uwPeriodValue)) {
    while (1);
  }
  systemTIMClockFrequency = uwTimclock / (uwPrescalerValue + 1);

  /* Initialize TIM7 */
  htim7.Instance = TIM7;

  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM7CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim7.Init.Period = uwPeriodValue;
  htim7.Init.Prescaler = uwPrescalerValue;
  htim7.Init.ClockDivision = 0U;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  status = HAL_TIM_Base_Init(&htim7);
  if (status == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    status = HAL_TIM_Base_Start_IT(&htim7);
    if (status == HAL_OK)
    {
      /* Enable the TIM7 global Interrupt */
      HAL_NVIC_EnableIRQ(TIM7_IRQn);
      /* Configure the TIM7 IRQ priority */
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* Enable the TIM7 global Interrupt */
        HAL_NVIC_SetPriority(TIM7_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

 /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM7 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM7 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM7 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM7 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
}

uint32_t Get_SystemTIMClockFrequency(void)
{
  return systemTIMClockFrequency;
}
