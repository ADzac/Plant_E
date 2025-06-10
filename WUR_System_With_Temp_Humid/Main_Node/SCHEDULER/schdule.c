/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @author  GPM WBL Application Team
  * @brief   This code implements a bidirectional point to point communication.
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "schedule.h"

#define DISCOVERY 0
#define DATAREQ 1
#define ALERT 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

void MX_RTC_Init(void)
{
    /* RTC clock enable */
    __HAL_RCC_RTC_CLK_ENABLE();

    __HAL_RCC_CLEAR_IT(RCC_IT_RTCRSTRELRDY);
    /* Force RTC peripheral reset */
    __HAL_RCC_RTC_FORCE_RESET();
    __HAL_RCC_RTC_RELEASE_RESET();
    /* Check if RTC Reset Release flag interrupt occurred or not */
    while(__HAL_RCC_GET_IT(RCC_IT_RTCRSTRELRDY) == 0)
    {
    }
    __HAL_RCC_CLEAR_IT(RCC_IT_RTCRSTRELRDY);

    /* Initialize RTC */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;    // Changed from 0 to 127
    hrtc.Init.SynchPrediv = 255;     // Changed from 0 to 255
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        printf("Error\r\n");
    }

    /* Configure the NVIC for RTC */
    NVIC_SetPriority(RTC_IRQn, 0);
    NVIC_EnableIRQ(RTC_IRQn);
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Configures the RTC wakeup timer, to wakeup the device from DEEPSTOP
  *         at specified timeout
  * @param  timeout wakeup timeout expressed in ms
  */
void configRTCWakeupTimer(uint32_t timeout)
{
  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ((timeout/1000)*2048), RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
	  printf("Error\r\n");
  }
}

