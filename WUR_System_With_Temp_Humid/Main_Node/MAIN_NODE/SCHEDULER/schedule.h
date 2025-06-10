/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wl3x_hal.h"
#include "stm32wl3x_nucleo.h"
#include <stdio.h>
#include "stm32wl3x_ll_pwr.h"

void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void configRTCWakeupTimer(uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
