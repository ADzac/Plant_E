/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32wl3x_hal.h"
#include "stm32wl3x_ll_bus.h"
#include "stm32wl3x_ll_cortex.h"
#include "stm32wl3x_ll_rcc.h"
#include "stm32wl3x_ll_system.h"
#include "stm32wl3x_ll_utils.h"
#include "stm32wl3x_ll_gpio.h"
#include "stm32wl3x_ll_pwr.h"
#include "stm32wl3x_ll_dma.h"
#include "stm32wl3x_ll_usart.h"
#include "stm32wl3x_ll_lpawur.h"
#include "stm32wl3x_nucleo.h"
#include<stdio.h>
#include "math.h"


#include "app_conf.h"
#include "crc_4wkup_rf.h"
#include "stm32_lpm.h"
#include "temphumid.h"
/* Private includes ----------------------------------------------------------*/

void Error_Handler(void);
#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
