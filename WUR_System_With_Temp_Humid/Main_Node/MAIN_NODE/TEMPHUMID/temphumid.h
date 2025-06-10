#ifndef TEMP_HUMID_H
#define TEMP_HUMID_H

#include "stm32wl3x_hal.h"
#include "stm32wl3x_nucleo.h"
#include <stdio.h>

HAL_StatusTypeDef Si7021_Init(void);
HAL_StatusTypeDef Si7021_ReadHumidity(float *humidity);
HAL_StatusTypeDef Si7021_ReadTempAndHumidity(float *temperature, float *humidity);

#endif
