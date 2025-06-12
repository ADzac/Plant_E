/*
 * temphumid.c
 *
 *  Created on: Jun 2, 2025
 *      Author: mzakri
 */
/* Includes ------------------------------------------------------------------*/
#include "temphumid.h"

// Si7021 I2C address (7-bit address)
#define SI7021_ADDRESS 0x40

// Si7021 Commands
#define SI7021_MEASURE_HUMIDITY_HOLD_MASTER     0xE5
#define SI7021_MEASURE_HUMIDITY_NO_HOLD_MASTER  0xF5
#define SI7021_MEASURE_TEMP_HOLD_MASTER         0xE3
#define SI7021_MEASURE_TEMP_NO_HOLD_MASTER      0xF3
#define SI7021_READ_TEMP_FROM_PREV_RH           0xE0
#define SI7021_RESET                            0xFE
#define SI7021_READ_USER_REG1                   0xE7
#define SI7021_WRITE_USER_REG1                  0xE6
#define SI7021_READ_HEATER_CONTROL_REG          0x11
#define SI7021_WRITE_HEATER_CONTROL_REG         0x51

uint32_t delay = 2000; // Changed to 2 seconds for sensor readings

I2C_HandleTypeDef hi2c2;

/* Private function prototypes -----------------------------------------------*/

void MX_I2C2_Init(void);

void Error_Handler(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef Si7021_Init(void);
HAL_StatusTypeDef Si7021_ReadHumidity(float *humidity);
HAL_StatusTypeDef Si7021_ReadTempAndHumidity(float *temperature, float *humidity);
/* USER CODE END PFP */

/**
 * @brief Initialize Si7021 sensor
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Si7021_Init(void)
{
  HAL_StatusTypeDef status;
  uint8_t cmd = SI7021_RESET;

  // Send reset command
  status = HAL_I2C_Master_Transmit(&hi2c2, SI7021_ADDRESS << 1, &cmd, 1, 1000);
  if (status != HAL_OK) {
    return status;
  }

  // Wait for reset to complete
  HAL_Delay(15);

  // Check if device is ready
  status = HAL_I2C_IsDeviceReady(&hi2c2, SI7021_ADDRESS << 1, 3, 1000);

  return status;
}

/**
 * @brief Read humidity from Si7021
 * @param humidity: pointer to store humidity value in %RH
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Si7021_ReadHumidity(float *humidity)
{
  HAL_StatusTypeDef status;
  uint8_t cmd = SI7021_MEASURE_HUMIDITY_HOLD_MASTER;
  uint8_t data[2];
  uint16_t hum_raw;

  // Send humidity measurement command
  status = HAL_I2C_Master_Transmit(&hi2c2, SI7021_ADDRESS << 1, &cmd, 1, 1000);
  if (status != HAL_OK) {
    return status;
  }

  // Read humidity data (2 bytes)
  status = HAL_I2C_Master_Receive(&hi2c2, SI7021_ADDRESS << 1, data, 2, 1000);
  if (status != HAL_OK) {
    return status;
  }

  // Convert raw data to humidity
  hum_raw = (data[0] << 8) | data[1];
  *humidity = ((125.0f * hum_raw) / 65536.0f) - 6.0f;

  // Clamp humidity to valid range
  if (*humidity > 100.0f) *humidity = 100.0f;
  if (*humidity < 0.0f) *humidity = 0.0f;

  return HAL_OK;
}

/**
 * @brief Read both temperature and humidity from Si7021
 * @param temperature: pointer to store temperature value in Celsius
 * @param humidity: pointer to store humidity value in %RH
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Si7021_ReadTempAndHumidity(float *temperature, float *humidity)
{
  HAL_StatusTypeDef status;

  // Read humidity first
  status = Si7021_ReadHumidity(humidity);
  if (status != HAL_OK) {
    return status;
  }

  // Read temperature from previous humidity measurement (more efficient)
  uint8_t cmd = SI7021_MEASURE_TEMP_HOLD_MASTER;
  uint8_t data[2];
  uint16_t temp_raw;

  status = HAL_I2C_Master_Transmit(&hi2c2, SI7021_ADDRESS << 1, &cmd, 1, 1000);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_I2C_Master_Receive(&hi2c2, SI7021_ADDRESS << 1, data, 2, 1000);
  if (status != HAL_OK) {
    return status;
  }

  // Convert raw data to temperature
  temp_raw = (data[0] << 8) | data[1];
  *temperature = ((175.72f * temp_raw) / 65536.0f) - 46.85f;

  return HAL_OK;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
