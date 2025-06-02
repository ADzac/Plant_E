/* Includes ------------------------------------------------------------------*/
#include "main.h"

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

COM_InitTypeDef BspCOMInit;
static uint32_t delay = 2000; // Changed to 2 seconds for sensor readings

I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef Si7021_Init(void);
HAL_StatusTypeDef Si7021_ReadHumidity(float *humidity);
HAL_StatusTypeDef Si7021_ReadTempAndHumidity(float *temperature, float *humidity);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  uint8_t cmd = SI7021_READ_TEMP_FROM_PREV_RH;
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
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(B1, BUTTON_MODE_EXTI);
  BSP_PB_Init(B2, BUTTON_MODE_EXTI);
  BSP_PB_Init(B3, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  printf("Welcome to STM32 world with Si7021 sensor!\n\r");

  /* -- Initialize Si7021 sensor ---- */
  while (Si7021_Init() != HAL_OK) {
    printf("Failed to initialize Si7021 sensor!\n\r");
    BSP_LED_On(LED_RED);
  }
  printf("Si7021 sensor initialized successfully!\n\r");
      BSP_LED_On(LED_GREEN);

  while (1)
  {
    /* -- Read Si7021 sensor data ---- */
    float temperature, humidity;

    if (Si7021_ReadTempAndHumidity(&temperature, &humidity) == HAL_OK) {
      printf("Temperature: %.2f°C, Humidity: %.2f \n\r", temperature, humidity);
      BSP_LED_Toggle(LED_GREEN);
    } else {
      printf("Error reading Si7021 sensor!\n\r");
      BSP_LED_Toggle(LED_RED);
    }
    HAL_Delay(delay);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitStruct structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource and SYSCLKDivider
  */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV2;
  PeriphClkInitStruct.KRMRateMultiplier = 2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C1 */
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_LCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_2);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief EXTI line detection callback.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
    case B1_PIN:
      /* Change the period to 500 ms for faster readings */
      delay = 500;
      break;
    case B2_PIN:
      /* Change the period to 2000 ms (default) */
      delay = 2000;
      break;
    case B3_PIN:
      /* Change the period to 5000 ms for slower readings */
      delay = 5000;
      break;
    default:
      break;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
