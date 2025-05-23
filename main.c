#include "main.h"
#include "crc_4wkup_rf.h"
#include "stm32_lpm.h"
/* Private includes ----------------------------------------------------------*/
#define MSG_SIZE 15
uint8_t vectcTxBuffV2[15];
#define PAYLOAD_LEN 7
#define MIN(a,b)                        (((a) < (b))? (a) : (b))
uint8_t LPAWUR_Payload[8];
int32_t  rssi_min = 0 ;
int32_t rssi_max = -150 ;
uint8_t mode = 0; // Rx = 1 , Tx = 0
uint8_t packet_Received = 0;
uint8_t checkForID = 0;
SMRSubGConfig MRSUBG_RadioInitStruct;
MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct;
SLPAWUR_RFConfig LPAWUR_RadioInitStruct;
SLPAWUR_FrameInit LPAWUR_FrameInitStruct;
/* USER CODE BEGIN PV */
__IO uint32_t aRandom16bit[100];
__IO int32_t rssi = 0;
uint8_t LPAWUR_Payload[8];
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_MRSUBG_Init(void);
/* USER CODE BEGIN PFP */
static void RandomNumbersGeneration(uint8_t j);
void CreateLPAWURFrameV2(uint8_t j,uint8_t aRandom16bit);
void EvaluateCrc(uint8_t * LPAWUR_payload);
void UTIL_LPM_EnterLowPower(void);
void UTIL_LPM_Init(void);
void MX_APPE_Process(void);
void GotoRx(void);
void MX_APPE_Idle(void);
static void MX_LPAWUR_Init(void);
void UTIL_LPM_Init( void );
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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


SystemClock_Config();


PeriphCommonClock_Config();


MX_GPIO_Init();
MX_MRSUBG_Init();
MX_LPAWUR_Init();
UTIL_LPM_Init();
COM_InitTypeDef COM_Init = {0};
COM_Init.BaudRate= 115200;
COM_Init.HwFlowCtl = COM_HWCONTROL_NONE;
COM_Init.WordLength = COM_WORDLENGTH_8B;
COM_Init.Parity = COM_PARITY_NONE;
COM_Init.StopBits = COM_STOPBITS_1;
BSP_COM_Init(COM1, &COM_Init);
UTIL_LPM_Init();
/* USER CODE BEGIN APPE_Init_2 */
BSP_LED_Init(LD2);
BSP_LED_Init(LD3);
/* Init SW2 User Button */
BSP_PB_Init(B2, BUTTON_MODE_GPIO);
/* Payload length config */
HAL_MRSubG_PktBasicSetPayloadLength(15);
/* Set Manchester Coding Type */
LL_MRSubG_PacketHandlerManchesterType(MANCHESTER_TYPE0);
/* Set TX Mode to Normal Mode*/
__HAL_MRSUBG_SET_TX_MODE(TX_NORMAL);
__HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t)&vectcTxBuffV2);
/* Create the data to transmit */
printf("STM32WL3 LPAWUR - Transmitter example.\n\r");
/* Infinite loop */
/* USER CODE BEGIN WHILE */
for (uint16_t j = 0;j<200;j++){
	 if (mode == 0){
	 	RandomNumbersGeneration(j);
	 	mode = 1;
	 	}
	 	else{
	 		GotoRx();
			if (mode == 1){
				MX_APPE_Idle();
			}
	   }
	 	printf("Number of packet received %d \r\n",packet_Received);
}
while (1)
{
//	if (mode == 0){
//	RandomNumbersGeneration();
//	mode = 1;
//
//	}
//	else{
//		GotoRx();
//		if (mode == 0){
//			packet_Received++;;
//		}
//		else{
//			MX_APPE_Idle();
//		}
//  }
//	printf("Number of packet received %d \r\n",packet_Received);
}
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
* in the RCC_OscInitTypeDef structure.
*/
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.LSEState = RCC_LSE_ON;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
{
  Error_Handler();
}
/** Configure the SYSCLKSource and SYSCLKDivider
*/
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_RC64MPLL;
RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;
if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1) != HAL_OK)
{
  Error_Handler();
}
}
void PeriphCommonClock_Config(void)
{
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
/** Initializes the peripherals clock
*/
PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;
PeriphClkInitStruct.KRMRateMultiplier = 4;
if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
{
  Error_Handler();
}
}
/**
* @brief MRSUBG Initialization Function
* @param None
* @retval None
*/
static void MX_MRSUBG_Init(void)
{
/* Configures the radio parameters */
MRSUBG_RadioInitStruct.lFrequencyBase = 868000000;
MRSUBG_RadioInitStruct.xModulationSelect = MOD_OOK;
MRSUBG_RadioInitStruct.lDatarate = 2000;
MRSUBG_RadioInitStruct.lFreqDev = 20000;
MRSUBG_RadioInitStruct.lBandwidth = 50000;
MRSUBG_RadioInitStruct.dsssExp = 0;
MRSUBG_RadioInitStruct.outputPower = 14;
MRSUBG_RadioInitStruct.PADrvMode = PA_DRV_TX_HP;
HAL_MRSubG_Init(&MRSUBG_RadioInitStruct);
/* Configures the packet parameters */
MRSUBG_PacketSettingsStruct.PreambleLength = 0;
MRSUBG_PacketSettingsStruct.PostambleLength = 0;
MRSUBG_PacketSettingsStruct.SyncLength = 0;
MRSUBG_PacketSettingsStruct.SyncWord = 0x88888888;
MRSUBG_PacketSettingsStruct.FixVarLength = FIXED;
MRSUBG_PacketSettingsStruct.PreambleSequence = PRE_SEQ_0101;
MRSUBG_PacketSettingsStruct.PostambleSequence = POST_SEQ_0101;
MRSUBG_PacketSettingsStruct.CrcMode = PKT_NO_CRC;
MRSUBG_PacketSettingsStruct.Coding = CODING_MANCHESTER;
MRSUBG_PacketSettingsStruct.DataWhitening = DISABLE;
MRSUBG_PacketSettingsStruct.LengthWidth = BYTE_LEN_1;
MRSUBG_PacketSettingsStruct.SyncPresent = DISABLE;
HAL_MRSubG_PacketBasicInit(&MRSUBG_PacketSettingsStruct);
}
void CreateLPAWURFrameV2(uint8_t j,uint8_t aRandom16bit) {
/* bit sync */
for(int i = 0; i<5; i++)
  vectcTxBuffV2[i] = 0x00;
/* Frame sync */
vectcTxBuffV2[5] = 0x99;
/* Payload */
vectcTxBuffV2[6] = j;
vectcTxBuffV2[7] = 0x00;
vectcTxBuffV2[8] = 0x00;
vectcTxBuffV2[9] = 0x00;
vectcTxBuffV2[10] = 0x00;
vectcTxBuffV2[11] = 0x00;
vectcTxBuffV2[12] = aRandom16bit;
printf("%d ",vectcTxBuffV2[6]);
printf("%d \r\n",vectcTxBuffV2[12]);
/* CRC */
EvaluateCrc(&vectcTxBuffV2[6]);
}
static void RandomNumbersGeneration(uint8_t j)
{
	aRandom16bit[0] = 5;
	CreateLPAWURFrameV2(j,aRandom16bit[0]);
	HAL_Delay(1000);
	MX_APPE_Process();
}
static void MX_LPAWUR_Init(void)
{
LPAWUR_RadioInitStruct.EnergyDetectorIcal = ED_ICAL_VBAT_3_25_TO_3_50;
LPAWUR_RadioInitStruct.ClockDivider = 7;
LPAWUR_RadioInitStruct.EnergyDetectorSwitch = DISABLE;
LPAWUR_RadioInitStruct.AgcResetMode = AGC_RESET_MODE_NEVER;
LPAWUR_RadioInitStruct.AgcHoldMode = AGC_HOLD_AFTER_PREAMBLE;
LPAWUR_RadioInitStruct.AgcMode = AGC_MODE_OFF;
LPAWUR_RadioInitStruct.AgcHiLvl = AGC_VBAT_0800;
LPAWUR_RadioInitStruct.DCCurrentSubtraction = ENABLE;
LPAWUR_RadioInitStruct.AgcLoLvl = AGC_LOW_0;
HAL_LPAWUR_RFConfigInit(&LPAWUR_RadioInitStruct);
LPAWUR_FrameInitStruct.TRecAlgoSel = TWO_STEPS;
LPAWUR_FrameInitStruct.SlowClkCyclePerBitCnt = 16;
LPAWUR_FrameInitStruct.PayloadLength = 7;
LPAWUR_FrameInitStruct.SyncThr = 16;
LPAWUR_FrameInitStruct.SyncLength = 0;
LPAWUR_FrameInitStruct.PreambleThrCnt = 0x3C;
LPAWUR_FrameInitStruct.PreambleEnable = ENABLE;
LPAWUR_FrameInitStruct.FrameSyncCntTimeout = 0x60;
LPAWUR_FrameInitStruct.FrameSyncPattenHigh = 0x00;
LPAWUR_FrameInitStruct.FrameSyncPatternLow = 38550;
LPAWUR_FrameInitStruct.KpGain = 6;
LPAWUR_FrameInitStruct.KiGain = 10;
HAL_LPAWUR_FrameInit(&LPAWUR_FrameInitStruct);
LL_LPAWUR_SetState(ENABLE);
/* USER CODE BEGIN LPAWUR_Init 2 */
/* USER CODE END LPAWUR_Init 2 */
}
void UpdateRssiStats(int32_t rssi)
{
	      rssi_min = (rssi < rssi_min) ? rssi : rssi_min;
	      rssi_max = (rssi > rssi_max) ? rssi : rssi_max;
	      printf("Current RSSI: %ld dBm | Min: %ld dBm | Max: %ld dBm\r\n", rssi, rssi_min, rssi_max);
}
void GotoRx(void)
{
/* Wakeup source configuration */
 HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
 uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();
 /* Wakeup on LPAWUR Frame Valid */
 if (wakeupSource & PWR_WAKEUP_LPAWUR)
 {
	BSP_LED_On(LD2);
	HAL_LPAWUR_GetPayload(LPAWUR_Payload);
	printf("LPAWUR data received: [ ");
	for(uint8_t i=0;i<PAYLOAD_LEN;i++)
	{
	  printf("%x",LPAWUR_Payload[i]);
	  checkForID = LPAWUR_Payload[i];
	}
	printf(" ]\n\r");
	if (checkForID == 1){
		packet_Received++;
	}
	HAL_LPAWUR_ClearStatus();
	LL_LPAWUR_SetState(ENABLE);
	printf("Changing to TX \r\n");
	mode = 0;
	HAL_Delay(500);
	BSP_LED_Off(LD2);
 }
}
/* USER CODE END MX_APPE_Process_2 */
#if (CFG_LPM_SUPPORTED == 1)
static PowerSaveLevels App_PowerSaveLevel_Check(void)
{
PowerSaveLevels output_level = POWER_SAVE_LEVEL_DEEPSTOP_NOTIMER;
/* USER CODE BEGIN App_PowerSaveLevel_Check_1 */
/* USER CODE END App_PowerSaveLevel_Check_1 */
return output_level;
}
#endif
__weak PowerSaveLevels HAL_MRSUBG_TIMER_PowerSaveLevelCheck()
{
return POWER_SAVE_LEVEL_DEEPSTOP_TIMER;
}
void MX_APPE_Idle(void)
{
#if (CFG_LPM_SUPPORTED == 1)
PowerSaveLevels app_powerSave_level, vtimer_powerSave_level, final_level;
app_powerSave_level = App_PowerSaveLevel_Check();
if(app_powerSave_level != POWER_SAVE_LEVEL_DISABLED)
{
  vtimer_powerSave_level = HAL_MRSUBG_TIMER_PowerSaveLevelCheck();
  final_level = (PowerSaveLevels)MIN(vtimer_powerSave_level, app_powerSave_level);
  switch(final_level)
  {
  case POWER_SAVE_LEVEL_DISABLED:
    /* Not Power Save device is busy */
    return;
    break;
  case POWER_SAVE_LEVEL_SLEEP:
    UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
    UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
    break;
  case POWER_SAVE_LEVEL_DEEPSTOP_TIMER:
    UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
    UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
    break;
  case POWER_SAVE_LEVEL_DEEPSTOP_NOTIMER:
    UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
    UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
    break;
  }
  UTIL_LPM_EnterLowPower();
}
#endif /* CFG_LPM_SUPPORTED */
}
void MX_APPE_Process(void)
{
  BSP_LED_On(LD3);
  __HAL_MRSUBG_STROBE_CMD(CMD_TX);
  /* Wait for TX done */
  while((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS() & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {};
  /* Clear the IRQ flag */
  __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);
  BSP_LED_Off(LD3);
  LL_LPAWUR_SetState(ENABLE);
}
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */
/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
void Error_Handler(void)
{
/* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
while(1)
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



