#include "main.h"


uint8_t vectcTxBuffV2[15];

uint8_t mode = 1; // RX = 1 , TX = 0
uint8_t packet_Received = 0;

float temperature=0;
float humidity = 0;


SMRSubGConfig MRSUBG_RadioInitStruct;
MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct;
SLPAWUR_RFConfig LPAWUR_RadioInitStruct;
SLPAWUR_FrameInit LPAWUR_FrameInitStruct;

/*----------------------------------------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_MRSUBG_Init(void);
static void MX_GPIO_Init(void);
void UTIL_LPM_Init(void);
void RX_TX_Init(void);
static void MX_LPAWUR_Init(void);
void UTIL_LPM_Init( void );
void MX_I2C2_Init(void);
/*----------------------------------------------------------------------------*/

int main(void)
{
	HAL_Init();

	SystemClock_Config();
	PeriphCommonClock_Config();
	MX_GPIO_Init();
	MX_MRSUBG_Init();
	MX_I2C2_Init();
	MX_LPAWUR_Init();
	UTIL_LPM_Init();
	RX_TX_Init();


//	Packet myPacket;



	printf("STM32WL3 LPAWUR - Transmitter example.\n\r");

	while (1)
	{
		GotoRx(&packet_Received,vectcTxBuffV2);
//		printf("Mode after receiving %d \r\n",mode);
		printf("Number of packet received %d \r\n",packet_Received);

		MX_APPE_Idle();
	}
}

void RX_TX_Init(void){
	COM_InitTypeDef COM_Init = {0};
	COM_Init.BaudRate= 115200;
	COM_Init.HwFlowCtl = COM_HWCONTROL_NONE;
	COM_Init.WordLength = COM_WORDLENGTH_8B;
	COM_Init.Parity = COM_PARITY_NONE;
	COM_Init.StopBits = COM_STOPBITS_1;
	BSP_COM_Init(COM1, &COM_Init);
	UTIL_LPM_Init();
	BSP_LED_Init(LD1);
	BSP_LED_Init(LD2);
	BSP_LED_Init(LD3);
	BSP_PB_Init(B2, BUTTON_MODE_GPIO); // if needed
	HAL_MRSubG_PktBasicSetPayloadLength(15);
	LL_MRSubG_PacketHandlerManchesterType(MANCHESTER_TYPE0);
	__HAL_MRSUBG_SET_TX_MODE(TX_NORMAL);
	__HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t)&vectcTxBuffV2);
}

static void MX_MRSUBG_Init(void)
{
	/* Configures the radio parameters */
	MRSUBG_RadioInitStruct.lFrequencyBase = 865000000;
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
}


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

void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
