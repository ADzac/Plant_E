/*
 * txrx.c
 *
 *  Created on: Jun 6, 2025
 *      Author: mzakri
 */
#include "transmission.h"

#define MSG_SIZE
#define PAYLOAD_LEN 7
#define MIN(a,b)                        (((a) < (b))? (a) : (b))

uint8_t vectcTxBuff[15];
uint8_t LPAWUR_Payload[8];

uint8_t m; // RX = 1 , TX = 0
uint8_t PR;
uint8_t ID = 1;
uint8_t checkForID = 5;
uint8_t jumpNode = 0; // original sender if jump needed

uint8_t LPAWUR_Payload[8];
int16_t rssi = 0;
int16_t  rssi_min = 0 ;
int16_t rssi_max = -150 ;

Packet p;

float datas[2];

float temp;
float humid;

//------Temp and humid----------------------------------------------------------------------------------

void TempANDHumidSensor(){
	while (Si7021_Init() != HAL_OK) {
		//printf("Failed to initialize Si7021 sensor!\n\r");
		BSP_LED_On(LED_RED);
	}
		//printf("Si7021 sensor initialized successfully!\n\r");
		BSP_LED_On(LED_GREEN);
		HAL_Delay(100);
		BSP_LED_Off(LED_GREEN);
	/* -- Read Si7021 sensor data ---- */

	if (Si7021_ReadTempAndHumidity(&temp, &humid) == HAL_OK) {
		printf("temp: %.2f°C, humid: %.2f \n\r", temp, humid);
		BSP_LED_On(LED_GREEN);
		HAL_Delay(100);
		BSP_LED_Off(LED_GREEN);
	} else {
		printf("Error reading Si7021 sensor!\n\r");
		BSP_LED_On(LED_RED);
	}
}

//---------------------------------------------------------------------------------------------------


// TX SETUP ----------------------------------------------------------------------------------------------

void RandomNumbersGeneration(Packet* p,uint8_t j,uint8_t* vectcTxBuff)
{
	HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_RTC, PWR_WUP_RISIEDG);

	/* Configures the Button 2 as interrupt wakeup source */
	HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN11, PWR_WUP_FALLEDG);

	uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

	if (wakeupSource & PWR_WAKEUP_RTC){
		CreateLPAWURFrameV2(p,j,vectcTxBuff);
		HAL_Delay(1000);
		MX_APPE_Process();
		printf("Packet sent \r\n");
	}

}

void CreateLPAWURFrameV2(Packet* packet, uint8_t j, uint8_t* vectcTxBuff) {
    /* bit sync */
	printf("Transmission type : %d \r\n",packet->TransmissionType);

    for (int i = 0; i < 5; i++)
        vectcTxBuff[i] = 0x00;

    /* Frame sync */
    vectcTxBuff[5] = 0x99;

    TempANDHumidSensor();
    packet->Temperature = temp;
    packet->Humidity = humid;

    /* Fill Tx buffer with payload */
    vectcTxBuff[6]  = packet->ID;
    vectcTxBuff[7]  = packet->Destination;
    vectcTxBuff[8]  = packet->TransmissionType;
    vectcTxBuff[9]  = (uint8_t)round(packet->Temperature);
    vectcTxBuff[10] = (uint8_t)round(packet->Humidity);
    vectcTxBuff[11] = packet->Dunno;
    vectcTxBuff[12] = j;

    /* CRC */
    EvaluateCrc(&vectcTxBuff[6]);
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

//---------------------------------------------------------------------------------------------------

// RX SETUP ----------------------------------------------------------------------------------------------

void UpdateRssiStats(int16_t rssi, int print_stats)
{
	  rssi_min = (rssi < rssi_min) ? rssi : rssi_min;
	  rssi_max = (rssi > rssi_max) ? rssi : rssi_max;
	  if (print_stats == 1)
	  {printf("Current RSSI: %d dBm | MIN : %d dBm | MAX : %d dBm\r\n", rssi, rssi_min, rssi_max);}
}
void GotoRx(uint8_t* PR)
{
/* Wakeup source configuration */
 HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
 uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();
 uint8_t compareID = 0;
 printf("Waiting for responses \r\n");
 /* Wakeup on LPAWUR Frame Valid */
 if (wakeupSource & PWR_WAKEUP_LPAWUR)
 {
	BSP_LED_On(LD2);
	HAL_LPAWUR_GetPayload(LPAWUR_Payload);
	rssi = HAL_MRSubG_GetRssidBm();
	//UpdateRssiStats(rssi,1);
	if (ID == LPAWUR_Payload[1]){
		  printf("My packet NGL\r\n");
	}
	printf("LPAWUR data received: [ ");

	for(uint8_t i=0;i<PAYLOAD_LEN;i++)
	{
	  if (i == 0){
		  jumpNode = LPAWUR_Payload[i];
		  compareID = LPAWUR_Payload[i];
	  }
	  if (i == 3){
		  temp = LPAWUR_Payload[i];
	  }
	  if (i == 4){
		  humid = LPAWUR_Payload[i];
	  }
	  printf("%x",LPAWUR_Payload[i]);
	}
	printf(" ]\n\r");
	if (compareID == checkForID){
		(*PR)++;
	}
	printf("temp: %.2f°C, humid: %.2f \n\r", temp, humid);

	HAL_LPAWUR_ClearStatus();
	LL_LPAWUR_SetState(ENABLE);
	printf("Changing to TX \r\n");
	HAL_Delay(1000);
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

//---------------------------------------------------------------------------------------------------
