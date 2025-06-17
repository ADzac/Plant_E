
/*
 * txrx.c
 *
 *  Created on: Jun 6, 2025
 *      Author: mzakri
 */
#include "transmission_mesh.h"

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
Packet rxPacket;
Packet txPacket;

float datas[2];

float temp = 0;
float humid = 0;

uint8_t UID[3];
//------Temp and humid----------------------------------------------------------------------------------

void TempANDHumidSensor(){
	while (Si7021_Init() != HAL_OK) {
		printf("Failed to initialize Si7021 sensor!\n\r");
		BSP_LED_On(LED_RED);
	}
		printf("Si7021 sensor initialized successfully!\n\r");
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
void RandomNumbersGeneration(uint8_t j,uint8_t* vectcTxBuff)
{
	CreateLPAWURFrameV2(j,vectcTxBuff);
	HAL_Delay(1000);
	MX_APPE_Process();
}

void CreateLPAWURFrameV2(uint8_t j, uint8_t* vectcTxBuff) {

	/* bit sync */
    for (int i = 0; i < 5; i++)
        vectcTxBuff[i] = 0x00;

    /* Frame sync */
    vectcTxBuff[5] = 0x99;

    if (rxPacket.TransmissionType == DATAREQ){
		TempANDHumidSensor();
		txPacket.Temperature = temp;
		txPacket.Humidity = humid;
    }

    /* Fill Tx buffer with payload */
    vectcTxBuff[6]  = txPacket.TransmissionType;
    vectcTxBuff[7]  = txPacket.ID;
    vectcTxBuff[8]  = txPacket.Destination;
    vectcTxBuff[9]  = (uint8_t)round(txPacket.Temperature);
    vectcTxBuff[10] = (uint8_t)round(txPacket.Humidity);
    vectcTxBuff[11] = txPacket.ADD;
    vectcTxBuff[12] = txPacket.ID_Assign;

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
void PacketHandler(uint8_t LPAWUR_Pay[8], Packet* handle_packet)
{
    handle_packet->TransmissionType = LPAWUR_Pay[0];
    handle_packet->ID = LPAWUR_Pay[1];
    handle_packet->Destination = LPAWUR_Pay[2];
    handle_packet->Temperature = (float)LPAWUR_Pay[3];
    handle_packet->Humidity = (float)LPAWUR_Pay[4];
    handle_packet->ADD = LPAWUR_Pay[5];
    handle_packet->ID_Assign = LPAWUR_Pay[6];
}

void GotoRx(uint8_t* PR,uint8_t* vectcTxBuff)
{

/* Wakeup source configuration */
 HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
 uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

 /* Wakeup on LPAWUR Frame Valid */
 if (wakeupSource & PWR_WAKEUP_LPAWUR)
 {
	BSP_LED_On(LD2);
	HAL_LPAWUR_GetPayload(LPAWUR_Payload);

	//GET packet infos
	PacketHandler(LPAWUR_Payload,&rxPacket);
//	rssi = HAL_MRSubG_GetRssidBm();
//	UpdateRssiStats(rssi,1);

	// Check the transmission type
	switch (rxPacket.TransmissionType)
	{
		case DISCOVERY_REQ:
			printf("Transmission Type: DISCOVERY\n\r");

			printf("LPAWUR data received: [ ");
			printf("Sender's ID : %x ,",rxPacket.ID);
			printf("Target Destination : %x ,",rxPacket.Destination);


			txPacket.TransmissionType = DISCOVERY_RESP;
			txPacket.Destination = MAIN_NODE_ID;
			txPacket.ID = UNASSIGNED_ID;
			txPacket.Temperature = UID[0];
			txPacket.Humidity = UID[1];
			txPacket.ADD = UID[2];
			txPacket.ID_Assign = UID[3];

			break;

		case DATAREQ:
			printf("Transmission Type: DATAREQ\n\r");

			// Update global temp & humid for printing
			temp = rxPacket.Temperature;
			humid = rxPacket.Humidity;
			txPacket.TransmissionType = DATAREP;

			printf("My packet NGL\r\n");
			printf("LPAWUR data received: [ ");
			printf("Sender's ID : %x ,",rxPacket.ID);
			printf("Target Destination : %x ,",rxPacket.Destination);

			break;

		default:
			printf("Unknown transmission type: %d\n\r", rxPacket.TransmissionType);
			break;
	}

	if (rxPacket.ID == checkForID){
		(*PR)++;
	}

	RandomNumbersGeneration(10,vectcTxBuff);
	HAL_LPAWUR_ClearStatus();
	LL_LPAWUR_SetState(ENABLE);
	HAL_Delay(100);
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

void UpdateRssiStats(int16_t rssi, int print_stats)
{
	  rssi_min = (rssi < rssi_min) ? rssi : rssi_min;
	  rssi_max = (rssi > rssi_max) ? rssi : rssi_max;
	  if (print_stats == 1)
	  {printf("Current RSSI: %d dBm | MIN : %d dBm | MAX : %d dBm\r\n", rssi, rssi_min, rssi_max);}
}
//---------------------------------------------------------------------------------------------------
