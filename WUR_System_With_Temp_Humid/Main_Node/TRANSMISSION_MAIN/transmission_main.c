/*
 * txrx.c
 *
 *  Created on: Jun 6, 2025
 *      Author: mzakri
 */
#include "transmission_main.h"

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
Packet txPacket;
Packet rxPacket;

float datas[2];

float temp = 0;
float humid = 0;

uint8_t nextAvailableID = 1;
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

void GETUID(uint8_t *uid) {
    uint32_t uid0 = LL_GetUID_Word0();
    uint32_t uid1 = LL_GetUID_Word1();

    // Example: use last byte of UID2 and 3 bytes of UID0 for uniqueness
    uid[0] = (uid0 >> 24) & 0xFF;
    uid[1] = (uid0 >> 16) & 0xFF;
    uid[2] = (uid1 >> 8) & 0xFF;
    uid[3] = (uid1 >> 0) & 0xFF;

    // Optional: print for debugging
    printf("UID = %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
}

uint8_t isSameNode(NodeEntry *node, Packet *rx) {
    return (node->temp == rx->Temperature &&
            node->hum == rx->Humidity &&
            node->add == rx->ADD &&
            node->id_assign == rx->ID_Assign);
}

//uint8_t GetOrAssignID(Packet *rx) {
//    for (uint8_t i = 0; i < nodeCount; i++) {
//        if (isSameNode(&knownNodes[i], rx)) {
//            return knownNodes[i].assignedID;  // Already assigned
//        }
//    }
//
//    // Not found → assign new ID
//    if (nodeCount < MAX_NODES) {
//        knownNodes[nodeCount].temp = rx->Temperature;
//        knownNodes[nodeCount].hum = rx->Humidity;
//        knownNodes[nodeCount].add = rx->ADD;
//        knownNodes[nodeCount].id_assign = rx->ID_Assign;
//        knownNodes[nodeCount].assignedID = nodeCount + 1;
//        nodeCount++;
//        return knownNodes[nodeCount - 1].assignedID;
//    }
//
//    return 0xFF; // Error: too many nodes
//}


void DiscoveryPhaseHandler(uint8_t ID) {

	printf("Discovery request from unassigned node\n\r");



	txPacket.TransmissionType = ID_ASSIGNMENT;
	txPacket.ID = MAIN_NODE_ID;
	txPacket.Destination = rxPacket.Temperature;  // Broadcast
	txPacket.Temperature = rxPacket.Humidity;  // Broadcast
	txPacket.Humidity = rxPacket.ADD;  // Broadcast
	txPacket.ADD = rxPacket.ID_Assign;  // Broadcast
	txPacket.ID_Assign = ID;  // Using Dunno field to carry new ID

	// Send the assignment
	CreateLPAWURFrameV2(&txPacket, 0, vectcTxBuff);
	HAL_Delay(100);
	MX_APPE_Process();

	printf("Assigned ID %d to new node\n\r", nextAvailableID);
	MX_APPE_Idle();
}
// TX SETUP ----------------------------------------------------------------------------------------------

void RandomNumbersGeneration(Packet* p,uint8_t j,uint8_t* vectcTxBuff)
{
	HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_RTC, PWR_WUP_RISIEDG);

	uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

	//printf("Schedule set \r\n");
	if (wakeupSource & PWR_WAKEUP_RTC){
		CreateLPAWURFrameV2(p,j,vectcTxBuff);
		HAL_Delay(1000);
		MX_APPE_Process();
		printf("Packet sent \r\n");
	}

}

void CreateLPAWURFrameV2(Packet* packet, uint8_t j, uint8_t* vectcTxBuff) {
	ID = packet->ID;
    /* bit sync */
    for (int i = 0; i < 5; i++)
        vectcTxBuff[i] = 0x00;

    /* Frame sync */
    vectcTxBuff[5] = 0x99;

//    if (packet->TransmissionType == DATAREQUEST){
//		TempANDHumidSensor();
//		packet->Temperature = temp;
//		packet->Humidity = humid;
//    }

    /* Fill Tx buffer with payload */
    vectcTxBuff[6]  = packet->TransmissionType;
	vectcTxBuff[7]  = packet->ID;
	vectcTxBuff[8]  = packet->Destination;
    vectcTxBuff[9]  = (uint8_t)round(packet->Temperature);
    vectcTxBuff[10] = (uint8_t)round(packet->Humidity);
    vectcTxBuff[11] = packet->ADD;
    vectcTxBuff[12] = packet->ID_Assign;

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

// RX SETUP --------------------------------------------------------------------------------------------
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

void GotoRx(uint8_t* PR)
{
/* Wakeup source configuration */
 Packet rxPacket;
 printf("Waiting for responses \r\n");
 LL_LPAWUR_SetState(ENABLE); // Ensure LPAWUR is listening

 HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, 1); // Enable wakeup source
 uint32_t wakeupSource2 = HAL_PWREx_GetClearInternalWakeUpLine();
 /* Wakeup on LPAWUR Frame Valid */
 if (wakeupSource2 && PWR_WAKEUP_LPAWUR)
 {
	BSP_LED_On(LD2);
	HAL_LPAWUR_GetPayload(LPAWUR_Payload);

	//GET packet infos
	PacketHandler(LPAWUR_Payload,&rxPacket);


	// Check the transmission type
	switch (rxPacket.TransmissionType)
	{
		case DISCOVERY_RESP:
			printf("Transmission Type: DISCOVERY\n\r");
			DiscoveryPhaseHandler(nextAvailableID);
			break;

		case DATAREP:
			printf("Transmission Type: DATAREQ\n\r");

			// Update global temp & humid for printing
			temp = rxPacket.Temperature;
			humid = rxPacket.Humidity;

			printf("My packet NGL\r\n");
			printf("LPAWUR data received: [ ");
			printf("Sender's ID : %x ,",rxPacket.ID);
			printf("Target Destination : %x ,",rxPacket.Destination);
			printf("temp: %.2f°C, humid: %.2f%% ,", temp, humid);
			printf(" ]\n\r");

			break;

		default:
			printf("Unknown transmission type: %d\n\r", rxPacket.TransmissionType);
			break;
	}

	if (rxPacket.ID == checkForID){
		(*PR)++;
	}

	HAL_LPAWUR_ClearStatus();
	LL_LPAWUR_SetState(ENABLE);
	//printf("Changing to TX \r\n");
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
void UpdateRssiStats(int16_t rssi, int print_stats)
{
	  rssi_min = (rssi < rssi_min) ? rssi : rssi_min;
	  rssi_max = (rssi > rssi_max) ? rssi : rssi_max;
	  if (print_stats == 1)
	  {printf("Current RSSI: %d dBm | MIN : %d dBm | MAX : %d dBm\r\n", rssi, rssi_min, rssi_max);}
}


//---------------------------------------------------------------------------------------------------
