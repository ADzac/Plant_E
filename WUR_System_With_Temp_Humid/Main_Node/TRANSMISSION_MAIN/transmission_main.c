/*
 * txrx.c
 *
 *  Created on: Jun 6, 2025
 *      Author: mzakri
 */
#include "transmission_main.h"

#define PAYLOAD_LEN 7
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

uint8_t vectcTxBuff[15],LPAWUR_Payload[8];

uint8_t ID = 1;
uint8_t checkForID = 5;
uint8_t IDList[255];
uint8_t IDListSize = 0;  // Track how many IDs are in the list
uint8_t alreadyReceived;

int16_t rssi = 0;
int16_t rssi_min = 0;
int16_t rssi_max = -150;

Packet txPacket,rxPacket;

float temp = 0;
float humid = 0;

uint8_t nextAvailableID = 1;
NodeEntry knownNodes[MAX_NODES];
uint8_t nodeCount = 0;
uint8_t uid[4],assignedID;
uint8_t SendDatabase = 0;

uint8_t firstResponseTime;


static DataReport receivedData[MAX_NODES];
static uint8_t receivedDataCount;
//----------------------------- Data Storage  ------------------------------------
void init_data_storage(void) {
    memset(receivedData, 0, MAX_NODES*sizeof(receivedData[0]));
    receivedDataCount = 0;
    alreadyReceived = 0;
    SendDatabase = 0;
    firstResponseTime = 0;
}

//----------------------------- ID Handler ------------------------------------
void GETUID(uint8_t *uid) {
    uint32_t uid0 = LL_GetUID_Word0();
    uint32_t uid1 = LL_GetUID_Word1();
    uid[0] = (uid0 >> 24) & 0xFF;
    uid[1] = (uid0 >> 16) & 0xFF;
    uid[2] = (uid1 >> 8) & 0xFF;
    uid[3] = (uid1 >> 0) & 0xFF;
    printf("UID = %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
}

uint8_t compareUIDs(uint8_t *uid1, uint8_t *uid2) {
    for (int i = 0; i < 4; i++) {
        if (uid1[i] != uid2[i]) return 0;
    }
    return 1;
}

int8_t getAssignedID(uint8_t *uid) {
    for (uint8_t i = 0; i < nodeCount; i++) {
        if (compareUIDs(knownNodes[i].uid, uid)) return knownNodes[i].assignedID;
    }
    return -1;  // Not found
}

uint8_t assignIDToUID(uint8_t *uid) {
    int8_t existingID = getAssignedID(uid);
    if (existingID != -1) return (uint8_t)existingID;  // Already assigned

    if (nodeCount >= MAX_NODES) {
        printf("Maximum node limit reached.\n\r");
        return 0xFF;  // Invalid ID
    }

    uint8_t newID = nextAvailableID;  // Or generate using any other method
    memcpy(knownNodes[nodeCount].uid, uid, 4);
    knownNodes[nodeCount].assignedID = newID;
    nodeCount++;

    printf("Assigned new ID %d to UID [%02X %02X %02X %02X]\n\r",newID, uid[0], uid[1], uid[2], uid[3]);
    nextAvailableID++;
    return newID;
}


// Function to add an ID to the IDList if it doesn't already exist
void AddToIDList(uint8_t id) {
    if (id == 0xFF) return;  // Don't add invalid ID

    // Check if ID already exists
    for (uint8_t i = 0; i < IDListSize; i++) {
        if (IDList[i] == id) {
            return;  // ID already exists
        }
    }
    // Add new ID if there's space
    if (IDListSize < 255) {
        IDList[IDListSize++] = id;
        printf("Added ID %d to IDList\n\r", id);
        printf("ID List Size %d \n\r",IDListSize);
    }
    else printf("IDList full, cannot add ID %d\n\r", id);
}

// Function to check if an ID is in the IDList
uint8_t IsInIDList(uint8_t id) {
    for (uint8_t i = 0; i < IDListSize; i++) {
        if (IDList[i] == id) return 1;
    }
    return 0;

}//----------------------------- Sensor ------------------------------------
void TempANDHumidSensor() {
    while (Si7021_Init() != HAL_OK) {
        BSP_LED_On(LED_RED);
    }

    if (Si7021_ReadTempAndHumidity(&temp, &humid) == HAL_OK) printf("temp: %.2f°C, humid: %.2f \n\r", temp, humid);
    else {
        printf("Error reading Si7021 sensor!\n\r");
        BSP_LED_On(LED_RED);
    }
}
//----------------------------- Packet Handler ------------------------------------
void DiscoveryPhaseHandler(Packet* rxPacketPtr) {
    // 1. Extract UID from the received packet
    uint8_t uid[4];
    uid[0] = rxPacketPtr->ID;
    uid[1] = rxPacketPtr->Destination;
    uid[2] = (uint8_t)rxPacketPtr->Payload[0];
    uid[3] = (uint8_t)rxPacketPtr->Payload[1];

    // 2. Assign ID based on UID
    assignedID = assignIDToUID(uid);
    if (assignedID == 0xFF) {
        printf("ERROR: Too many nodes, cannot assign more IDs\n\r");
        return;
    }

    printf("Discovery request handled, assigned ID = %d\n\r", assignedID);

    // 3. Prepare response packet with assigned ID
    txPacket.TransmissionType = ID_ASSIGNMENT;
    txPacket.ID = uid[0];                   // Who is sending the packet (the main controller)
    txPacket.Destination = uid[1];       // Who is receiving the packet (the original sender)
    txPacket.Payload[0] = uid[2];
    txPacket.Payload[1] = uid[3];
    txPacket.Payload[2] = 0;
    txPacket.Payload[3] = assignedID;

    CreateLPAWURFrameV2();  // Your custom packet creation logic
    HAL_Delay(5);         // Wait 5 ms
    MX_APPE_Process();      // Continue processing (maybe RF stack-related)
}

void SendToDataBase(void){
	if (SendDatabase == 0){
		printf("Sending to DB \n\r");
		for (int i = 1; i < IDListSize+1; i++) {
		        if (receivedData[i].received) {
		            printf("%x,%d,%d \n\r", receivedData[i].id, receivedData[i].temp, receivedData[i].humid);
		        } else {
		            printf("%x,N/A,N/A\n\r", i);  // Mark missing node
		        }
		    }
		SendDatabase = 1;
	}
}
//----------------------------- TX ------------------------------------
void SendPacket() {
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_RTC, PWR_WUP_RISIEDG);
    uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();
    if (wakeupSource & PWR_WAKEUP_RTC) {
        CreateLPAWURFrameV2();
        HAL_Delay(100);
        MX_APPE_Process();
        printf("Packet sent \r\n");
    }
}

void CreateLPAWURFrameV2() {
    for (int i = 0; i < 5; i++) vectcTxBuff[i] = 0x00;
    vectcTxBuff[5] = 0x99;
    vectcTxBuff[6]  = (txPacket.TransmissionType << 4) | (MAIN_NODE_ID & 0x0F);

    vectcTxBuff[7]  = txPacket.ID;
    vectcTxBuff[8]  = txPacket.Destination;
    vectcTxBuff[9]  = (uint8_t)round(txPacket.Payload[0]);
    vectcTxBuff[10] = (uint8_t)round(txPacket.Payload[1]);
    vectcTxBuff[11] = txPacket.Payload[2];
    vectcTxBuff[12] = txPacket.Payload[3];
    EvaluateCrc(&vectcTxBuff[6]);
}

void MX_APPE_Process(void) {
    BSP_LED_On(LD3);
    __HAL_MRSUBG_STROBE_CMD(CMD_TX);
    while ((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS() & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {}
    __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);
    BSP_LED_Off(LD3);
    LL_LPAWUR_SetState(ENABLE);
}

//----------------------------- RX ------------------------------------
void PacketHandler(uint8_t LPAWUR_Pay[8], Packet* handle_packet) {
    handle_packet->TransmissionType = LPAWUR_Pay[0];
    handle_packet->ID = LPAWUR_Pay[1];
    handle_packet->Destination = LPAWUR_Pay[2];
    handle_packet->Payload[0] = (float)LPAWUR_Pay[3];
    handle_packet->Payload[1] = (float)LPAWUR_Pay[4];
    handle_packet->Payload[2] = LPAWUR_Pay[5];
    handle_packet->Payload[3] = LPAWUR_Pay[6];
}

void GotoRx(uint8_t* PR) {
    LL_LPAWUR_SetState(ENABLE);
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, 1);
    uint32_t wakeupSource2 = HAL_PWREx_GetClearInternalWakeUpLine();

    if (wakeupSource2 & PWR_WAKEUP_LPAWUR) {
        BSP_LED_On(LD2);
        HAL_LPAWUR_GetPayload(LPAWUR_Payload);
        PacketHandler(LPAWUR_Payload, &rxPacket);
        int transType = (rxPacket.TransmissionType >> 4) & 0x0F;

        switch (transType) {
            case DISCOVERY_RESP:
                DiscoveryPhaseHandler(&rxPacket);
                break;

            case DATAREP:
            	if (rxPacket.ID == UNASSIGNED_ID){
            		printf("Unknown NODE \n\r");
            		txPacket.ID = MAIN_NODE_ID;
					txPacket.TransmissionType = DISCOVERY_REQ;
					txPacket.Payload[2] = 0;
					txPacket.Payload[3] = 5;
					CreateLPAWURFrameV2();
					HAL_Delay(5);
					MX_APPE_Process();
					printf("Packet sent \r\n");
            	}
            	if (nodeCount != IDListSize){
            		for (int c = 0 ; c < nodeCount ; c ++){
            			if (knownNodes[c].assignedID == rxPacket.ID) AddToIDList(rxPacket.ID);
            		}
            	}

            	//Start counting packet
            	if (receivedDataCount == 0) {
            	        firstResponseTime = HAL_GetTick();  // Set on first packet
            	    }

            	if (IsInIDList(rxPacket.ID)== 1) {
                	    if (receivedData[rxPacket.ID].received == 1) {
                	        //printf("Dropping duplicate DATAREP from ID %d\n\r", rxPacket.ID);
                	    } else {
                	    	receivedData[rxPacket.ID].id = rxPacket.ID;
                	        receivedData[rxPacket.ID].temp = rxPacket.Payload[0];
                	        receivedData[rxPacket.ID].humid = rxPacket.Payload[1];
                	        receivedData[rxPacket.ID].received = 1;
                	        receivedDataCount++;
                	        printf("DATAREP from %x: Temp = %d°C, Humid = %d%%\n\r",
                	               rxPacket.ID, rxPacket.Payload[0], rxPacket.Payload[1]);
                	    }
                }
                if (receivedDataCount == IDListSize) SendToDataBase();
                break;

            case DATAREQ:
                break;
            case DISCOVERY_REQ:
                break;
            case ID_ASSIGNMENT:
                break;
            case ID_RECEIVED:
            	if (IsInIDList(txPacket.ID) == 1) printf("Id already assigned : %d \n\r", txPacket.ID);
            	else  AddToIDList(rxPacket.ID);
				break;
            case ALERT:
                printf("NI NO NI NO \r\n");
                break;
            default:
                printf("Unknown transmission type: %d\n\r", rxPacket.TransmissionType);
                break;
        }

        HAL_LPAWUR_ClearStatus();
        LL_LPAWUR_SetState(ENABLE);
        HAL_Delay(5);
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
	if (print_stats == 1) printf("Current RSSI: %d dBm | MIN : %d dBm | MAX : %d dBm\r\n", rssi, rssi_min, rssi_max);
}
//--------------------------------------------------------------------------------------------------
