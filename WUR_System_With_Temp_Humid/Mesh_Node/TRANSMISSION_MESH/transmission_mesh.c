/*
 * transmission_mesh.c
 *
 * Reworked for Mesh Network
 *  Created on: Jun 6, 2025
 *      Author: mzakri
 */
#include "transmission_mesh.h"

#define PAYLOAD_LEN 7
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

uint8_t vectcTxBuff[15],LPAWUR_Payload[8];

Packet txPacket,rxPacket;

SimpleCache myCache ;// Initialize as empty

float temp = 0;
float humid = 0;

uint8_t UID[4];
uint8_t myHop = 10;
uint8_t myID = UNASSIGNED_ID;

uint8_t datareqSent,hopper ;

RoutingEntry routingTable[MAX_ROUTES];
//----------------------------- Sensor ------------------------------------
void TempANDHumidSensor() {
    while (Si7021_Init() != HAL_OK) BSP_LED_On(LED_RED);

    if (Si7021_ReadTempAndHumidity(&temp, &humid) == HAL_OK) {
        printf("TEMP: %.2f°C, HUMID: %.2f \n\r", temp, humid);
        temp = round(temp);
        humid = round(humid);
    }
    else BSP_LED_On(LED_RED);
}
//----------------------------- UID ------------------------------------
void GETUID(uint8_t *uid) {
    uint32_t uid0 = LL_GetUID_Word0();
    uint32_t uid1 = LL_GetUID_Word1();
    uid[0] = (uid0 >> 24) & 0xFF;
    uid[1] = (uid0 >> 16) & 0xFF;
    uid[2] = (uid1 >> 8) & 0xFF;
    uid[3] = (uid1 >> 0) & 0xFF;
    printf("UID = %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
}
//----------------------------- PacketHandling ------------------------------------
uint8_t isSelfPacket(Packet *pkt) {
    return (pkt->ID == myID || (pkt->ID == UID[0] && (uint8_t)pkt->Payload[0] == UID[1] && (uint8_t)pkt->Payload[1] == UID[2]
			&& pkt->Payload[2] == UID[3]) );
}

void InitRoutingTable(void) {
    for (int i = 0; i < MAX_ROUTES; i++) {
        routingTable[i].nodeID = UNASSIGNED_ID;
        routingTable[i].nextHop = 0;
        routingTable[i].hopCount = 255; // max hop means undefined
        routingTable[i].lastSeen = 0;
    }
}

void UpdateRoutingTable(uint8_t nodeID, uint8_t nextHop, uint8_t hopCount) {
    // Accept only direct neighbors (hop count 0 or 1)
    if (hopCount > 1 || nodeID == myID)  {
        return; // Ignore non-direct nodes
    }

    // Check if the node is already in the table
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routingTable[i].nodeID == nodeID) {
            // Update if hop count is better or same next hop
            if (hopCount <= routingTable[i].hopCount || routingTable[i].nextHop == nextHop) {
                routingTable[i].nextHop = nextHop;
                routingTable[i].hopCount = hopCount;
            }
            return;
        }
    }

    // Insert into empty slot
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routingTable[i].nodeID == UNASSIGNED_ID) {
            routingTable[i].nodeID = nodeID;
            routingTable[i].nextHop = nextHop;
            routingTable[i].hopCount = hopCount;
            return;
        }
    }

    // Table full, you could log or evict oldest entry
    printf("Routing table full. Could not add node %d\n", nodeID);
}

void printRoutingTable(void) {
    printf("=== Routing Table ===\r\n");

    for (int i = 0; i < MAX_ROUTES; i++) {
		if (routingTable[i].nodeID != UNASSIGNED_ID) printf("Slot %d: NodeID: %u | NextHop: %u | Hops: %u\r\n",
			   i,
			   routingTable[i].nodeID,
			   routingTable[i].nextHop,
			   routingTable[i].hopCount);
    }

    printf("======================\r\n");
}


uint8_t shouldForward(Packet *pkt) {
    // Drop packet if it's from self
    if (isSelfPacket(pkt)) {
        return 0;
    }
    // Validate TTL before modifying it
    uint8_t ttl = pkt->Payload[3];
    if (ttl < 1) {
        return 0;
    }
    return 1;
}

void forwardPacket(uint8_t type) {
	memcpy(&txPacket, &rxPacket, sizeof(Packet)); // current
    // Forward packet: increase hop count, decrease TTL
    printf("REBROADCAST...\r\n");
    txPacket.TransmissionType = type;
    if (type != ID_ASSIGNMENT && type != DISCOVERY_RESP) txPacket.Destination = myID;  // rxPacket.ID is the original sender ,the current hopper
    txPacket.Payload[2] += 1;  // Increment hop count

   if (type != ID_ASSIGNMENT) txPacket.Payload[3] -= 1;  // Decrement TTL
   SendPacket(vectcTxBuff);
}

uint8_t isDuplicate(Packet *pkt) {
    // Expire cache after 5 minutes
    if ((HAL_GetTick() - myCache.lastRxTime) > (5 * 60 * 1000)) myCache.lastSenderID = 0; // Force accept next packet

    if (hopper == MAIN_NODE_ID) return 0;

    // Check if same sender + type
    if (myCache.lastSenderID == pkt->ID && myCache.lastTransType == pkt->TransmissionType) return 1;

    // Update cache
    myCache.lastSenderID = pkt->ID;
    myCache.lastTransType = pkt->TransmissionType;
    myCache.lastRxTime = HAL_GetTick();
    return 0;
}

// Call this when:
// 1. Starting new discovery and data req phase
// 2. After long periods of inactivity
// 3. On network reset commands
void ResetCache(void) {
    myCache.lastSenderID = 0;
    myCache.lastTransType = 0xFF; // Invalid value
    myCache.lastRxTime = 0;
}

void PrepareDiscoveryResponse(Packet *Pack) {
    Pack->TransmissionType = DISCOVERY_RESP;
    Pack->ID = UID[0];
    Pack->Destination = UID[1];
    Pack->Payload[0] = UID[2];
    Pack->Payload[1] = UID[3];
    Pack->Payload[2] = rxPacket.Payload[2];
    Pack->Payload[3] = 5;
    SendPacket(vectcTxBuff);
}

void SendAckALIVE(Packet *Pack) {
    Pack->TransmissionType = ID_RECEIVED;
    Pack->ID = myID;
    Pack->Destination = MAIN_NODE_ID;
    Pack->Payload[0] = 0;
    Pack->Payload[1] = 0;
    Pack->Payload[2] = 0;
    Pack->Payload[3] = 5;
    SendPacket(vectcTxBuff);
}
//----------------------------- TX ------------------------------------
/*
 * Randomize the delay before sending the Packet
 */
void SimpleRand16(void)
{
	uint16_t val = LL_RNG_ReadRandData16(RNG);
	switch (myHop) {
		case 0:
			val /= 32;
			if(val < 200) val +=200;
			break;
		case 1:
			val /= 16;
			if(val < 2000) val +=2000;
			break;
		case 2:
			val /= 8;
			if(val < 4000) val +=4000;
			break;
		case 3:
			val /= 6;
			if(val < 8000) val +=8000;
			break;
		case 4:
			val /= 4;
			if(val < 1000) val +=10000;
			break;
		default:
			val /= 2;
			if(val < 16000) val +=16000;
			if (val > 20000) val = 20000 + LL_RNG_ReadRandData16(RNG)/32;
			break;
	}
	printf("Random Delay: %d\r\n", val);
	HAL_Delay(val);
}

void SendPacket(uint8_t* vectcTxBuff) {
    CreateLPAWURFrameV2(vectcTxBuff);
    SimpleRand16();
    MX_APPE_Process();
}

void CreateLPAWURFrameV2(uint8_t* vectcTxBuff) {
    for (int i = 0; i < 5; i++) vectcTxBuff[i] = 0x00;
    vectcTxBuff[5] = 0x99;

    vectcTxBuff[6]  = (txPacket.TransmissionType << 4) | (myID & 0x0F);
    if (txPacket.TransmissionType == DISCOVERY_RESP || txPacket.TransmissionType == ID_ASSIGNMENT ||
        txPacket.TransmissionType == DATAREP) vectcTxBuff[7] = txPacket.ID;
    else vectcTxBuff[7]  = (myID != UNASSIGNED_ID) ? myID : txPacket.ID;

    vectcTxBuff[8]  = txPacket.Destination;
    vectcTxBuff[9]  = txPacket.Payload[0];
    vectcTxBuff[10] = txPacket.Payload[1];
    vectcTxBuff[11] = txPacket.Payload[2];
    vectcTxBuff[12] = txPacket.Payload[3];

    EvaluateCrc(&vectcTxBuff[6]);
}

void MX_APPE_Process(void) {
    //BSP_LED_On(LD3);
    __HAL_MRSUBG_STROBE_CMD(CMD_TX);
    while((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS() & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {}
    __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);
    //BSP_LED_Off(LD3);
}
//----------------------------- RX ------------------------------------
void PacketHandler(uint8_t LPAWUR_Pay[8], Packet* pkt) {
    pkt->TransmissionType = LPAWUR_Pay[0];
    pkt->ID = LPAWUR_Pay[1];
    pkt->Destination = LPAWUR_Pay[2];
    pkt->Payload[0] = (float)LPAWUR_Pay[3];
    pkt->Payload[1] = (float)LPAWUR_Pay[4];
    pkt->Payload[2] = LPAWUR_Pay[5];
    pkt->Payload[3] = LPAWUR_Pay[6];
}

void GotoRx(uint8_t* vectcTxBuff) {
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
    uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

    if (wakeupSource & PWR_WAKEUP_LPAWUR) {
        //BSP_LED_On(LD2);
        HAL_LPAWUR_GetPayload(LPAWUR_Payload);
        PacketHandler(LPAWUR_Payload, &rxPacket);
        uint8_t transType = (rxPacket.TransmissionType >> 4) & 0x0F;
        if (transType != ID_ASSIGNMENT) hopper = rxPacket.ID;
        if (hopper != 0) printf("Received from a hopper ID %d \r\n",hopper);

        if (transType != ID_ASSIGNMENT  && transType != ALERT  && transType != DISCOVERY_RESP){
			UpdateRoutingTable(hopper, rxPacket.Destination, rxPacket.Payload[2]);
			printRoutingTable();
        }

		if (myHop > 6 && rxPacket.ID == MAIN_NODE_ID) myHop = rxPacket.Payload[2];
        if (isDuplicate(&rxPacket) == 0){

			switch (transType) {
				case DISCOVERY_REQ:
					printf("DISCOVERY_REQ received\n\r");
					ResetCache();

					if (rxPacket.Payload[3] > 2 && shouldForward(&rxPacket)  && datareqSent == 0) {
						myHop = (uint8_t)rxPacket.Payload[2];
						datareqSent = 1;
						printf("Im this amount of hop to main : %d \n\r",myHop);
						forwardPacket(DISCOVERY_REQ);
					}

					if (myID == UNASSIGNED_ID) PrepareDiscoveryResponse(&txPacket);
					else SendAckALIVE(&txPacket);
				break;

				case DISCOVERY_RESP:
					if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) forwardPacket(DISCOVERY_RESP);
				break;

				case DATAREQ:
					ResetCache();
					if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
						forwardPacket(DATAREQ);
					}
					if (myID == UNASSIGNED_ID){
						PrepareDiscoveryResponse(&txPacket);
						break;
					}
					HAL_Delay(10);
					TempANDHumidSensor();
					txPacket.TransmissionType = DATAREP;
					txPacket.ID = myID;
					txPacket.Destination = myID; //for hop routing cases
					txPacket.Payload[0] = temp;
					txPacket.Payload[1] = humid;
					txPacket.Payload[2] = 0;
					txPacket.Payload[3] = 5;
					SendPacket(vectcTxBuff);
				break;

				case DATAREP:
					if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
						printf("DataREP received \r\n");
						TempANDHumidSensor();

						if (abs(temp -rxPacket.Payload[0])>10) forwardPacket(ALERT);
						else forwardPacket(DATAREP);
					}
				break;

				case ID_ASSIGNMENT:
					printf("ID_ASS received \r\n");
					if (rxPacket.ID == UID[0] && rxPacket.Destination == UID[1] && rxPacket.Payload[0] == UID[2] &&
					rxPacket.Payload[1] == UID[3]) {
						if (myID == UNASSIGNED_ID){
							myID = rxPacket.Payload[3];
							printf("ID assigned: %d\n\r", myID);
							SendAckALIVE(&txPacket);
						}
					}
					else if (shouldForward(&rxPacket)) forwardPacket(ID_ASSIGNMENT);
				break;

				case ALERT :
					memcpy(&txPacket, &rxPacket, sizeof(Packet));
					SendPacket(vectcTxBuff);
				break;

				default:
				break;
			}
        }
        HAL_LPAWUR_ClearStatus();
        LL_LPAWUR_SetState(ENABLE);
        //BSP_LED_Off(LD2);
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
