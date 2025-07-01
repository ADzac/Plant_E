
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

uint8_t vectcTxBuff[15];
uint8_t LPAWUR_Payload[8];

Packet txPacket;
Packet rxPacket;
PacketSignature packetCache[MAX_CACHE] = {0};

float temp = 0;
float humid = 0;

uint8_t UID[4];
uint8_t myHop = 0;
uint8_t myID = UNASSIGNED_ID;
uint16_t rssi_min = -150;
uint16_t rssi_max = 0;

uint8_t datareqSent = 0;
//----------------------------- Sensor ------------------------------------
void TempANDHumidSensor() {
    while (Si7021_Init() != HAL_OK) {
        printf("Failed to initialize Si7021 sensor!\n\r");
        BSP_LED_On(LED_RED);
    }

    if (Si7021_ReadTempAndHumidity(&temp, &humid) == HAL_OK) {
        printf("temp: %.2f°C, humid: %.2f \n\r", temp, humid);
    } else {
        printf("Error reading Si7021 sensor!\n\r");
        BSP_LED_On(LED_RED);
    }
}

//----------------------------- UID ------------------------------------
void GETUID(uint8_t *uid) {
    uint32_t uid0 = LL_GetUID_Word0();
    uint32_t uid1 = LL_GetUID_Word1();
    //printf("FIRST 32 BITS %X \r\n",uid0);
    uid[0] = (uid0 >> 24) & 0xFF;
    uid[1] = (uid0 >> 16) & 0xFF;
    uid[2] = (uid1 >> 8) & 0xFF;
    uid[3] = (uid1 >> 0) & 0xFF;
    printf("UID = %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
}

uint8_t isSelfPacket(Packet *pkt) {
    return (pkt->ID == myID || (pkt->ID == UID[0] &&
            (uint8_t)pkt->Payload[0] == UID[1] &&
            (uint8_t)pkt->Payload[1] == UID[2] &&
            pkt->Payload[2] == UID[3]));
}

uint8_t shouldForward(Packet *pkt) {
    // Don't forward self-generated packets
    if (isSelfPacket(pkt)) {
        printf("Mine \r\n");
        return 0;
    }

    // Check TTL first (if TTL is 0 or invalid, drop immediately)
    if (pkt->Payload[3] <= 1) {  // TTL is about to expire (after decrement)
        printf("TTL expired \r\n");
        return 0;
    }

    // Update packet metadata (hop count + TTL)
    pkt->Payload[2]++;  // Increment hop count
    pkt->Payload[3]--;  // Decrement TTL

    // Forward the packet
    return 1;
}


//----------------------------- TX ------------------------------------
/*
 * Randomize the delay before sending the Packet
 */
uint16_t SimpleRand16(void)
{
    uint16_t* val = malloc(sizeof(uint16_t));
    if (val == NULL) {
        printf("Memory allocation failed\n");
        return 1;
    }

    *val = LL_RNG_ReadRandData16(RNG);  // replace NULL with RNG if available

//    printf("Original value: %u \r\n", *val);  // use %u for uint16_t
    switch(myHop){ //Hopcount
		case (0):
			*val = *val/15;
			break;
		case (1):
			*val = *val/10; // /8
			break;
		case (2):
			*val = *val/8;
			break;
		case (3):
			*val = *val/6; // /4
			break;
		case(4):
			*val = *val/4; // /2
			break;
		default :
			break;
			*val = *val/2;
    }
    // Perform right-shift on value, not pointer


    printf("Shifted value: %u \r\n", *val);
    HAL_Delay(*val);
    free(val);
    return 0;
}

void RandomNumbersGeneration(uint8_t j, uint8_t* vectcTxBuff) {
    CreateLPAWURFrameV2(j, vectcTxBuff);
    SimpleRand16();
    MX_APPE_Process();
}

void CreateLPAWURFrameV2(uint8_t j, uint8_t* vectcTxBuff) {
    for (int i = 0; i < 5; i++) vectcTxBuff[i] = 0x00;
    vectcTxBuff[5] = 0x99;

    vectcTxBuff[6]  = (txPacket.TransmissionType << 4) | (myID & 0x0F);
    vectcTxBuff[7]  = txPacket.ID;
    vectcTxBuff[8]  = txPacket.Destination;
    vectcTxBuff[9]  = txPacket.Payload[0];
    vectcTxBuff[10] = txPacket.Payload[1];
    vectcTxBuff[11] = txPacket.Payload[2];
    vectcTxBuff[12] = txPacket.Payload[3];

    EvaluateCrc(&vectcTxBuff[6]);
}

void MX_APPE_Process(void) {
    BSP_LED_On(LD3);
    __HAL_MRSUBG_STROBE_CMD(CMD_TX);
    while((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS() & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {}
    __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);
    BSP_LED_Off(LD3);

}
uint8_t PackMinimal() {
	temp = (uint8_t)round((temp));  // Mask to 4 bits

	// Scale humidity to 4 bits (0-100% in 6.67% steps)
	humid = (uint8_t)round((humid));  // Mask to 4 bits

	// Combine: Temp in upper 4 bits, Humidity in lower 4 bits
	return 0;// (t << 4) | h;
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

void GotoRx(uint8_t* PR, uint8_t* vectcTxBuff) {
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
    uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

    if (wakeupSource & PWR_WAKEUP_LPAWUR) {
        BSP_LED_On(LD2);
        HAL_LPAWUR_GetPayload(LPAWUR_Payload);
        PacketHandler(LPAWUR_Payload, &rxPacket);
        uint8_t transType = (rxPacket.TransmissionType >> 4) & 0x0F;
        uint8_t hopper = rxPacket.TransmissionType & 0x0F;
        if (hopper != 0) printf("Received from a hopper ID %d \r\n",hopper);


        switch (transType) {
        case DISCOVERY_REQ:
			printf("DISCOVERY_REQ received\n\r");

			// Rebroadcast discovery
			if (rxPacket.Payload[3] > 2 && shouldForward(&rxPacket)  && datareqSent == 0) {
				txPacket = rxPacket;
				myHop = (uint8_t)txPacket.Payload[2];
				printf("Rebroadcasting  \r\n");
				txPacket.TransmissionType = DISCOVERY_REQ;
				txPacket.Payload[2] += 1;
				txPacket.Payload[3] -= 1;
				datareqSent = 1;
				RandomNumbersGeneration(10, vectcTxBuff);
			}

			// Respond to main if unassigned
			if (myID == UNASSIGNED_ID) {
				GETUID(UID);
				txPacket.TransmissionType = DISCOVERY_RESP;
				txPacket.ID = UID[0];
				txPacket.Destination = UID[1];
				txPacket.Payload[0] = UID[2];
				txPacket.Payload[1] = UID[3];
				txPacket.Payload[2] = rxPacket.Payload[2];
				txPacket.Payload[3] = 5;
				printf("Asking for ID \r\n");
				HAL_Delay(100);
				RandomNumbersGeneration(10, vectcTxBuff);
			}
			break;

            case DISCOVERY_RESP:
                if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                    txPacket = rxPacket;
                    printf("Rebroadcasting  \r\n");
                    txPacket.TransmissionType = DISCOVERY_RESP;
                    txPacket.Payload[2] += 1;
                    txPacket.Payload[3] -= 1;
                    RandomNumbersGeneration(10, vectcTxBuff);
                }
                break;

            case DATAREQ:

				if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket) && datareqSent == 0) {
					txPacket = rxPacket;
					printf("Rebroadcasting  \r\n");
					txPacket.TransmissionType = DATAREQ;
					txPacket.Payload[2] += 1;
					txPacket.Payload[3] -= 1;
					datareqSent = 1;
					RandomNumbersGeneration(10, vectcTxBuff);
				}
				HAL_Delay(100);
				TempANDHumidSensor();
				PackMinimal();
				txPacket.TransmissionType = DATAREP;
				txPacket.ID = myID;
				txPacket.Destination = MAIN_NODE_ID;
				txPacket.Payload[0] = temp;
				txPacket.Payload[1] = humid;
				txPacket.Payload[2] = 0;
				txPacket.Payload[3] = 5;
				RandomNumbersGeneration(10, vectcTxBuff);

				break;

            case DATAREP:
                if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                	printf("DataREP received \r\n");
                    txPacket = rxPacket;
                    printf("Rebroadcasting  \r\n");
                    TempANDHumidSensor();
                    if (abs(temp -txPacket.Payload[0])>10){
                    	txPacket.TransmissionType = ALERT;
                    	//txPacket.Payload[1] = "High Temp \a\n";
                    }
                    else{
                    	txPacket.TransmissionType = DATAREP;
                    }
                    txPacket.Payload[2] += 1;
                    txPacket.Payload[3] -= 1;
                    RandomNumbersGeneration(10, vectcTxBuff);
                }
                break;


            case ID_ASSIGNMENT:
                if (rxPacket.ID == UID[0] && rxPacket.Destination == UID[1] &&
                    rxPacket.Payload[0] == UID[2] &&
                    rxPacket.Payload[1] == UID[3]
                    ) {
                	if (myID == UNASSIGNED_ID){
                		myID = rxPacket.Payload[3];
                		printf("ID assigned: %d\n\r", myID);
                	}

                } else if (shouldForward(&rxPacket)) {
                    txPacket = rxPacket;
                    printf("Rebroadcasting  \r\n");
                    txPacket.Payload[2] += 1;
                    RandomNumbersGeneration(10, vectcTxBuff);
                }
                break;


            case ALERT :
            	txPacket = rxPacket;
                RandomNumbersGeneration(10, vectcTxBuff);

            	break;

            default:
                printf("Unknown transmission type: %d\n\r", rxPacket.TransmissionType);
                break;
        }

        HAL_LPAWUR_ClearStatus();
        LL_LPAWUR_SetState(ENABLE);
        printf("Finish \r\n");
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
