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

uint8_t myID = UNASSIGNED_ID;
uint16_t rssi_min = -150;
uint16_t rssi_max = 0;
//----------------------------- Sensor ------------------------------------
void TempANDHumidSensor() {
    while (Si7021_Init() != HAL_OK) {
        printf("Failed to initialize Si7021 sensor!\n\r");
        BSP_LED_On(LED_RED);
    }
    BSP_LED_On(LED_GREEN);
    HAL_Delay(100);
    BSP_LED_Off(LED_GREEN);

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
    uid[0] = (uid0 >> 24) & 0xFF;
    uid[1] = (uid0 >> 16) & 0xFF;
    uid[2] = (uid1 >> 8) & 0xFF;
    uid[3] = (uid1 >> 0) & 0xFF;
    printf("UID = %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
}

uint8_t shouldForward(Packet *pkt) {
    // ❌ Do not forward packets you originated
    if (pkt->ID == myID) {
        return 0;
    }

    for (int i = 0; i < MAX_CACHE; i++) {
        if (packetCache[i].sender == pkt->ID &&
            packetCache[i].transType == pkt->TransmissionType &&
            packetCache[i].temp == (uint8_t)pkt->Payload[0] &&
            packetCache[i].humid == (uint8_t)pkt->Payload[1]) {
            if (pkt->Payload[2] < packetCache[i].hop) {
                packetCache[i].hop = pkt->Payload[2];  // update to better path
                return 1;
            }
            return 0; // duplicate or worse
        }
    }

    // Add new packet to cache
    for (int i = 0; i < MAX_CACHE; i++) {
        if (packetCache[i].transType == 0xFF || packetCache[i].hop == 0) {
            packetCache[i].sender = pkt->ID;
            packetCache[i].transType = pkt->TransmissionType;
            packetCache[i].temp = (uint8_t)pkt->Payload[0];
            packetCache[i].humid = (uint8_t)pkt->Payload[1];
            packetCache[i].hop = pkt->Payload[2];
            return 1;
        }
    }

    return 0; // cache full or not eligible
}


//----------------------------- TX ------------------------------------
void RandomNumbersGeneration(uint8_t j, uint8_t* vectcTxBuff) {
    CreateLPAWURFrameV2(j, vectcTxBuff);
    HAL_Delay(HAL_GetTick() % 3000);
    MX_APPE_Process();
}

void CreateLPAWURFrameV2(uint8_t j, uint8_t* vectcTxBuff) {
    for (int i = 0; i < 5; i++) vectcTxBuff[i] = 0x00;
    vectcTxBuff[5] = 0x99;

    vectcTxBuff[6]  = txPacket.TransmissionType;
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
    LL_LPAWUR_SetState(ENABLE);
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

        switch (rxPacket.TransmissionType) {
        case DISCOVERY_REQ:
            printf("Discovery request received\n\r");

            // 1. If this node has no ID, respond to main
            if (myID == UNASSIGNED_ID) {
                txPacket.TransmissionType = DISCOVERY_RESP;
                txPacket.ID = UID[0]; // sender
                txPacket.Destination = MAIN_NODE_ID;
                txPacket.Payload[0] = UID[1];
                txPacket.Payload[1] = UID[2];
                txPacket.Payload[2] = UID[3];
                txPacket.Payload[3] = 5; // TTL
                RandomNumbersGeneration(10, vectcTxBuff);
            }

            // 2. Rebroadcast the DISCOVERY_REQ to help reach other nodes
            if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                txPacket = rxPacket;
                txPacket.Payload[2] += 1; // HopCount++
                txPacket.Payload[3] -= 1; // TTL--
                RandomNumbersGeneration(10, vectcTxBuff);
            }

            break;


            case DISCOVERY_RESP:
                if (rxPacket.Destination == MAIN_NODE_ID && rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                    txPacket = rxPacket;
                    txPacket.Payload[2] += 1;
                    txPacket.Payload[3] -= 1;
                    RandomNumbersGeneration(10, vectcTxBuff);
                }
                break;

            case DATAREQ:
                printf("DATAREQ received from %d\n\r", rxPacket.ID);

                // Respond with sensor data
                TempANDHumidSensor();
                txPacket.TransmissionType = DATAREP;
                txPacket.ID = myID;
                txPacket.Destination = MAIN_NODE_ID;
                txPacket.Payload[0] = (uint8_t)roundf(temp);
                txPacket.Payload[1] = (uint8_t)roundf(humid);
                txPacket.Payload[2] = 0;
                txPacket.Payload[3] = 5; // TTL
                RandomNumbersGeneration(10, vectcTxBuff);

                // Rebroadcast DATAREQ for others
                if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                    txPacket = rxPacket;
                    txPacket.Payload[2] += 1;
                    txPacket.Payload[3] -= 1;
                    RandomNumbersGeneration(10, vectcTxBuff);
                }

                break;


            case DATAREP:
                if (rxPacket.Destination == MAIN_NODE_ID &&
                    rxPacket.Payload[3] > 0 &&
                    rxPacket.ID != myID &&      // ✅ don't forward your own reply
                    shouldForward(&rxPacket)) {

                    txPacket = rxPacket;
                    txPacket.Payload[2] += 1;
                    txPacket.Payload[3] -= 1;
                    RandomNumbersGeneration(10, vectcTxBuff);
                }
                break;


            case ID_ASSIGNMENT:
                if (rxPacket.Destination == UID[0] &&
                    rxPacket.Payload[0] == UID[1] &&
                    rxPacket.Payload[1] == UID[2] &&
                    rxPacket.Payload[2] == UID[3]) {
                    myID = rxPacket.Payload[3];
                    printf("ID assigned: %d\n\r", myID);
                } else if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                    txPacket = rxPacket;
                    txPacket.Payload[2] += 1;
                    txPacket.Payload[3] -= 1;
                    RandomNumbersGeneration(10, vectcTxBuff);
                }
                break;

            default:
                printf("Unknown transmission type: %d\n\r", rxPacket.TransmissionType);
                break;
        }

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
