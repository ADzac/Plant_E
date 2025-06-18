/*
 * txrx.c
 *
 *  Created on: Jun 6, 2025
 *      Author: mzakri
 */

#include "transmission_main.h"

#define PAYLOAD_LEN 7
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

uint8_t vectcTxBuff[15];
uint8_t LPAWUR_Payload[8];

uint8_t ID = 1;
uint8_t checkForID = 5;

int16_t rssi = 0;
int16_t rssi_min = 0;
int16_t rssi_max = -150;

Packet txPacket;
Packet rxPacket;

float temp = 0;
float humid = 0;

uint8_t nextAvailableID = 1;
NodeEntry knownNodes[MAX_NODES];
uint8_t nodeCount = 0;

void TempANDHumidSensor() {
    while (Si7021_Init() != HAL_OK) {
        BSP_LED_On(LED_RED);
    }
    BSP_LED_On(LED_GREEN);
    HAL_Delay(100);
    BSP_LED_Off(LED_GREEN);

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

void GETUID(uint8_t *uid) {
    uint32_t uid0 = LL_GetUID_Word0();
    uint32_t uid1 = LL_GetUID_Word1();
    uid[0] = (uid0 >> 24) & 0xFF;
    uid[1] = (uid0 >> 16) & 0xFF;
    uid[2] = (uid1 >> 8) & 0xFF;
    uid[3] = (uid1 >> 0) & 0xFF;
    printf("UID = %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
}

uint8_t isSameNode(NodeEntry *node, Packet *rx) {
    return (node->temp == rx->ID &&
            node->hum == rx->Payload[0] &&
            node->hopcount == rx->Payload[1] &&
            node->id_assign == rx->Payload[2]);
}

uint8_t GetOrAssignID(Packet *rx) {
    for (uint8_t i = 0; i < nodeCount; i++) {
        if (isSameNode(&knownNodes[i], rx)) {
            return knownNodes[i].assignedID;
        }
    }
    if (nodeCount < MAX_NODES) {
        knownNodes[nodeCount].temp = rx->ID;
        knownNodes[nodeCount].hum = rx->Payload[0];
        knownNodes[nodeCount].hopcount = rx->Payload[1];
        knownNodes[nodeCount].id_assign = rx->Payload[2];
        knownNodes[nodeCount].assignedID = nextAvailableID++;
        nodeCount++;
        return knownNodes[nodeCount - 1].assignedID;
    }
    return 0xFF;
}

void DiscoveryPhaseHandler(Packet* rxPacketPtr) {
    uint8_t assignedID = GetOrAssignID(rxPacketPtr);
    if (assignedID == 0xFF) {
        printf("ERROR: Too many nodes, cannot assign more IDs\n\r");
        return;
    }
    printf("Discovery request handled, assigned ID = %d\n\r", assignedID);

    txPacket.TransmissionType = ID_ASSIGNMENT;
    txPacket.ID = MAIN_NODE_ID;
    txPacket.Destination = rxPacketPtr->ID;
    txPacket.Payload[0] = rxPacketPtr->Payload[0];
    txPacket.Payload[1] = rxPacketPtr->Payload[1];
    txPacket.Payload[2] = rxPacketPtr->Payload[2];
    txPacket.Payload[3] = assignedID;

    CreateLPAWURFrameV2();
    HAL_Delay(500);
    HAL_Delay(HAL_GetTick() % 1000);
    MX_APPE_Process();
}

void SendPacket() {
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_RTC, PWR_WUP_RISIEDG);
    uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();
    if (wakeupSource & PWR_WAKEUP_RTC) {
        CreateLPAWURFrameV2();
        HAL_Delay(1000);
        MX_APPE_Process();
        printf("Packet sent \r\n");
    }
}

void CreateLPAWURFrameV2() {
    for (int i = 0; i < 5; i++) vectcTxBuff[i] = 0x00;
    vectcTxBuff[5] = 0x99;
    vectcTxBuff[6]  = txPacket.TransmissionType;
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
    printf("Waiting for responses \r\n");
    LL_LPAWUR_SetState(ENABLE);
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, 1);
    uint32_t wakeupSource2 = HAL_PWREx_GetClearInternalWakeUpLine();

    if (wakeupSource2 && PWR_WAKEUP_LPAWUR) {
        BSP_LED_On(LD2);
        HAL_LPAWUR_GetPayload(LPAWUR_Payload);
        PacketHandler(LPAWUR_Payload, &rxPacket);

        switch (rxPacket.TransmissionType) {

            case DISCOVERY_RESP:
                printf("Received DISCOVERY_RESP from UID [%02X %02X %02X %02X]\n\r",
                    rxPacket.ID,
                    (uint8_t)rxPacket.Payload[0],
                    (uint8_t)rxPacket.Payload[1],
                    rxPacket.Payload[2]);
                DiscoveryPhaseHandler(&rxPacket);
                break;

            case DATAREP:
                temp = rxPacket.Payload[0];
                humid = rxPacket.Payload[1];
                printf("DATAREP from %x: Temp = %.2f°C, Humid = %.2f%%\n\r",
                    rxPacket.ID, temp, humid);
                break;

            default:
                printf("Unknown transmission type: %d\n\r", rxPacket.TransmissionType);
                break;
        }

        if (rxPacket.ID == checkForID) {
            (*PR)++;
        }

        HAL_LPAWUR_ClearStatus();
        LL_LPAWUR_SetState(ENABLE);
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
