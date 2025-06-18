#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include "stm32wl3x_hal.h"
#include "stm32wl3x_ll_bus.h"
#include "stm32wl3x_ll_cortex.h"
#include "stm32wl3x_ll_rcc.h"
#include "stm32wl3x_ll_system.h"
#include "stm32wl3x_ll_utils.h"
#include "stm32wl3x_ll_gpio.h"
#include "stm32wl3x_ll_pwr.h"
#include "stm32wl3x_ll_dma.h"
#include "stm32wl3x_ll_usart.h"
#include "stm32wl3x_ll_lpawur.h"
#include "stm32wl3x_nucleo.h"
#include<stdio.h>

#include "math.h"
#include "app_conf.h"
#include "crc_4wkup_rf.h"
#include "stm32_lpm.h"
#include "temphumid.h"

#define DISCOVERY_REQ 10
#define DISCOVERY_RESP 11
#define ID_ASSIGNMENT 12
#define DATAREQ 13
#define DATAREP 14

#define ALERT 15

#define MAIN_NODE_ID 0
#define UNASSIGNED_ID 0xFF


#if defined(__ARMCC_VERSION)
#define UID_WORD0 "0x%X"
#define UID_WORD1 "0x%X"
#else
#define UID_WORD0 "0x%lX"
#define UID_WORD1 "0x%lX"
#endif /* defined(__ARMCC_VERSION) */

typedef struct {
    uint8_t TransmissionType;
    uint8_t ID;
    uint8_t Destination;
    uint8_t Payload[4];
} Packet;

typedef struct {
    uint8_t temp;
    uint8_t hum;
    uint8_t hopcount;
    uint8_t id_assign;
    uint8_t assignedID;
} NodeEntry;

#define MAX_NODES 50

void CreateLPAWURFrameV2();
void SendPacket();
void MX_APPE_Process(void);
void UpdateRssiStats(int16_t rssi, int print_stats);
void GotoRx(uint8_t* PR);
void MX_APPE_Idle(void);
void GETUID(uint8_t *uid);
void DiscoveryPhaseHandler(Packet* rxPacketptr);
void AssignID(uint8_t newID);

extern Packet txPacket;
extern Packet rxPacket;
extern uint8_t vectcTxBuff[15];

#endif
