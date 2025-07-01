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
#include <stdlib.h>
#include "math.h"
#include "app_conf.h"
#include "crc_4wkup_rf.h"
#include "stm32_lpm.h"
#include "temphumid.h"

#define DISCOVERY_REQ 0
#define DISCOVERY_RESP 1
#define ID_ASSIGNMENT 2
#define DATAREQ 3
#define DATAREP 4

#define ALERT 5

#define MAIN_NODE_ID 0
#define UNASSIGNED_ID 254


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
    uint8_t uid[4];  // Assuming UID is 4 bytes
    uint8_t assignedID;
} NodeEntry;

typedef struct {
    uint8_t nodeID;
    float temperature;
    float humidity;
    float battery;
} SensorData;

// Structure to track received data reports
typedef struct {
    uint8_t id;
    float temp;
    float humid;
    uint8_t received;  // Flag to indicate if we've received data from this ID
} DataReport;

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
void init_data_storage();
uint8_t compareUIDs(uint8_t *uid1, uint8_t *uid2) ;
int8_t getAssignedID(uint8_t *uid);
uint8_t assignIDToUID(uint8_t *uid) ;
void AddToIDList(uint8_t id);

extern Packet txPacket;
extern Packet rxPacket;
extern uint8_t vectcTxBuff[15];
extern uint8_t alreadyReceived;

static DataReport receivedData[MAX_NODES];
static uint8_t receivedDataCount;

extern uint8_t IDList[255];  // Start with all zeros
extern uint8_t IDListSize;
#endif
