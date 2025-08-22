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
#define ID_RECEIVED 3
#define DATAREQ 4
#define DATAREP 5

#define ALERT 10

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
    uint8_t Destination; // for hop cases, if missing node than try to reach it
    uint8_t Payload[4];
} Packet;

typedef enum {
    NODE_STATUS_UNKNOWN = 0,
    NODE_STATUS_DISCOVERED,
    NODE_STATUS_RESPONDING,
    NODE_STATUS_MISSING,
    NODE_STATUS_TIMEOUT
} NodeStatus;

typedef struct {
    uint8_t uid[4];
    uint8_t assignedID;
    NodeStatus status;
    uint32_t lastSeen;
    uint8_t missedResponses;
} NodeEntry;

typedef struct {
    uint8_t nodeID;
    uint8_t temperature;
    uint8_t humidity;
    float battery;
} SensorData;

// Structure to track received data reports
typedef struct {
    uint8_t id;
    uint8_t temp;
    uint8_t humid;
    uint8_t received;  // Flag to indicate if we've received data from this ID
} DataReport;

#define MAX_NODES 255

void CreateLPAWURFrameV2();
void SendPacket();
void MX_APPE_Process(void);
void UpdateRssiStats(int16_t rssi, int print_stats);
void GotoRx(void);
void MX_APPE_Idle(void);
void GETUID(uint8_t *uid);
void DiscoveryPhaseHandler(Packet* rxPacketptr);
void AssignID(uint8_t newID);
void init_data_storage();
uint8_t compareUIDs(uint8_t *uid1, uint8_t *uid2) ;
int8_t getAssignedID(uint8_t *uid);
uint8_t assignIDToUID(uint8_t *uid) ;
void AddToIDList(uint8_t id);
void SendToDataBase(void);
void UpdateNodeStatus(uint8_t nodeID, NodeStatus status);
void HandleDataCollection(void);
void SendDataRequestToMissingNodes(void);

extern Packet txPacket;
extern Packet rxPacket;
extern uint8_t vectcTxBuff[15];
extern uint8_t alreadyReceived;
extern uint8_t firstResponseTime;
extern uint8_t collectionPhase;
extern uint8_t retryCount;

extern uint8_t IDList[255];  // Start with all zeros
extern uint8_t IDListSize;
#endif
