
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
#include "stm32wl3x_ll_rng.h"
#include<stdlib.h>
#include<stdio.h>

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
#define MAX_CACHE 10

typedef struct {
	uint8_t senderID;     // Who sent this?
	uint8_t packetType;   // DISCOVERY_REQ, DATAREP, etc.
	uint8_t seqNum;       // Optional: Sequence number (if available)
	uint32_t timestamp;
} PacketSignature;

typedef struct {
    uint8_t lastSenderID;      // Who sent the last DISCOVERY/DATAREQ?
    uint8_t lastTransType;     // Was it DISCOVERY_REQ (0) or DATAREQ (3)?
    uint32_t lastRxTime;       // When was it received?
} SimpleCache;

typedef struct {
    uint8_t TransmissionType;
    uint8_t ID;
    uint8_t Destination; //only for missing node , hop for other node
    uint8_t Payload[4]; // only for first discovery than can be use for something else i.e. TTL "2" for hop "3" for TTL
} Packet;

#define MAX_ROUTES 10

typedef struct {
    uint8_t nodeID;
    uint8_t nextHop;
    uint8_t hopCount;
    uint32_t lastSeen; // time in ms
} RoutingEntry;


// Add these function prototypes
void ProcessDiscoveryPacket(Packet* rxPacket);
void SendDiscoveryResponse(uint8_t j, uint8_t* vectcTxBuff);
void PrepareDiscoveryResponse(Packet *Pack);
void SendAckALIVE(Packet *Pack);

void CreateLPAWURFrameV2(uint8_t* vectcTxBuff);
void SendPacket(uint8_t* vectcTxBuff);
void MX_APPE_Process(void);
void GotoRx(uint8_t* vectcTxBuff);
void MX_APPE_Idle(void);
void GETUID(uint8_t *uid);
void forwardPacket(uint8_t type);
uint8_t isDuplicate(Packet *pkt);
void InitRoutingTable(void);
void ResetCache(void) ;
void UpdateRoutingTable(uint8_t nodeID, uint8_t nextHop, uint8_t hopCount);
void printRoutingTable(void) ;

extern uint8_t UID[4];

extern SimpleCache myCache;
extern uint8_t vectcTxBuff[15];
extern PacketSignature packetCache[MAX_CACHE];
extern uint8_t datareqSent;
extern RoutingEntry routingTable[MAX_ROUTES];
#endif
