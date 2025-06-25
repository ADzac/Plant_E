
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
#define DATAREQ 3
#define DATAREP 4

#define ALERT 5

#define MAIN_NODE_ID 0
#define UNASSIGNED_ID 0xFF
#define MAX_CACHE 10

typedef struct {
    uint8_t valid;
    uint8_t sender;
    uint8_t transType;
    uint8_t temp;
    uint8_t humid;
    uint8_t hop;
} PacketSignature;


typedef struct {
    uint8_t TransmissionType;
    uint8_t ID;
    uint8_t Destination; //only for first discovery
    uint8_t UID[4];
    uint8_t Payload[4]; // only for first discovery than can be use for something else i.e. TTL
} Packet;

// Add these function prototypes
void ProcessDiscoveryPacket(Packet* rxPacket);
void SendDiscoveryResponse(uint8_t j, uint8_t* vectcTxBuff);

void CreateLPAWURFrameV2(uint8_t j, uint8_t* vectcTxBuff);
void RandomNumbersGeneration(uint8_t j,uint8_t* vectcTxBuff);
void MX_APPE_Process(void);
void UpdateRssiStats(int16_t rssi, int print_stats);
void GotoRx(uint8_t* PR,uint8_t* vectcTxBuff);
void MX_APPE_Idle(void);
void GETUID(uint8_t *uid);

extern uint8_t UID[4];
extern Packet txPacket;
extern uint8_t vectcTxBuff[15];
extern PacketSignature packetCache[MAX_CACHE];
#endif
