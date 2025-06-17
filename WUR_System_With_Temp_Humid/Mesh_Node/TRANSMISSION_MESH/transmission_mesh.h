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

typedef struct {
    uint8_t TransmissionType;
    uint8_t ID;
    uint8_t Destination;
    uint8_t Temperature;
    uint8_t Humidity;
    uint8_t ADD;
    uint8_t ID_Assign;
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

extern uint8_t Uid[3];
extern Packet txPacket;
extern uint8_t vectcTxBuff[15];

#endif
