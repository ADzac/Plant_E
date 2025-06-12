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

#define DISCOVERY 0
#define DATAREQUEST 1
#define ALERT 2

typedef struct {
    uint8_t TransmissionType;
    uint8_t ID;
    uint8_t Destination;
    float Temperature;
    float Humidity;
    uint8_t Dunno;
    uint8_t Dunno2;
} Packet;

void CreateLPAWURFrameV2(Packet* packet, uint8_t j, uint8_t* vectcTxBuff);
void RandomNumbersGeneration(Packet* p,uint8_t j,uint8_t* vectcTxBuff);
void MX_APPE_Process(void);
void UpdateRssiStats(int16_t rssi, int print_stats);
void GotoRx(uint8_t* PR);
void MX_APPE_Idle(void);



#endif
