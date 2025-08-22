#include "main.h"

/* Wake-up timeout values in milliseconds */
#define WAKEUP_TIMEOUT_STD   60000   // Standard timeout (60 seconds)
#define WAKEUP_TIMEOUT_RETRY 10000   // Retry timeout (10 seconds)

/* Global variables */
uint8_t wakeup_counter = 24;         // Counts wakeup cycles
uint8_t TYPE = DISCOVERY_REQ;        // Packet type (default: discovery request)
uint32_t wakeupSource2;

uint8_t vectcTxBuffV2[15];           // Transmission buffer (15 bytes)

uint8_t mode = 0;                    // Radio mode: TX=0, RX=1

float temperature = 0;               // Placeholder for sensor reading
float humidity = 0;                  // Placeholder for sensor reading

int startTimeout;                    // Timestamp for timeout handling

Packet myPacket;                     // Custom packet structure

/* Radio configuration structures */
SMRSubGConfig MRSUBG_RadioInitStruct;
MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct;
SLPAWUR_RFConfig LPAWUR_RadioInitStruct;
SLPAWUR_FrameInit LPAWUR_FrameInitStruct;

/*----------------------------------------------------------------------------*/
/* Function prototypes */
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_MRSUBG_Init(void);
static void MX_GPIO_Init(void);
void UTIL_LPM_EnterLowPower(void);
void UTIL_LPM_Init(void);
void RX_TX_Init(void);
static void MX_LPAWUR_Init(void);
void UTIL_LPM_Init(void);
void MX_I2C2_Init(void);
void MX_RTC_Init(void);
void configRTCWakeupTimer(uint32_t timeout);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
/*----------------------------------------------------------------------------*/

int main(void)
{
    /* Initialize HAL library and hardware abstraction */
    HAL_Init();

    /* System clock and peripheral configuration */
    SystemClock_Config();
    PeriphCommonClock_Config();
    MX_GPIO_Init();
    MX_MRSUBG_Init();   // Configure main Sub-GHz radio
    MX_I2C2_Init();     // Initialize I2C2 (for sensors)
    MX_LPAWUR_Init();   // Configure Low-Power Wake-Up Radio
    UTIL_LPM_Init();    // Initialize low power manager
    RX_TX_Init();       // Configure TX/RX settings
    MX_RTC_Init();      // Configure RTC
    configRTCWakeupTimer(WAKEUP_TIMEOUT_STD);  // Start periodic wake-up

    collectionPhase = 0;

    /* Initialize first transmission packet */
    txPacket.TransmissionType = TYPE;
    txPacket.ID = MAIN_NODE_ID;
    txPacket.Destination = UNASSIGNED_ID;

    printf("STM32WL3 LPAWUR - Main Node Started\n\r");

    while (1)
    {
        /* Handle transmission phase */
        if (mode == 0) {
            SendPacket();  // Send packet
            mode = 1;      // Switch to RX mode
        }

        /* Handle reception phase */
        GotoRx();

        /* Manage data collection with timeout */
        HandleDataCollection();

        /* Enter low power mode while waiting */
        MX_APPE_Idle();
    }
}

/*----------------------------------------------------------------------------*/
/* RTC wakeup timer callback
   Called whenever the RTC wakeup timer expires.
   Used to manage the communication cycle between nodes. */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF); // Clear wake-up flag
    mode = 0;  // Back to TX mode at each wakeup

    printf("======================== New cycle Starto : %d ======================== \n\r", wakeup_counter);

    if (wakeup_counter >= 24) {
        /* Perform a discovery request every 24 cycles */
        TYPE = DISCOVERY_REQ;
        BSP_LED_Toggle(LD3);                  // Blink LD3 for discovery
        configRTCWakeupTimer(WAKEUP_TIMEOUT_RETRY);

        if (wakeup_counter == 28) {
            /* Reset counter after 28 cycles */
            wakeup_counter = 0;
            configRTCWakeupTimer(WAKEUP_TIMEOUT_STD);
        }
    } else {
        /* Send data request during normal cycle */
        retryCount = 0;
        TYPE = DATAREQ;
        BSP_LED_Toggle(LD1);                  // Blink LD1 for data request
    }

    /* Handle collection process */
    if (collectionPhase == 1) {
        /* Still collecting → retry sooner */
        configRTCWakeupTimer(WAKEUP_TIMEOUT_RETRY);
    }

    if (collectionPhase == 2 && wakeup_counter % 4 == 0) {
        /* Every 4th cycle, send to database */
        SendToDataBase();
        collectionPhase = 0;

        /* Reset to standard cycle */
        configRTCWakeupTimer(WAKEUP_TIMEOUT_STD);

        /* Prepare for next cycle */
        init_data_storage();
        startTimeout = HAL_GetTick();
        mode = 1; // Switch to RX
        return;
    }

    /* Update counters and prepare packet for next round */
    wakeup_counter++;
    txPacket.ID = MAIN_NODE_ID;
    txPacket.Destination = MAIN_NODE_ID;
    txPacket.TransmissionType = TYPE;
    txPacket.Payload[2] = 0;
    txPacket.Payload[3] = 5;
}

/*----------------------------------------------------------------------------*/
/* Initialize UART, LEDs, packet handling, and Sub-GHz radio TX settings */
void RX_TX_Init(void) {
    COM_InitTypeDef COM_Init = {0};

    COM_Init.BaudRate   = 115200;
    COM_Init.HwFlowCtl  = COM_HWCONTROL_NONE;
    COM_Init.WordLength = COM_WORDLENGTH_8B;
    COM_Init.Parity     = COM_PARITY_NONE;
    COM_Init.StopBits   = COM_STOPBITS_1;

    BSP_COM_Init(COM1, &COM_Init);

    UTIL_LPM_Init();
    BSP_LED_Init(LD1);
    BSP_LED_Init(LD2);
    BSP_LED_Init(LD3);

    BSP_PB_Init(B2, BUTTON_MODE_GPIO); // Optional push button init

    HAL_MRSubG_PktBasicSetPayloadLength(15); // Set packet payload length
    LL_MRSubG_PacketHandlerManchesterType(MANCHESTER_TYPE0);
    __HAL_MRSUBG_SET_TX_MODE(TX_NORMAL);
    __HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t)&vectcTxBuff);
}

/*----------------------------------------------------------------------------*/
/* Configure Sub-GHz radio parameters */
static void MX_MRSUBG_Init(void)
{
    /* Radio configuration */
    MRSUBG_RadioInitStruct.lFrequencyBase   = 865000000; // 865 MHz band
    MRSUBG_RadioInitStruct.xModulationSelect = MOD_OOK;  // OOK modulation
    MRSUBG_RadioInitStruct.lDatarate        = 2000;      // 2 kbps
    MRSUBG_RadioInitStruct.lFreqDev         = 20000;
    MRSUBG_RadioInitStruct.lBandwidth       = 50000;
    MRSUBG_RadioInitStruct.dsssExp          = 0;
    MRSUBG_RadioInitStruct.outputPower      = 14;        // 14 dBm
    MRSUBG_RadioInitStruct.PADrvMode        = PA_DRV_TX_HP;

    HAL_MRSubG_Init(&MRSUBG_RadioInitStruct);

    /* Packet configuration */
    MRSUBG_PacketSettingsStruct.PreambleLength     = 0;
    MRSUBG_PacketSettingsStruct.PostambleLength    = 0;
    MRSUBG_PacketSettingsStruct.SyncLength         = 0;
    MRSUBG_PacketSettingsStruct.SyncWord           = 0x88888888;
    MRSUBG_PacketSettingsStruct.FixVarLength       = FIXED;
    MRSUBG_PacketSettingsStruct.PreambleSequence   = PRE_SEQ_0101;
    MRSUBG_PacketSettingsStruct.PostambleSequence  = POST_SEQ_0101;
    MRSUBG_PacketSettingsStruct.CrcMode            = PKT_NO_CRC;
    MRSUBG_PacketSettingsStruct.Coding             = CODING_MANCHESTER;
    MRSUBG_PacketSettingsStruct.DataWhitening      = DISABLE;
    MRSUBG_PacketSettingsStruct.LengthWidth        = BYTE_LEN_1;
    MRSUBG_PacketSettingsStruct.SyncPresent        = DISABLE;

    HAL_MRSubG_PacketBasicInit(&MRSUBG_PacketSettingsStruct);
}

/*----------------------------------------------------------------------------*/
/* Configure Low-Power Wake-Up Radio (LPAWUR) */
static void MX_LPAWUR_Init(void)
{
    /* RF configuration */
    LPAWUR_RadioInitStruct.EnergyDetectorIcal   = ED_ICAL_VBAT_3_25_TO_3_50;
    LPAWUR_RadioInitStruct.ClockDivider         = 7;
    LPAWUR_RadioInitStruct.EnergyDetectorSwitch = DISABLE;
    LPAWUR_RadioInitStruct.AgcResetMode         = AGC_RESET_MODE_NEVER;
    LPAWUR_RadioInitStruct.AgcHoldMode          = AGC_HOLD_AFTER_PREAMBLE;
    LPAWUR_RadioInitStruct.AgcMode              = AGC_MODE_OFF;
    LPAWUR_RadioInitStruct.AgcHiLvl             = AGC_VBAT_0800;
    LPAWUR_RadioInitStruct.DCCurrentSubtraction = ENABLE;
    LPAWUR_RadioInitStruct.AgcLoLvl             = AGC_LOW_0;

    HAL_LPAWUR_RFConfigInit(&LPAWUR_RadioInitStruct);

    /* Frame detection configuration */
    LPAWUR_FrameInitStruct.TRecAlgoSel          = TWO_STEPS;
    LPAWUR_FrameInitStruct.SlowClkCyclePerBitCnt = 16;
    LPAWUR_FrameInitStruct.PayloadLength        = 7;
    LPAWUR_FrameInitStruct.SyncThr              = 16;
    LPAWUR_FrameInitStruct.SyncLength           = 0;
    LPAWUR_FrameInitStruct.PreambleThrCnt       = 0x3C;
    LPAWUR_FrameInitStruct.PreambleEnable       = ENABLE;
    LPAWUR_FrameInitStruct.FrameSyncCntTimeout  = 0x60;
    LPAWUR_FrameInitStruct.FrameSyncPattenHigh  = 0x00;
    LPAWUR_FrameInitStruct.FrameSyncPatternLow  = 38550;
    LPAWUR_FrameInitStruct.KpGain               = 6;
    LPAWUR_FrameInitStruct.KiGain               = 10;

    HAL_LPAWUR_FrameInit(&LPAWUR_FrameInitStruct);

    /* Enable wake-up radio */
    LL_LPAWUR_SetState(ENABLE);
}

/*----------------------------------------------------------------------------*/
/* System clock configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Enable HSE and LSE oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Configure system clock source */
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_RC64MPLL;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1) != HAL_OK) {
        Error_Handler();
    }
}

/*----------------------------------------------------------------------------*/
/* Peripheral clock configuration */
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
    PeriphClkInitStruct.SmpsDivSelection     = RCC_SMPSCLK_DIV4;
    PeriphClkInitStruct.KRMRateMultiplier    = 4;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
}

/*----------------------------------------------------------------------------*/
/* GPIO initialization */
void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/*----------------------------------------------------------------------------*/
#ifdef USE_FULL_ASSERT
/* Error handler for invalid parameters */
void assert_failed(uint8_t *file, uint32_t line)
{
    // User can add debug output here
}
#endif /* USE_FULL_ASSERT */
