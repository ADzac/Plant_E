/*
 * main.c
 *
 * STM32WL3 LPAWUR Mesh Network Node Implementation
 *
 * This file implements a mesh networking node using STM32WL3's dual-radio architecture:
 * - MRSubG (Multi-Radio Sub-GHz): Main communication radio for data transmission
 * - LPAWUR (Low Power Always-On Wake-Up Radio): Ultra-low power wake-up receiver
 *
 * The system enables energy-efficient mesh networking where nodes can remain in
 * deep sleep and wake up only when receiving wake-up signals via LPAWUR.
 *
 *  Created on: Jun 6, 2025
 *      Author: mzakri
 */

#include "main.h"

// Global configuration structures for radio subsystems
SMRSubGConfig MRSUBG_RadioInitStruct;           // MRSubG radio configuration
MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct; // MRSubG packet format settings
SLPAWUR_RFConfig LPAWUR_RadioInitStruct;        // LPAWUR RF configuration
SLPAWUR_FrameInit LPAWUR_FrameInitStruct;       // LPAWUR frame structure settings

/*----------------------------------------------------------------------------*/
/* Function Prototypes */
/*----------------------------------------------------------------------------*/
void SystemClock_Config(void);          // Configure system clocks
void PeriphCommonClock_Config(void);    // Configure peripheral clocks
static void MX_MRSUBG_Init(void);       // Initialize MRSubG radio
static void MX_GPIO_Init(void);         // Initialize GPIO pins
void UTIL_LPM_Init(void);               // Initialize Low Power Manager
void RX_TX_Init(void);                  // Initialize communication interfaces
static void MX_LPAWUR_Init(void);       // Initialize LPAWUR wake-up radio
static void MX_RNG_Init(void);          // Initialize Random Number Generator
void MX_I2C2_Init(void);                // Initialize I2C2 interface

/*----------------------------------------------------------------------------*/
/* Main Application Entry Point */
/*----------------------------------------------------------------------------*/
int main(void)
{
    // Initialize HAL (Hardware Abstraction Layer)
    HAL_Init();

    // Configure system clocks for optimal performance
    SystemClock_Config();
    PeriphCommonClock_Config();

    // Initialize hardware peripherals
    MX_GPIO_Init();         // Configure GPIO pins for LEDs, buttons, etc.
    MX_MRSUBG_Init();       // Setup main communication radio (865MHz, OOK modulation)
    MX_I2C2_Init();         // Initialize I2C for sensor communication
    MX_LPAWUR_Init();       // Setup ultra-low power wake-up radio
    UTIL_LPM_Init();        // Configure low power management
    MX_RNG_Init();          // Initialize hardware random number generator
    RX_TX_Init();           // Setup UART communication and radio buffers

    // Initialize mesh networking components
    GETUID(UID);            // Get unique device identifier for mesh addressing
    InitRoutingTable();     // Initialize mesh routing table

    // Initialize packet cache to prevent duplicate processing
    myCache.lastSenderID = 255;    // Invalid sender ID (uninitialized state)
    myCache.lastTransType = 255;   // Invalid transaction type
    myCache.lastRxTime = 0;        // No previous reception time
    datareqSent = 0;              // No data request sent yet

    // Print startup message via UART
    printf("STM32WL3 LPAWUR - Mesh Node.\n\r");

    // Main application loop - alternates between RX and low power modes
    while (1)
    {
        // Enter receive mode and process incoming packets
        GotoRx(vectcTxBuff);    // Start receiving with TX buffer for responses

        // Enter idle/low power mode until next event
        MX_APPE_Idle();         // Application-specific idle processing
    }
}

/*----------------------------------------------------------------------------*/
/* Communication Interface Initialization */
/*----------------------------------------------------------------------------*/
void RX_TX_Init(void)
{
    // Configure UART communication parameters
    COM_InitTypeDef COM_Init = {0};
    COM_Init.BaudRate = 115200;              // Standard baud rate for debugging
    COM_Init.HwFlowCtl = COM_HWCONTROL_NONE; // No hardware flow control
    COM_Init.WordLength = COM_WORDLENGTH_8B; // 8-bit data words
    COM_Init.Parity = COM_PARITY_NONE;       // No parity checking
    COM_Init.StopBits = COM_STOPBITS_1;      // Single stop bit
    BSP_COM_Init(COM1, &COM_Init);           // Initialize COM1 port

    // Initialize low power management utilities
    UTIL_LPM_Init();

    // Initialize status LEDs for visual feedback
    BSP_LED_Init(LD1);      // LED 1 - typically for status/activity
    BSP_LED_Init(LD2);      // LED 2 - typically for error/warning
    BSP_LED_Init(LD3);      // LED 3 - typically for network state

    // Initialize user button (if needed for manual operations)
    BSP_PB_Init(B2, BUTTON_MODE_GPIO);

    // Configure MRSubG radio packet settings
    HAL_MRSubG_PktBasicSetPayloadLength(15);    // Set packet payload to 15 bytes
    LL_MRSubG_PacketHandlerManchesterType(MANCHESTER_TYPE0); // Manchester encoding type
    __HAL_MRSUBG_SET_TX_MODE(TX_NORMAL);        // Normal transmission mode
    __HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t)&vectcTxBuff); // Set TX buffer pointer
}

/*----------------------------------------------------------------------------*/
/* Random Number Generator Initialization */
/*----------------------------------------------------------------------------*/
static void MX_RNG_Init(void)
{
    /* Enable RNG peripheral clock */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_RNG);

    /* Enable the Random Number Generator */
    LL_RNG_Enable(RNG);

    /* RNG is used for:
     * - Random backoff delays in mesh networking
     * - Cryptographic operations
     * - Collision avoidance in radio transmissions
     */
}

/*----------------------------------------------------------------------------*/
/* MRSubG Radio Initialization (Main Communication Radio) */
/*----------------------------------------------------------------------------*/
static void MX_MRSUBG_Init(void)
{
    /* Configure radio RF parameters */
    MRSUBG_RadioInitStruct.lFrequencyBase = 865000000;      // Base frequency: 865 MHz (SRD band)
    MRSUBG_RadioInitStruct.xModulationSelect = MOD_OOK;     // OOK modulation (On-Off Keying)
    MRSUBG_RadioInitStruct.lDatarate = 2000;                // Data rate: 2 kbps (low for better range)
    MRSUBG_RadioInitStruct.lFreqDev = 20000;                // Frequency deviation: 20 kHz
    MRSUBG_RadioInitStruct.lBandwidth = 50000;              // Channel bandwidth: 50 kHz
    MRSUBG_RadioInitStruct.dsssExp = 0;                     // DSSS disabled
    MRSUBG_RadioInitStruct.outputPower = 14;                // TX power: 14 dBm (max allowed)
    MRSUBG_RadioInitStruct.PADrvMode = PA_DRV_TX_HP;        // High power PA driver mode

    HAL_MRSubG_Init(&MRSUBG_RadioInitStruct); // Apply radio configuration

    /* Configure packet structure parameters */
    MRSUBG_PacketSettingsStruct.PreambleLength = 0;         // No preamble (minimal overhead)
    MRSUBG_PacketSettingsStruct.PostambleLength = 0;        // No postamble
    MRSUBG_PacketSettingsStruct.SyncLength = 0;             // No sync word
    MRSUBG_PacketSettingsStruct.SyncWord = 0x88888888;      // Sync pattern (not used)
    MRSUBG_PacketSettingsStruct.FixVarLength = FIXED;       // Fixed packet length
    MRSUBG_PacketSettingsStruct.PreambleSequence = PRE_SEQ_0101; // Preamble pattern
    MRSUBG_PacketSettingsStruct.PostambleSequence = POST_SEQ_0101; // Postamble pattern
    MRSUBG_PacketSettingsStruct.CrcMode = PKT_NO_CRC;       // No CRC (application layer handles integrity)
    MRSUBG_PacketSettingsStruct.Coding = CODING_MANCHESTER; // Manchester encoding for DC balance
    MRSUBG_PacketSettingsStruct.DataWhitening = DISABLE;    // No data whitening
    MRSUBG_PacketSettingsStruct.LengthWidth = BYTE_LEN_1;   // 1-byte length field
    MRSUBG_PacketSettingsStruct.SyncPresent = DISABLE;      // No sync detection

    HAL_MRSubG_PacketBasicInit(&MRSUBG_PacketSettingsStruct); // Apply packet configuration
}

/*----------------------------------------------------------------------------*/
/* LPAWUR Initialization (Low Power Always-On Wake-Up Radio) */
/*----------------------------------------------------------------------------*/
static void MX_LPAWUR_Init(void)
{
    /* Configure LPAWUR RF parameters for ultra-low power operation */
    LPAWUR_RadioInitStruct.EnergyDetectorIcal = ED_ICAL_VBAT_3_25_TO_3_50; // Energy detector calibration for 3.3V
    LPAWUR_RadioInitStruct.ClockDivider = 7;                    // Clock division for low power
    LPAWUR_RadioInitStruct.EnergyDetectorSwitch = DISABLE;      // Energy detector disabled
    LPAWUR_RadioInitStruct.AgcResetMode = AGC_RESET_MODE_NEVER; // AGC never resets
    LPAWUR_RadioInitStruct.AgcHoldMode = AGC_HOLD_AFTER_PREAMBLE; // Hold AGC after preamble
    LPAWUR_RadioInitStruct.AgcMode = AGC_MODE_OFF;              // AGC disabled for consistency
    LPAWUR_RadioInitStruct.AgcHiLvl = AGC_VBAT_0800;           // High AGC threshold
    LPAWUR_RadioInitStruct.DCCurrentSubtraction = ENABLE;       // Enable DC offset compensation
    LPAWUR_RadioInitStruct.AgcLoLvl = AGC_LOW_0;                // Low AGC threshold

    HAL_LPAWUR_RFConfigInit(&LPAWUR_RadioInitStruct); // Apply LPAWUR RF configuration

    /* Configure LPAWUR frame detection parameters */
    LPAWUR_FrameInitStruct.TRecAlgoSel = TWO_STEPS;         // Two-step recovery algorithm
    LPAWUR_FrameInitStruct.SlowClkCyclePerBitCnt = 16;      // Clock cycles per bit
    LPAWUR_FrameInitStruct.PayloadLength = 7;               // Wake-up frame payload: 7 bytes
    LPAWUR_FrameInitStruct.SyncThr = 16;                    // Sync detection threshold
    LPAWUR_FrameInitStruct.SyncLength = 0;                  // No sync field
    LPAWUR_FrameInitStruct.PreambleThrCnt = 0x3C;           // Preamble detection threshold
    LPAWUR_FrameInitStruct.PreambleEnable = ENABLE;         // Enable preamble detection
    LPAWUR_FrameInitStruct.FrameSyncCntTimeout = 0x60;      // Frame sync timeout
    LPAWUR_FrameInitStruct.FrameSyncPattenHigh = 0x00;      // Sync pattern high bits
    LPAWUR_FrameInitStruct.FrameSyncPatternLow = 38550;     // Sync pattern low bits (0x9696)
    LPAWUR_FrameInitStruct.KpGain = 6;                      // Proportional gain for clock recovery
    LPAWUR_FrameInitStruct.KiGain = 10;                     // Integral gain for clock recovery

    HAL_LPAWUR_FrameInit(&LPAWUR_FrameInitStruct); // Apply frame configuration

    /* Enable LPAWUR receiver - now always listening for wake-up signals */
    LL_LPAWUR_SetState(ENABLE);
}

/*----------------------------------------------------------------------------*/
/* System Clock Configuration */
/*----------------------------------------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure oscillators:
     * - HSE: High Speed External oscillator (crystal/resonator)
     * - LSE: Low Speed External oscillator (32.768 kHz crystal for RTC)
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;    // Enable HSE for stable high-frequency operation
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;    // Enable LSE for low power timekeeping

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler(); // Critical error - system cannot continue
    }

    /** Configure system clock source and divider
     * - Use RC64MPLL: 64 MHz RC oscillator with PLL for high performance
     * - No division: Full speed operation
     */
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_RC64MPLL;  // 64 MHz system clock
    RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;        // No division (64 MHz)

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1) != HAL_OK)
    {
        Error_Handler(); // Critical error - system cannot continue
    }
}

/*----------------------------------------------------------------------------*/
/* Peripheral Clock Configuration */
/*----------------------------------------------------------------------------*/
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Configure SMPS (Switched Mode Power Supply) clock
     * - SMPS provides efficient power conversion for RF circuits
     * - Divide by 4 for optimal switching frequency
     * - KRM rate multiplier = 4 for balanced performance/efficiency
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
    PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;    // SMPS clock / 4
    PeriphClkInitStruct.KRMRateMultiplier = 4;                  // KRM multiplier

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler(); // Critical error - power management failure
    }
}

/*----------------------------------------------------------------------------*/
/* GPIO Initialization */
/*----------------------------------------------------------------------------*/
void MX_GPIO_Init(void)
{
    /* Enable GPIO clocks for ports used by the application
     * - GPIOA: Typically used for analog inputs, UART, SPI
     * - GPIOB: Typically used for LEDs, buttons, I2C
     */
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA peripheral clock
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable GPIOB peripheral clock

    /* Note: Specific GPIO pin configurations are handled by BSP functions
     * called in RX_TX_Init() and other initialization routines */
}

/*----------------------------------------------------------------------------*/
/* Debug Assert Handler */
/*----------------------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation here to report the file name and line number,
     * for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
     * This function is called when an assertion fails during development/debug.
     */
}
#endif /* USE_FULL_ASSERT */
