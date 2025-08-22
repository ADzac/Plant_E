/*
 * txrx.c - Wireless Sensor Network Main Controller
 *
 * This file implements the main controller/coordinator functionality for a
 * wireless sensor network using LPAWUR (Low Power Asynchronous Wake-Up Radio).
 *
 * Main functionalities:
 * - Device discovery and ID assignment
 * - Data collection from sensor nodes
 * - Network topology management
 * - Power management and low-power operations
 *
 * Created on: Jun 6, 2025
 * Author: mzakri
 */
#include "transmission_main.h"

// Communication protocol constants
#define PAYLOAD_LEN 7                    // Maximum payload length for packets
#define MIN(a,b) (((a) < (b)) ? (a) : (b)) // Utility macro for minimum value

// Communication buffers
uint8_t vectcTxBuff[15];                 // Transmission buffer for LPAWUR frames
uint8_t LPAWUR_Payload[8];               // Received payload buffer

// Network identification and management
uint8_t ID = 1;                          // Current node's ID (main controller)
uint8_t checkForID = 5;                  // ID validation/checking value
uint8_t IDList[255];                     // List of active/discovered node IDs
uint8_t IDListSize = 0;                  // Number of IDs currently in the list
uint8_t alreadyReceived;                 // Flag to track duplicate receptions

// Radio signal strength monitoring
int16_t rssi = 0;                        // Current RSSI value
int16_t rssi_min = 0;                    // Minimum recorded RSSI
int16_t rssi_max = -150;                 // Maximum recorded RSSI

// Packet structures for TX and RX operations
Packet txPacket;                         // Outgoing packet structure
Packet rxPacket;                         // Incoming packet structure

// Environmental sensor data
float temp = 0;                          // Temperature reading
float humid = 0;                         // Humidity reading

// Node management and ID assignment
uint8_t nextAvailableID = 1;             // Next ID to assign to new nodes
NodeEntry knownNodes[MAX_NODES];         // Database of known network nodes
uint8_t nodeCount = 0;                   // Current number of known nodes
uint8_t uid[4];                          // Unique identifier buffer
uint8_t assignedID;                      // ID assigned to current node
uint8_t SendDatabase = 0;                // Flag to control database reporting

// Timing and response tracking
uint8_t firstResponseTime;               // Timestamp of first response received

// Data collection state machine configuration
#define DATA_COLLECTION_TIMEOUT 30000    // 30 seconds timeout for data collection
#define MAX_RETRIES 3                    // Maximum retry attempts for missing data

// Data collection state variables
uint32_t dataCollectionStartTime;        // When current collection cycle started
uint8_t collectionPhase;                 // State: 0=idle, 1=collecting, 2=complete
uint8_t retryCount;                      // Current retry attempt number

// Data storage for collected sensor readings
static DataReport receivedData[MAX_NODES]; // Array to store received sensor data
static uint8_t receivedDataCount;         // Number of successful data receptions

//----------------------------- Data Storage Management ------------------------------------
/**
 * Initialize all data storage structures and reset collection state
 * Called at startup or when resetting the network
 */
void init_data_storage(void) {
    // Clear all received data entries
    memset(receivedData, 0, MAX_NODES*sizeof(receivedData[0]));
    receivedDataCount = 0;      // Reset data counter
    alreadyReceived = 0;        // Clear duplicate reception flag
    SendDatabase = 0;           // Reset database send flag
    firstResponseTime = 0;      // Clear first response timestamp
}

//----------------------------- Node ID Management ------------------------------------
/**
 * Extract unique identifier (UID) from microcontroller hardware registers
 * Each device has a unique 96-bit ID that we compress to 32 bits
 * @param uid: 4-byte buffer to store the extracted UID
 */
void GETUID(uint8_t *uid) {
    // Read 96-bit unique ID from hardware registers
    uint32_t uid0 = LL_GetUID_Word0();  // Lower 32 bits
    uint32_t uid1 = LL_GetUID_Word1();  // Upper 32 bits

    // Extract and pack into 4 bytes for transmission efficiency
    uid[0] = (uid0 >> 24) & 0xFF;       // Most significant byte of word0
    uid[1] = (uid0 >> 16) & 0xFF;       // Second byte of word0
    uid[2] = (uid1 >> 8) & 0xFF;        // Second byte of word1
    uid[3] = (uid1 >> 0) & 0xFF;        // Least significant byte of word1

    printf("UID = %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
}

/**
 * Compare two 4-byte UIDs for equality
 * @param uid1: First UID to compare
 * @param uid2: Second UID to compare
 * @return: 1 if UIDs match, 0 if different
 */
uint8_t compareUIDs(uint8_t *uid1, uint8_t *uid2) {
    for (int i = 0; i < 4; i++) {
        if (uid1[i] != uid2[i]) return 0; // Mismatch found
    }
    return 1; // All bytes match
}

/**
 * Look up the assigned ID for a given UID in the known nodes database
 * @param uid: The UID to search for
 * @return: Assigned ID if found, -1 if not found
 */
int8_t getAssignedID(uint8_t *uid) {
    for (uint8_t i = 0; i < nodeCount; i++) {
        if (compareUIDs(knownNodes[i].uid, uid)) {
            return knownNodes[i].assignedID;
        }
    }
    return -1;  // UID not found in database
}

/**
 * Assign a network ID to a device based on its UID
 * If the UID is already known, return existing ID
 * If new UID, assign next available ID and add to database
 * @param uid: The 4-byte UID of the device requesting an ID
 * @return: Assigned ID (1-254), or 0xFF if assignment failed
 */
uint8_t assignIDToUID(uint8_t *uid) {
    // Check if this UID already has an assigned ID
    int8_t existingID = getAssignedID(uid);
    if (existingID != -1) {
        return (uint8_t)existingID;  // Return existing assignment
    }

    // Check if we've reached maximum node capacity
    if (nodeCount >= MAX_NODES) {
        printf("Maximum node limit reached.\n\r");
        return 0xFF;  // Invalid ID indicates failure
    }

    // Assign new ID and add to database
    uint8_t newID = nextAvailableID;
    memcpy(knownNodes[nodeCount].uid, uid, 4);           // Store UID
    knownNodes[nodeCount].assignedID = newID;            // Store assigned ID
    UpdateNodeStatus(newID, NODE_STATUS_DISCOVERED);     // Set initial status
    nodeCount++;                                         // Increment node count

    printf("Assigned new ID %d to UID [%02X %02X %02X %02X]\n\r",
           newID, uid[0], uid[1], uid[2], uid[3]);

    nextAvailableID++;  // Prepare next ID for future assignments
    return newID;       // Return newly assigned ID
}

/**
 * Update the status and activity tracking for a network node
 * @param nodeID: The ID of the node to update
 * @param status: New status to set (discovered, responding, timeout, etc.)
 */
void UpdateNodeStatus(uint8_t nodeID, NodeStatus status) {
    // Find the node in our database
    for (int i = 0; i < nodeCount; i++) {
        if (knownNodes[i].assignedID == nodeID) {
            knownNodes[i].status = status;              // Update status
            knownNodes[i].lastSeen = HAL_GetTick();     // Update last seen timestamp

            // Reset missed response counter if node is responding
            if (status == NODE_STATUS_RESPONDING) {
                knownNodes[i].missedResponses = 0;
            }
            // Increment missed responses on timeout
            else if (status == NODE_STATUS_TIMEOUT) {
                knownNodes[i].missedResponses++;
            }
            break;
        }
    }
}

/**
 * Add a node ID to the active ID list if it's not already present
 * This list tracks which nodes are currently active in the network
 * @param id: Node ID to add to the list
 */
void AddToIDList(uint8_t id) {
    if (id == 0xFF) return;  // Don't add invalid ID

    // Check if ID already exists in the list
    for (uint8_t i = 0; i < IDListSize; i++) {
        if (IDList[i] == id) {
            return;  // ID already exists, no need to add
        }
    }

    // Add new ID if there's space in the list
    if (IDListSize < 255) {
        IDList[IDListSize++] = id;
        printf("Added ID %d to IDList\n\r", id);
        printf("ID List Size %d \n\r", IDListSize);
    }
    else {
        printf("IDList full, cannot add ID %d\n\r", id);
    }
}

/**
 * Check if a given ID exists in the active ID list
 * @param id: Node ID to search for
 * @return: 1 if ID found in list, 0 if not found
 */
uint8_t IsInIDList(uint8_t id) {
    for (uint8_t i = 0; i < IDListSize; i++) {
        if (IDList[i] == id) return 1;
    }
    return 0;
}

//----------------------------- Environmental Sensor Management ------------------------------------
/**
 * Initialize and read data from Si7021 temperature and humidity sensor
 * This function handles sensor initialization with error checking and LED indication
 */
void TempANDHumidSensor() {
    // Keep trying to initialize the sensor until successful
    while (Si7021_Init() != HAL_OK) {
        BSP_LED_On(LED_RED);  // Indicate initialization error
    }

    // Attempt to read temperature and humidity
    if (Si7021_ReadTempAndHumidity(&temp, &humid) == HAL_OK) {
        printf("temp: %.2f°C, humid: %.2f \n\r", temp, humid);
    } else {
        printf("Error reading Si7021 sensor!\n\r");
        BSP_LED_On(LED_RED);  // Indicate read error
    }
}

//----------------------------- Packet Processing Handlers ------------------------------------
/**
 * Handle device discovery phase - when new nodes request network access
 * This function processes discovery requests and assigns network IDs
 * @param rxPacketPtr: Pointer to received discovery packet
 */
void DiscoveryPhaseHandler(Packet* rxPacketPtr) {
    // 1. Extract 4-byte UID from the received discovery packet
    //    The UID is distributed across multiple packet fields for transmission efficiency
    uint8_t uid[4];
    uid[0] = rxPacketPtr->ID;           // First byte of UID
    uid[1] = rxPacketPtr->Destination;  // Second byte of UID
    uid[2] = (uint8_t)rxPacketPtr->Payload[0]; // Third byte of UID
    uid[3] = (uint8_t)rxPacketPtr->Payload[1]; // Fourth byte of UID

    // 2. Assign network ID based on the extracted UID
    assignedID = assignIDToUID(uid);
    if (assignedID == 0xFF) {
        printf("ERROR: Too many nodes, cannot assign more IDs\n\r");
        return; // Exit if ID assignment failed
    }

    printf("Discovery request handled, assigned ID = %d\n\r", assignedID);

    // 3. Prepare response packet with assigned ID
    txPacket.TransmissionType = ID_ASSIGNMENT;  // Set packet type
    txPacket.ID = uid[0];                      // Echo back first UID byte
    txPacket.Destination = uid[1];             // Echo back second UID byte
    txPacket.Payload[0] = uid[2];              // Echo back third UID byte
    txPacket.Payload[1] = uid[3];              // Echo back fourth UID byte
    txPacket.Payload[2] = 0;                   // Reserved/unused
    txPacket.Payload[3] = assignedID;          // The newly assigned ID

    // 4. Transmit the ID assignment response
    CreateLPAWURFrameV2();  // Format packet for LPAWUR transmission
    HAL_Delay(5);           // Small delay to ensure clean transmission
    MX_APPE_Process();      // Execute RF transmission
}

/**
 * Generate comprehensive data collection report
 * Reports all expected nodes, response rates, and missing/timeout nodes
 */
void SendToDataBase(void) {
    if (SendDatabase == 0) {  // Only send report once per collection cycle
        uint32_t timestamp = HAL_GetTick();
        uint8_t missingNodes = 0;

        printf("=== DATA COLLECTION REPORT ===\n\r");
        printf("Timestamp: %lu ms\n\r", timestamp);

        // Report data for all expected nodes in the network
        for (int i = 0; i < IDListSize; i++) {
            uint8_t nodeID = IDList[i];

            // Check if we received data from this node
            if (nodeID < MAX_NODES && receivedData[nodeID].received) {
                printf("Node %02X: Temp=%d°C, Humid=%d%%\n\r",
                       nodeID, receivedData[nodeID].temp, receivedData[nodeID].humid);
            } else {
                printf("Node %02X: MISSING/TIMEOUT\n\r", nodeID);
                missingNodes++;
            }
        }

        // Calculate and report collection statistics
        float responseRate = (float)(receivedDataCount) / IDListSize * 100.0f;
        printf("Response rate: %.1f%% (%d/%d)\n\r", responseRate, receivedDataCount, IDListSize);

        // Warn about missing responses
        if (missingNodes > 0) {
            printf("WARNING: %d nodes did not respond\n\r", missingNodes);
        }

        printf("=== END REPORT ===\n\r\n\r");
        SendDatabase = 1;  // Mark report as sent
    }
}

/**
 * State machine for managing data collection cycles
 * Handles timeout detection, retry logic, and collection completion
 */
void HandleDataCollection(void) {
    uint32_t currentTime = HAL_GetTick();

    // Force completion after maximum retries
    if (retryCount == 4) collectionPhase = 2;

    switch(collectionPhase) {
        case 0: // IDLE - Start new collection cycle
            if (IDListSize > 0) {  // Only start if we have nodes to collect from
                dataCollectionStartTime = currentTime;
                collectionPhase = 1;   // Move to collecting phase
                printf("RTC wakeup during data collection\n\r");
            }
            break;

        case 1: // COLLECTING - Monitor progress and handle timeouts
            // Check completion conditions: all responses received OR timeout reached
            if (receivedDataCount >= IDListSize ||
                (currentTime - dataCollectionStartTime) > DATA_COLLECTION_TIMEOUT) {

                // Handle timeout scenario with retry logic
                if (receivedDataCount < IDListSize && retryCount < MAX_RETRIES) {
                    printf("Timeout! Received %d/%d responses. Retry %d/%d\n\r",
                           receivedDataCount, IDListSize, retryCount + 1, MAX_RETRIES);

                    // Send targeted requests only to non-responding nodes
                    SendDataRequestToMissingNodes();
                    dataCollectionStartTime = currentTime; // Reset timeout for retry
                } else {
                    printf("All collected \n\r");
                    collectionPhase = 2;  // Collection complete
                }
            }
            break;

        case 2: // COMPLETE - Collection finished (handled elsewhere)
            // This state is processed in main loop to generate reports
            break;
    }
    retryCount++;  // Increment retry counter
}

/**
 * Send data requests only to nodes that haven't responded yet
 * This targeted approach reduces network traffic during retry attempts
 */
void SendDataRequestToMissingNodes(void) {
    for (int i = 0; i < IDListSize; i++) {
        uint8_t nodeID = IDList[i];

        // Send request only if we haven't received data from this node
        if (nodeID < MAX_NODES && !receivedData[nodeID].received) {
            // Prepare targeted data request packet
            txPacket.TransmissionType = DATAREQ;      // Data request packet type
            txPacket.ID = MAIN_NODE_ID;               // Source: main controller
            txPacket.Destination = nodeID;            // Target: specific missing node
            txPacket.Payload[2] = 0;                  // Reserved
            txPacket.Payload[3] = 5;                  // Request parameter

            // Transmit the targeted request
            CreateLPAWURFrameV2();
            HAL_Delay(10);  // Small delay between transmissions to avoid collisions
            MX_APPE_Process();
            printf("Retry data request sent to node %d\n\r", nodeID);
        }
    }
}

//----------------------------- Transmission Functions ------------------------------------
/**
 * Handle RTC wake-up triggered packet transmission
 * This function is called when the system wakes up from low power mode
 */
void SendPacket() {
    // Enable and check RTC wake-up line
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_RTC, PWR_WUP_RISIEDG);
    uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

    // If wake-up was triggered by RTC, send the prepared packet
    if (wakeupSource & PWR_WAKEUP_RTC) {
        CreateLPAWURFrameV2();  // Format packet for transmission
        HAL_Delay(100);         // Wait for stable transmission conditions
        MX_APPE_Process();      // Execute RF transmission
        printf("Packet sent \r\n");
    }
}

/**
 * Create LPAWUR (Low Power Asynchronous Wake-Up Radio) frame from packet data
 * This function formats the packet structure into the specific frame format
 * required by the LPAWUR protocol
 */
void CreateLPAWURFrameV2() {
    // Clear the first 5 bytes (preamble/sync pattern)
    for (int i = 0; i < 5; i++) vectcTxBuff[i] = 0x00;

    // Set frame start marker
    vectcTxBuff[5] = 0x99;  // Frame synchronization byte

    // Combine transmission type (upper nibble) with main node ID (lower nibble)
    vectcTxBuff[6] = (txPacket.TransmissionType << 4) | (MAIN_NODE_ID & 0x0F);

    // Pack packet data into transmission buffer
    vectcTxBuff[7]  = txPacket.ID;          // Source ID
    vectcTxBuff[8]  = txPacket.Destination; // Destination ID
    vectcTxBuff[9]  = txPacket.Payload[0];  // Payload byte 0
    vectcTxBuff[10] = txPacket.Payload[1];  // Payload byte 1
    vectcTxBuff[11] = txPacket.Payload[2];  // Payload byte 2
    vectcTxBuff[12] = txPacket.Payload[3];  // Payload byte 3

    // Calculate and append CRC for error detection
    EvaluateCrc(&vectcTxBuff[6]);
}

/**
 * Execute RF transmission using the MR-SubG radio
 * This function handles the low-level radio transmission process
 */
void MX_APPE_Process(void) {
    BSP_LED_On(LD3);  // Indicate transmission activity

    // Initiate transmission command
    __HAL_MRSUBG_STROBE_CMD(CMD_TX);

    // Wait for transmission completion
    while ((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS() & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {
        // Busy wait for TX completion interrupt
    }

    // Clear transmission done flag
    __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);

    BSP_LED_Off(LD3);          // Turn off transmission indicator
    LL_LPAWUR_SetState(ENABLE); // Re-enable LPAWUR receiver for next reception
}

//----------------------------- Reception Functions ------------------------------------
/**
 * Parse received LPAWUR payload into packet structure
 * @param LPAWUR_Pay: 8-byte raw payload from LPAWUR receiver
 * @param handle_packet: Pointer to packet structure to populate
 */
void PacketHandler(uint8_t LPAWUR_Pay[8], Packet* handle_packet) {
    handle_packet->TransmissionType = LPAWUR_Pay[0]; // Extract packet type
    handle_packet->ID = LPAWUR_Pay[1];               // Extract source ID
    handle_packet->Destination = LPAWUR_Pay[2];      // Extract destination ID
    handle_packet->Payload[0] = (float)LPAWUR_Pay[3]; // Convert payload bytes
    handle_packet->Payload[1] = (float)LPAWUR_Pay[4]; // to float format
    handle_packet->Payload[2] = LPAWUR_Pay[5];       // Keep as byte
    handle_packet->Payload[3] = LPAWUR_Pay[6];       // Keep as byte
}

/**
 * Main reception handler - processes incoming LPAWUR packets
 * This is the central function that handles all incoming network traffic
 * @param PR: Processing parameter (unused in current implementation)
 */
void GotoRx(void) {
    LL_LPAWUR_SetState(ENABLE);  // Enable LPAWUR receiver

    // Enable LPAWUR wake-up interrupt
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, 1);
    uint32_t wakeupSource2 = HAL_PWREx_GetClearInternalWakeUpLine();

    // Process packet if wake-up was triggered by LPAWUR reception
    if (wakeupSource2 & PWR_WAKEUP_LPAWUR) {
        BSP_LED_On(LD2);  // Indicate reception activity

        // Get received payload from LPAWUR hardware
        HAL_LPAWUR_GetPayload(LPAWUR_Payload);

        // Parse raw payload into packet structure
        PacketHandler(LPAWUR_Payload, &rxPacket);

        // Extract transmission type from upper nibble
        int transType = (rxPacket.TransmissionType >> 4) & 0x0F;

        // Process packet based on transmission type
        switch (transType) {
            case DISCOVERY_RESP:
                // Handle device discovery responses
                DiscoveryPhaseHandler(&rxPacket);
                break;

            case DATAREP:
                // Handle sensor data reports from network nodes

                // Check for unknown/unassigned nodes responding
                if (rxPacket.ID == UNASSIGNED_ID || rxPacket.ID > IDListSize) {
                    printf("Unknown NODE responding\n\r");

                    // Send discovery request to unknown node
                    txPacket.ID = MAIN_NODE_ID;
                    txPacket.TransmissionType = DISCOVERY_REQ;
                    txPacket.Payload[2] = 0;
                    txPacket.Payload[3] = 5;
                    CreateLPAWURFrameV2();
                    HAL_Delay(5);
                    MX_APPE_Process();
                    printf("Discovery request sent to unknown node\n\r");
                    break;
                } else {
                    // Add known node to active ID list
                    AddToIDList(rxPacket.ID);
                }

                // Ensure consistency between node database and ID list
                if (nodeCount != IDListSize) {
                    for (int c = 0; c < nodeCount; c++) {
                        if (knownNodes[c].assignedID == rxPacket.ID) {
                            AddToIDList(rxPacket.ID);
                        }
                    }
                }

                // Record timestamp of first data packet in collection cycle
                if (receivedDataCount == 0) {
                    firstResponseTime = HAL_GetTick();
                }

                // Process data if node is in active ID list
                if (IsInIDList(rxPacket.ID) == 1) {
                    if (receivedData[rxPacket.ID].received == 1) {
                        // Update existing data entry
                        printf("Updating..\n\r");
                        receivedData[rxPacket.ID].temp = rxPacket.Payload[0];
                        receivedData[rxPacket.ID].humid = rxPacket.Payload[1];
                    } else if (rxPacket.ID < MAX_NODES) {
                        // Create new data entry
                        receivedData[rxPacket.ID].id = rxPacket.ID;
                        receivedData[rxPacket.ID].temp = rxPacket.Payload[0];
                        receivedData[rxPacket.ID].humid = rxPacket.Payload[1];
                        receivedData[rxPacket.ID].received = 1;
                        receivedDataCount++;

                        printf("DATAREP from %x: Temp = %d°C, Humid = %d%%\n\r",
                               rxPacket.ID, rxPacket.Payload[0], rxPacket.Payload[1]);
                    }
                }
                break;

            case DATAREQ:
                // Handle data requests (currently not implemented for main controller)
                break;

            case DISCOVERY_REQ:
                // Handle discovery requests (nodes looking for network access)
                break;

            case ID_ASSIGNMENT:
                // Handle ID assignment responses (not typically received by main controller)
                break;

            case ID_RECEIVED:
                // Handle ID acknowledgment from newly assigned nodes
                if (IsInIDList(txPacket.ID) == 1) {
                    printf("Id already assigned : %d \n\r", txPacket.ID);
                } else {
                    AddToIDList(rxPacket.ID);
                }
                break;

            case ALERT:
                // Handle alert/emergency packets
                printf("High Temp Alert \r\n");  // Alert indication
                break;

            default:
                // Handle unknown packet types
                printf("Unknown transmission type: %d\n\r", rxPacket.TransmissionType);
                break;
        }

        // Clean up after packet processing
        HAL_LPAWUR_ClearStatus();    // Clear LPAWUR status flags
        LL_LPAWUR_SetState(ENABLE);  // Re-enable receiver for next packet
        HAL_Delay(5);                // Small delay for stability
        BSP_LED_Off(LD2);            // Turn off reception indicator
    }
}

//----------------------------- Power Management ------------------------------------
/* USER CODE END MX_APPE_Process_2 */
#if (CFG_LPM_SUPPORTED == 1)
/**
 * Determine the appropriate power save level for the application
 * @return: Power save level based on current application state
 */
static PowerSaveLevels App_PowerSaveLevel_Check(void)
{
    // Default to deepest power save mode when no timer operations needed
    PowerSaveLevels output_level = POWER_SAVE_LEVEL_DEEPSTOP_NOTIMER;
    /* USER CODE BEGIN App_PowerSaveLevel_Check_1 */
    /* USER CODE END App_PowerSaveLevel_Check_1 */
    return output_level;
}
#endif

/**
 * Check power save level for MR-SubG timer operations
 * @return: Power save level that allows timer functionality
 */
__weak PowerSaveLevels HAL_MRSUBG_TIMER_PowerSaveLevelCheck()
{
    return POWER_SAVE_LEVEL_DEEPSTOP_TIMER;
}

/**
 * Main idle/power management function
 * Handles low power mode transitions when system is not actively processing
 */
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
	if (print_stats == 1) printf("Current RSSI: %d dBm | MIN : %d dBm | MAX : %d dBm\r\n", rssi, rssi_min, rssi_max);
}
//--------------------------------------------------------------------------------------------------
