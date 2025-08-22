/*
 * transmission_mesh.c
 *
 * Mesh Network Implementation for IoT Sensor Nodes
 * This file implements a wireless mesh network protocol using LPAWUR (Low Power Always Wake Up Radio)
 * for communication between sensor nodes. The network supports automatic node discovery,
 * ID assignment, data collection, routing, and alert forwarding.
 *
 * Reworked for Mesh Network
 *  Created on: Jun 6, 2025
 *      Author: mzakri
 */
#include "transmission_mesh.h"

// Network configuration constants
#define PAYLOAD_LEN 7                    // Standard payload length for packets
#define MIN(a,b) (((a) < (b)) ? (a) : (b)) // Utility macro for minimum value

// Global transmission and reception buffers
uint8_t vectcTxBuff[15],LPAWUR_Payload[8];  // TX buffer and LPAWUR payload buffer

// Packet structures for transmission and reception
Packet txPacket,rxPacket;               // Current TX and RX packet structures

// Cache for duplicate packet detection
SimpleCache myCache ;                   // Initialize as empty - prevents packet loops

// Sensor data variables
float temp = 0;                         // Temperature reading from Si7021 sensor
float humid = 0;                        // Humidity reading from Si7021 sensor

// Node identification and network topology
uint8_t UID[4];                         // Unique identifier extracted from MCU
uint8_t myHop = 10;                     // Initial hop count (distance from main node)
uint8_t myID = UNASSIGNED_ID;           // Node's assigned ID (starts unassigned)

// Network state variables
uint8_t datareqSent,hopper ;            // Flags for data request state and current hopper

// Routing table for mesh network topology
RoutingEntry routingTable[MAX_ROUTES];  // Table storing known routes to other nodes

//----------------------------- Sensor Functions ------------------------------------

/**
 * @brief Initialize and read temperature and humidity from Si7021 sensor
 *
 * This function initializes the Si7021 I2C temperature and humidity sensor,
 * reads current values, and processes them for transmission. If sensor fails,
 * the red LED is turned on to indicate error.
 */
void TempANDHumidSensor() {
    // Keep trying to initialize sensor until successful
    while (Si7021_Init() != HAL_OK) BSP_LED_On(LED_RED);

    // Read temperature and humidity values
    if (Si7021_ReadTempAndHumidity(&temp, &humid) == HAL_OK) {
        printf("TEMP: %.2f°C, HUMID: %.2f \n\r", temp, humid);
        // Round values to integers for transmission (saves bandwidth)
        temp = round(temp);
        humid = round(humid);
    }
    else BSP_LED_On(LED_RED);  // Signal sensor error with LED
}

//----------------------------- Unique ID Management ------------------------------------

/**
 * @brief Extract unique identifier from MCU hardware registers
 * @param uid: Pointer to 4-byte array to store the UID
 *
 * This function reads the hardware UID registers and extracts a 4-byte
 * unique identifier used for node identification before ID assignment.
 */
void GETUID(uint8_t *uid) {
    // Read 32-bit UID registers from MCU
    uint32_t uid0 = LL_GetUID_Word0();
    uint32_t uid1 = LL_GetUID_Word1();

    // Extract 4 bytes from the 64-bit UID for compact identification
    uid[0] = (uid0 >> 24) & 0xFF;  // MSB of first register
    uid[1] = (uid0 >> 16) & 0xFF;  // Second byte
    uid[2] = (uid1 >> 8) & 0xFF;   // Third byte
    uid[3] = (uid1 >> 0) & 0xFF;   // LSB of second register

    printf("UID = %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
}

//----------------------------- Packet Handling Functions ------------------------------------

/**
 * @brief Check if received packet originated from this node
 * @param pkt: Pointer to packet to check
 * @return: 1 if packet is from self, 0 otherwise
 *
 * Prevents forwarding of own packets by checking both assigned ID
 * and hardware UID patterns.
 */
uint8_t isSelfPacket(Packet *pkt) {
    return (pkt->ID == myID || (pkt->ID == UID[0] && (uint8_t)pkt->Payload[0] == UID[1] &&
            (uint8_t)pkt->Payload[1] == UID[2] && pkt->Payload[2] == UID[3]) );
}

/**
 * @brief Initialize the routing table with default values
 *
 * Sets all routing table entries to unassigned state with maximum hop counts.
 * Called during node startup or network reset.
 */
void InitRoutingTable(void) {
    for (int i = 0; i < MAX_ROUTES; i++) {
        routingTable[i].nodeID = UNASSIGNED_ID;  // No node assigned
        routingTable[i].nextHop = 0;             // No next hop
        routingTable[i].hopCount = 255;          // Max hop means undefined/unreachable
        routingTable[i].lastSeen = 0;            // Never seen
    }
}

/**
 * @brief Update routing table with new node information
 * @param nodeID: ID of the node to add/update
 * @param nextHop: Next hop node to reach this destination
 * @param hopCount: Number of hops to reach the destination
 *
 * This function maintains a routing table for the mesh network, only accepting
 * direct neighbors (hopCount <= 1) to prevent routing loops and ensure reliability.
 */
void UpdateRoutingTable(uint8_t nodeID, uint8_t nextHop, uint8_t hopCount) {
    // Security check: Accept only direct neighbors to prevent routing attacks
    if (hopCount > 1 || nodeID == myID)  {
        return; // Ignore non-direct nodes and self-references
    }

    // Check if the node already exists in routing table
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routingTable[i].nodeID == nodeID) {
            // Update entry if we found a better route or same next hop
            if (hopCount <= routingTable[i].hopCount || routingTable[i].nextHop == nextHop) {
                routingTable[i].nextHop = nextHop;
                routingTable[i].hopCount = hopCount;
            }
            return;
        }
    }

    // Find empty slot for new entry
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routingTable[i].nodeID == UNASSIGNED_ID) {
            routingTable[i].nodeID = nodeID;
            routingTable[i].nextHop = nextHop;
            routingTable[i].hopCount = hopCount;
            return;
        }
    }

    // Table is full - could implement LRU eviction here
    printf("Routing table full. Could not add node %d\n", nodeID);
}

/**
 * @brief Print current routing table contents for debugging
 *
 * Displays all active routes in the routing table for network topology visualization.
 */
void printRoutingTable(void) {
    printf("=== Routing Table ===\r\n");

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routingTable[i].nodeID != UNASSIGNED_ID) {
            printf("Slot %d: NodeID: %u | NextHop: %u | Hops: %u\r\n",
                   i,
                   routingTable[i].nodeID,
                   routingTable[i].nextHop,
                   routingTable[i].hopCount);
        }
    }

    printf("======================\r\n");
}

/**
 * @brief Determine if a packet should be forwarded
 * @param pkt: Pointer to packet to evaluate
 * @return: 1 if packet should be forwarded, 0 otherwise
 *
 * Checks packet validity and TTL before allowing forwarding to prevent loops.
 */
uint8_t shouldForward(Packet *pkt) {
    // Drop packet if it originated from this node
    if (isSelfPacket(pkt)) {
        return 0;
    }

    // Check Time-To-Live to prevent infinite forwarding
    uint8_t ttl = pkt->Payload[3];
    if (ttl < 1) {
        return 0; // TTL expired, drop packet
    }
    return 1;
}

/**
 * @brief Forward a received packet to other nodes in the mesh
 * @param type: Transmission type for the forwarded packet
 *
 * Creates a forwarded packet by copying received packet, updating hop count,
 * decreasing TTL, and retransmitting with appropriate delays.
 */
void forwardPacket(uint8_t type) {
    // Copy received packet as base for forwarding
    memcpy(&txPacket, &rxPacket, sizeof(Packet));

    printf("REBROADCAST...\r\n");

    // Set transmission type for forwarded packet
    txPacket.TransmissionType = type;

    // Update destination field (except for special packet types)
    if (type != ID_ASSIGNMENT && type != DISCOVERY_RESP) {
        txPacket.Destination = myID;  // Mark this node as the forwarder
    }

    txPacket.Payload[2] += 1;  // Increment hop count for distance tracking

    // Decrement TTL for most packet types (ID_ASSIGNMENT has no TTL limit)
    if (type != ID_ASSIGNMENT) {
        txPacket.Payload[3] -= 1;  // Decrease Time-To-Live
    }

    SendPacket(vectcTxBuff);
}

/**
 * @brief Check if received packet is a duplicate
 * @param pkt: Pointer to packet to check
 * @return: 1 if duplicate, 0 if new packet
 *
 * Simple cache-based duplicate detection to prevent packet storms.
 * Cache expires after 5 minutes to allow for network changes.
 */
uint8_t isDuplicate(Packet *pkt) {
    // Expire cache after 5 minutes to allow network topology changes
    if ((HAL_GetTick() - myCache.lastRxTime) > (5 * 60 * 1000)) {
        myCache.lastSenderID = 0; // Force accept next packet
    }

    // Main node always processes packets (no duplicate filtering)
    if (hopper == MAIN_NODE_ID) return 0;

    // Check if same sender sent same packet type recently
    if (myCache.lastSenderID == pkt->ID && myCache.lastTransType == pkt->TransmissionType) {
        return 1; // Duplicate detected
    }

    // Update cache with current packet info
    myCache.lastSenderID = pkt->ID;
    myCache.lastTransType = pkt->TransmissionType;
    myCache.lastRxTime = HAL_GetTick();
    return 0; // New packet
}

/**
 * @brief Reset the duplicate detection cache
 *
 * Called when starting new discovery phases, after inactivity,
 * or on network reset to ensure fresh packet processing.
 */
void ResetCache(void) {
    myCache.lastSenderID = 0;
    myCache.lastTransType = 0xFF; // Invalid transmission type
    myCache.lastRxTime = 0;
}

/**
 * @brief Prepare and send discovery response packet
 * @param Pack: Pointer to packet structure to prepare
 *
 * Sends node's UID back to main node during discovery phase,
 * maintaining hop count from original discovery request.
 */
void PrepareDiscoveryResponse(Packet *Pack) {
    Pack->TransmissionType = DISCOVERY_RESP;
    Pack->ID = UID[0];              // First byte of UID as temporary ID
    Pack->Destination = UID[1];     // Second byte as destination
    Pack->Payload[0] = UID[2];      // Third byte in payload
    Pack->Payload[1] = UID[3];      // Fourth byte in payload
    Pack->Payload[2] = rxPacket.Payload[2]; // Maintain hop count from discovery
    Pack->Payload[3] = 5;           // Set TTL
    SendPacket(vectcTxBuff);
}

/**
 * @brief Send acknowledgment that node is alive and has received ID
 * @param Pack: Pointer to packet structure to prepare
 *
 * Confirms to main node that this node is active and has valid ID assignment.
 */
void SendAckALIVE(Packet *Pack) {
    Pack->TransmissionType = ID_RECEIVED;
    Pack->ID = myID;                // Use assigned ID
    Pack->Destination = MAIN_NODE_ID; // Send directly to main node
    Pack->Payload[0] = 0;           // No additional data
    Pack->Payload[1] = 0;
    Pack->Payload[2] = 0;           // No hop count needed for direct ack
    Pack->Payload[3] = 5;           // Set TTL
    SendPacket(vectcTxBuff);
}

//----------------------------- Transmission Functions ------------------------------------

/**
 * @brief Generate randomized delay before packet transmission
 *
 * Implements collision avoidance by adding random delays based on hop distance.
 * Nodes closer to main node (lower hop count) get shorter delays to reduce latency.
 * This prevents all nodes from transmitting simultaneously and causing collisions.
 */
void SimpleRand16(void)
{
    // Get random 16-bit value from hardware RNG
    uint16_t val = LL_RNG_ReadRandData16(RNG);

    // Scale delay based on hop distance from main node
    switch (myHop) {
        case 0: // Direct connection to main node
            val /= 32;
            if(val < 200) val +=200;    // Minimum 200ms delay
            break;
        case 1: // One hop from main
            val /= 16;
            if(val < 2000) val +=2000;  // Minimum 2s delay
            break;
        case 2: // Two hops from main
            val /= 8;
            if(val < 4000) val +=4000;  // Minimum 4s delay
            break;
        case 3: // Three hops from main
            val /= 6;
            if(val < 8000) val +=8000;  // Minimum 8s delay
            break;
        case 4: // Four hops from main
            val /= 4;
            if(val < 1000) val +=10000; // Minimum 10s delay
            break;
        default: // Distant nodes or unknown hop count
            val /= 20;
            if(val < 1600) val +=1600;  // Base delay
            if (val > 2000) val = 2000+ LL_RNG_ReadRandData16(RNG)/32; // Cap with randomness
            break;
    }
    printf("Random Delay: %d\r\n", val);
    HAL_Delay(val); // Apply the calculated delay
}

/**
 * @brief Send prepared packet over LPAWUR radio
 * @param vectcTxBuff: Transmission buffer containing formatted packet
 *
 * Main transmission function that formats packet, applies random delay,
 * and triggers the radio transmission process.
 */
void SendPacket(uint8_t* vectcTxBuff) {
    CreateLPAWURFrameV2(vectcTxBuff);  // Format packet for LPAWUR protocol
    SimpleRand16();                     // Apply collision avoidance delay
    MX_APPE_Process();                  // Execute radio transmission
}

/**
 * @brief Format packet data into LPAWUR frame structure
 * @param vectcTxBuff: Buffer to store formatted frame
 *
 * Converts internal packet structure to LPAWUR protocol format with
 * proper headers, addressing, and CRC calculation.
 */
void CreateLPAWURFrameV2(uint8_t* vectcTxBuff) {
    // Clear first 5 bytes (preamble/sync)
    for (int i = 0; i < 5; i++) vectcTxBuff[i] = 0x00;
    vectcTxBuff[5] = 0x99; // LPAWUR sync word

    // Pack transmission type and source ID into first data byte
    vectcTxBuff[6]  = (txPacket.TransmissionType << 4) | (myID & 0x0F);

    // Set source ID field based on packet type
    if (txPacket.TransmissionType == DISCOVERY_RESP || txPacket.TransmissionType == ID_ASSIGNMENT ||
        txPacket.TransmissionType == DATAREP) {
        vectcTxBuff[7] = txPacket.ID; // Use packet's ID field
    }
    else {
        vectcTxBuff[7]  = (myID != UNASSIGNED_ID) ? myID : txPacket.ID; // Use assigned ID if available
    }

    // Pack remaining packet fields
    vectcTxBuff[8]  = txPacket.Destination;  // Destination node
    vectcTxBuff[9]  = txPacket.Payload[0];   // Payload byte 0 (often temperature)
    vectcTxBuff[10] = txPacket.Payload[1];   // Payload byte 1 (often humidity)
    vectcTxBuff[11] = txPacket.Payload[2];   // Payload byte 2 (often hop count)
    vectcTxBuff[12] = txPacket.Payload[3];   // Payload byte 3 (often TTL)

    EvaluateCrc(&vectcTxBuff[6]); // Calculate and append CRC for error detection
}

/**
 * @brief Execute radio transmission at hardware level
 *
 * Triggers the MRSUBG radio to transmit the prepared frame and waits
 * for transmission completion before returning.
 */
void MX_APPE_Process(void) {
    // Uncomment to indicate transmission with LED
    //BSP_LED_On(LD3);

    // Trigger transmission command
    __HAL_MRSUBG_STROBE_CMD(CMD_TX);

    // Wait for transmission completion
    while((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS() & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {}

    // Clear transmission done flag
    __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);

    // Turn off transmission indicator LED
    //BSP_LED_Off(LD3);
}

//----------------------------- Reception Functions ------------------------------------

/**
 * @brief Parse received LPAWUR payload into packet structure
 * @param LPAWUR_Pay: Raw 8-byte payload from LPAWUR radio
 * @param pkt: Pointer to packet structure to populate
 *
 * Converts raw LPAWUR frame data back into internal packet structure
 * for processing by the mesh network protocol.
 */
void PacketHandler(uint8_t LPAWUR_Pay[8], Packet* pkt) {
    pkt->TransmissionType = LPAWUR_Pay[0];      // Packet type and source info
    pkt->ID = LPAWUR_Pay[1];                    // Source node ID
    pkt->Destination = LPAWUR_Pay[2];           // Destination node ID
    pkt->Payload[0] = (float)LPAWUR_Pay[3];     // First payload byte (temperature)
    pkt->Payload[1] = (float)LPAWUR_Pay[4];     // Second payload byte (humidity)
    pkt->Payload[2] = LPAWUR_Pay[5];            // Third payload byte (hop count)
    pkt->Payload[3] = LPAWUR_Pay[6];            // Fourth payload byte (TTL)
}

/**
 * @brief Main packet reception and processing function
 * @param vectcTxBuff: Transmission buffer for response packets
 *
 * This is the core function that handles all received packets. It:
 * 1. Checks for LPAWUR wake-up events
 * 2. Parses received packets
 * 3. Updates routing information
 * 4. Processes different packet types according to mesh protocol
 * 5. Forwards packets when appropriate
 * 6. Generates responses (sensor data, acknowledgments, etc.)
 */
void GotoRx(uint8_t* vectcTxBuff) {
    // Enable LPAWUR wake-up interrupt for packet reception
    HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
    uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

    if (wakeupSource & PWR_WAKEUP_LPAWUR) {
        // Uncomment to indicate reception with LED
        //BSP_LED_On(LD2);

        // Get payload from LPAWUR radio and parse it
        HAL_LPAWUR_GetPayload(LPAWUR_Payload);
        PacketHandler(LPAWUR_Payload, &rxPacket);

        // Extract transmission type from packet header
        uint8_t transType = (rxPacket.TransmissionType >> 4) & 0x0F;

        // Identify the hopping node (except for ID assignment packets)
        if (transType != ID_ASSIGNMENT) hopper = rxPacket.ID;
        if (hopper != 0) printf("Received from a hopper ID %d \r\n",hopper);

        // Update routing table with learned topology (except for certain packet types)
        if (transType != ID_ASSIGNMENT  && transType != ALERT  && transType != DISCOVERY_RESP){
            UpdateRoutingTable(hopper, rxPacket.Destination, rxPacket.Payload[2]);
            printRoutingTable();
        }

        // Update hop count if we receive packet directly from main node
        if (myHop > 6 && rxPacket.ID == MAIN_NODE_ID) myHop = rxPacket.Payload[2];

        // Process packet only if it's not a duplicate
        if (isDuplicate(&rxPacket) == 0){

            switch (transType) {
                case DISCOVERY_REQ:
                    printf("DISCOVERY_REQ received\n\r");
                    ResetCache(); // Clear cache for new discovery phase

                    // Forward discovery request if TTL allows and we haven't sent data request yet
                    if (rxPacket.Payload[3] > 2 && shouldForward(&rxPacket)  && datareqSent == 0) {
                        myHop = (uint8_t)rxPacket.Payload[2]; // Update our hop count
                        datareqSent = 1; // Mark that we've forwarded discovery
                        printf("Im this amount of hop to main : %d \n\r",myHop);
                        forwardPacket(DISCOVERY_REQ);
                    }

                    // Respond based on our current state
                    if (myID == UNASSIGNED_ID) {
                        PrepareDiscoveryResponse(&txPacket); // Send UID for ID assignment
                    }
                    else {
                        SendAckALIVE(&txPacket); // Acknowledge we're still alive
                    }
                break;

                case DISCOVERY_RESP:
                    // Forward discovery responses toward main node
                    if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                        forwardPacket(DISCOVERY_RESP);
                    }
                break;

                case DATAREQ:
                    ResetCache(); // Clear cache for new data collection phase

                    // Forward data request to other nodes
                    if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                        forwardPacket(DATAREQ);
                    }

                    // If we don't have ID yet, request one
                    if (myID == UNASSIGNED_ID){
                        PrepareDiscoveryResponse(&txPacket);
                        break;
                    }

                    // Collect sensor data and send response
                    HAL_Delay(10); // Brief delay before sensor reading
                    TempANDHumidSensor(); // Read current sensor values

                    // Prepare sensor data response
                    txPacket.TransmissionType = DATAREP;
                    txPacket.ID = myID;
                    txPacket.Destination = myID; // For hop routing cases
                    txPacket.Payload[0] = temp;   // Temperature reading
                    txPacket.Payload[1] = humid;  // Humidity reading
                    txPacket.Payload[2] = 0;      // Reset hop count
                    txPacket.Payload[3] = 5;      // Set TTL
                    SendPacket(vectcTxBuff);
                break;

                case DATAREP:
                    // Process and forward sensor data reports
                    if (rxPacket.Payload[3] > 0 && shouldForward(&rxPacket)) {
                        printf("DataREP received \r\n");
                        TempANDHumidSensor(); // Read our own sensor for comparison

                        // Check for temperature anomaly (>10°C difference)
                        if (abs(temp - rxPacket.Payload[0]) > 10) {
                            forwardPacket(ALERT); // Send alert if significant difference
                        }
                        else {
                            forwardPacket(DATAREP); // Normal forwarding
                        }
                    }
                break;

                case ID_ASSIGNMENT:
                    printf("ID_ASS received \r\n");

                    // Check if this ID assignment is for us (match full UID)
                    if (rxPacket.ID == UID[0] && rxPacket.Destination == UID[1] &&
                        rxPacket.Payload[0] == UID[2] && rxPacket.Payload[1] == UID[3]) {

                        if (myID == UNASSIGNED_ID){
                            myID = rxPacket.Payload[3]; // Accept assigned ID
                            printf("ID assigned: %d\n\r", myID);
                            SendAckALIVE(&txPacket); // Acknowledge assignment
                        }
                    }
                    // Forward ID assignment if not for us
                    else if (shouldForward(&rxPacket)) {
                        forwardPacket(ID_ASSIGNMENT);
                    }
                break;

                case ALERT :
                    // Immediately retransmit alert messages (high priority)
                    memcpy(&txPacket, &rxPacket, sizeof(Packet));
                    SendPacket(vectcTxBuff);
                break;

                default:
                    // Unknown packet type - ignore
                break;
            }
        }

        // Clean up LPAWUR state for next reception
        HAL_LPAWUR_ClearStatus();
        LL_LPAWUR_SetState(ENABLE);

        // Turn off reception indicator LED
        //BSP_LED_Off(LD2);
    }
}

//----------------------------- Power Management Functions ------------------------------------

/**
 * @brief Check what power save level is appropriate for current application state
 * @return: Recommended power save level
 *
 * Determines the deepest sleep mode that can be used based on application requirements.
 */
#if (CFG_LPM_SUPPORTED == 1)
static PowerSaveLevels App_PowerSaveLevel_Check(void)
{
    PowerSaveLevels output_level = POWER_SAVE_LEVEL_DEEPSTOP_NOTIMER;
    /* USER CODE BEGIN App_PowerSaveLevel_Check_1 */
    /* USER CODE END App_PowerSaveLevel_Check_1 */
    return output_level;
}
#endif

/**
 * @brief Weak function to check timer-related power save constraints
 * @return: Power save level allowed by timer subsystem
 */
__weak PowerSaveLevels HAL_MRSUBG_TIMER_PowerSaveLevelCheck()
{
    return POWER_SAVE_LEVEL_DEEPSTOP_TIMER;
}

/**
 * @brief Enter appropriate low power mode when system is idle
 *
 * This function is called when the system has no active tasks and can enter
 * sleep mode. It coordinates between application and timer constraints to
 * select the deepest appropriate sleep level.
 */
void MX_APPE_Idle(void)
{
#if (CFG_LPM_SUPPORTED == 1)
    PowerSaveLevels app_powerSave_level, vtimer_powerSave_level, final_level;

    // Check power save constraints from application
    app_powerSave_level = App_PowerSaveLevel_Check();

    if(app_powerSave_level != POWER_SAVE_LEVEL_DISABLED)
    {
        // Check power save constraints from timer subsystem
        vtimer_powerSave_level = HAL_MRSUBG_TIMER_PowerSaveLevelCheck();

        // Use the most restrictive (minimum) power save level
        final_level = (PowerSaveLevels)MIN(vtimer_powerSave_level, app_powerSave_level);

        switch(final_level)
        {
        case POWER_SAVE_LEVEL_DISABLED:
            /* Device is busy, no power saving */
            return;
            break;
        case POWER_SAVE_LEVEL_SLEEP:
            /* Light sleep - CPU off, peripherals active */
            UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
            UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
            break;
        case POWER_SAVE_LEVEL_DEEPSTOP_TIMER:
            /* Deep sleep with timer wake-up capability */
            UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
            UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
            break;
        case POWER_SAVE_LEVEL_DEEPSTOP_NOTIMER:
            /* Deepest sleep - only external events can wake up */
            UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
            UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
            break;
        }

        // Enter the selected low power mode
        UTIL_LPM_EnterLowPower();
    }
#endif /* CFG_LPM_SUPPORTED */
}

//---------------------------------------------------------------------------------------------------

/*
 * MESH NETWORK PROTOCOL SUMMARY:
 *
 * This implementation creates a self-organizing wireless sensor network with the following features:
 *
 * 1. NETWORK TOPOLOGY:
 *    - Star-mesh hybrid with one main node (coordinator)
 *    - Sensor nodes auto-discover and form multi-hop paths
 *    - Dynamic routing table maintains direct neighbor information
 *
 * 2. PACKET TYPES:
 *    - DISCOVERY_REQ: Main node broadcasts to find all sensors
 *    - DISCOVERY_RESP: Sensors respond with their unique identifiers
 *    - ID_ASSIGNMENT: Main node assigns short IDs to discovered sensors
 *    - DATAREQ: Main node requests sensor readings
 *    - DATAREP: Sensors respond with temperature/humidity data
 *    - ALERT: High-priority messages for anomaly detection
 *    - ID_RECEIVED: Acknowledgment of successful ID assignment
 *
 * 3. KEY FEATURES:
 *    - Collision avoidance through randomized delays
 *    - Duplicate packet detection to prevent loops
 *    - TTL (Time-To-Live) mechanism for hop limiting
 *    - Automatic sensor anomaly detection (>10°C temperature difference)
 *    - Power-optimized operation with deep sleep modes
 *    - Hardware-based unique identification using MCU UID
 *
 * 4. NETWORK OPERATION FLOW:
 *    a) Main node sends DISCOVERY_REQ
 *    b) Unassigned sensors respond with DISCOVERY_RESP containing UID
 *    c) Main node sends ID_ASSIGNMENT packets with short IDs
 *    d) Assigned sensors acknowledge with ID_RECEIVED
 *    e) Main node periodically sends DATAREQ for sensor readings
 *    f) Sensors respond with DATAREP containing temperature/humidity
 *    g) Nodes compare readings and generate ALERT if anomalies detected
 *
 * 5. RELIABILITY MECHANISMS:
 *    - Packet forwarding with hop count tracking
 *    - Duplicate detection cache with timeout
 *    - Routing table for topology awareness
 *    - CRC error detection in radio frames
 *    - LED indicators for error conditions
 *
 * This mesh network is designed for IoT sensor applications where:
 * - Low power operation is critical
 * - Nodes may be distributed over wide areas
 * - Automatic network formation is required
 * - Environmental monitoring with anomaly detection is needed
 * - Reliable data collection from multiple sensors is important
 */
