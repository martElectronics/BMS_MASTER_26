#include "MART_CAN.h"

/**
 * Reads a message from the CAN bus if available.
 * It checks for the availability of a message using the interrupt pin.
 * If a message is available, it reads the message ID, length, and data bytes
 * into the DataIN structure. It also determines whether the message uses an
 * extended ID and if it is a Remote Request Frame (RRF).
 * return true if a message was successfully read, false otherwise.
 */
bool CAN_BUS::readBytes()
{
    bool ok = false;
    if (type == HardwareType::Transciever)
    {
#if defined(ESP32) || defined(ESP32S3)

        CanFrame frame = {0};
        if (ESP32Can.readFrame(frame) && !config.simulating)
        {
            DataIN.dataRaw.id = frame.identifier;
            DataIN.dataRaw.size = frame.data_length_code;
            for (int i = 0; i < 8; i++)
            {
                DataIN.dataRaw.bytes[i] = frame.data[i];
            }
            DataIN.dataRaw.typeExtendedId = frame.extd;

            ok = true;
        }
#elif defined(STM32G4xx)
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) > 0 && !config.simulating)
        {
            if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &RxHeader, DataIN.dataRaw.bytes) == HAL_OK)
            {
                DataIN.dataRaw.id = RxHeader.Identifier;
                
                // Map DLC to size
                switch (RxHeader.DataLength) {
                    case FDCAN_DLC_BYTES_0: DataIN.dataRaw.size = 0; break;
                    case FDCAN_DLC_BYTES_1: DataIN.dataRaw.size = 1; break;
                    case FDCAN_DLC_BYTES_2: DataIN.dataRaw.size = 2; break;
                    case FDCAN_DLC_BYTES_3: DataIN.dataRaw.size = 3; break;
                    case FDCAN_DLC_BYTES_4: DataIN.dataRaw.size = 4; break;
                    case FDCAN_DLC_BYTES_5: DataIN.dataRaw.size = 5; break;
                    case FDCAN_DLC_BYTES_6: DataIN.dataRaw.size = 6; break;
                    case FDCAN_DLC_BYTES_7: DataIN.dataRaw.size = 7; break;
                    default: DataIN.dataRaw.size = 8; break;
                }
                
                DataIN.dataRaw.typeExtendedId = (RxHeader.IdType == FDCAN_EXTENDED_ID);
                ok = true;
            }
        }
#endif
    }
    return (ok || config.simulating);
}

/**
 * Sends all packets in the DataOUT structure to the CAN bus.
 * Iterates over each packet in DataOUT and sends them individually.
 * If a message fails to send, it logs an error but continues sending
 * the remaining messages. The success status is updated accordingly.
 * return true if all messages were sent successfully, false if any failed.
 */
bool CAN_BUS::send()
{
    bool success = true;

    unsigned long currentTime = millis();

    // Send status data if the timer reaches PT and the ESP is configured accordingly
    if (((millis() - previousStatusIntervalTime) >= (intervalTime / 3)) && (config.sendStatusData))
    {
        numCurrentSamples++;
        setCANStatusData();
        previousStatusIntervalTime = millis();

        if (numCurrentSamples > 3)
        {
            // numRXPaqOK = 0;
            // numTXPaqOK = 0;
            // numTxPaqError = 0;
            // numCurrentSamples = 1;
        }
    }

    DataOUT.forEachPacket([this, &success, currentTime](CanPacketRawData &packet)
                          {
        // Initially assume the packet does not have a timer and is ready to send
        bool readyToSend = true;

        // Check for a timer associated with this packet
        for (const auto& timer : packetTimers) {
            if (timer.packetID == packet.id) {
                // Check if the current time is past the next scheduled send time
                if (currentTime < packet.nextSendTime) {
                    readyToSend = false; // Not yet time to send this packet
                }
                break; // Timer found, no need to check further
            }
        }

       
         
        if (readyToSend) {

            //Store the packet ID before the possible ID change if the packet is a RRF
            unsigned long idAux=packet.id;
            byte buf[8];
            // Copy data to buffer
            
            memcpy(buf, packet.bytes, packet.size);

            if(type == HardwareType::Transciever){
#if defined(ESP32) || defined(ESP32S3)
                
                CanFrame frame = {0};
                frame.identifier = packet.id;
                frame.extd = packet.typeExtendedId;
                frame.data_length_code = packet.size;
                for (int i = 0; i < 8; i++)
                {
                    frame.data[i] = packet.bytes[i];
                }
                if (!ESP32Can.writeFrame(frame,0))
                {
                    ERROR_PRINTLN("Error sending message");
                    success = false; // Mark failure but continue sending the rest
                    numTxPaqError++;
                }
                else
                {
                    DEBUG_PRINTLN((String)"Packet sent ID = " + packet.id);
                    packet.id=idAux;
                    // Update the next send time for this packet if it has a timer
                    for (auto& timer : packetTimers) {
                        if (timer.packetID == packet.id) {
                            packet.nextSendTime = currentTime + timer.interval;
                            break;
                        }
                    }
                    numTXPaqOK++;
                }
#elif defined(STM32G4xx)
                TxHeader.Identifier = packet.id;
                TxHeader.IdType = packet.typeExtendedId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
                TxHeader.TxFrameType = FDCAN_DATA_FRAME;
                
                switch(packet.size) {
                    case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
                    case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
                    case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
                    case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
                    case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
                    case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
                    case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
                    case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
                    default: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
                }
                
                TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
                TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
                TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
                TxHeader.MessageMarker = 0;

                if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, packet.bytes) != HAL_OK)
                {
                    ERROR_PRINTLN("Error sending message");
                    success = false;
                    numTxPaqError++;
                }
                else
                {
                    DEBUG_PRINTLN((String)"Packet sent ID = " + packet.id);
                    packet.id=idAux;
                    for (auto& timer : packetTimers) {
                        if (timer.packetID == packet.id) {
                            packet.nextSendTime = currentTime + timer.interval;
                            break;
                        }
                    }
                    numTXPaqOK++;
                }
#endif
            }
            }
           }); // Ensure this closing brace matches the lambda function

    runtimeTime = millis() - previousStatusRuntimeTime;

    return success;
}

/**
 * Looks for an specific packet in DataOUT and sends it if exists
 * If a message fails to send, it logs an error but continues sending
 * the remaining messages. The success status is updated accordingly.
 * @return true if all messages were sent successfully, false if any failed.
 */
bool CAN_BUS::send(unsigned long id)
{
    bool success = true;

    CanPacketRawData packet;
    DEBUG_PRINT((String) "Sending packet with id " + id);
    if (DataOUT.getPacketById(id, packet))
    {
        byte len = packet.size;
        byte buf[8];

        // Copy data to buffer
        memcpy(buf, packet.bytes, len);
        if (type == HardwareType::Transciever)
        {
#if defined(ESP32) || defined(ESP32S3)

            CanFrame frame = {0};
            frame.identifier = packet.id;
            frame.extd = packet.typeExtendedId;
            frame.data_length_code = packet.size;
            for (int i = 0; i < 8; i++)
            {
                frame.data[i] = packet.bytes[i];
            }
            if (!ESP32Can.writeFrame(frame))
            {
                ERROR_PRINTLN("Error sending message");
                success = false; // Mark failure but continue sending the rest
            }
#elif defined(STM32G4xx)
            TxHeader.Identifier = packet.id;
            TxHeader.IdType = packet.typeExtendedId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
            TxHeader.TxFrameType = FDCAN_DATA_FRAME;
            
            switch(packet.size) {
                case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
                case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
                case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
                case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
                case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
                case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
                case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
                case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
                default: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
            }
            
            TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
            TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
            TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            TxHeader.MessageMarker = 0;

            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, buf) != HAL_OK)
            {
                ERROR_PRINTLN("Error sending message");
                success = false;
            }
#endif
        }
    }
    else
    {
        DEBUG_PRINTLN("NULLPTR");
        success = false;
    }

    return success;
}

/**
 * Receives messages from the CAN bus and stores them in DataIN.
 * This method repeatedly calls readBytes() to read any available CAN messages.
 * Each read message is added to the DataIN structure for later processing.
 */
void CAN_BUS::receive()
{
    previousStatusRuntimeTime = millis();
    int after;
    if (readBytes() || config.simulating)
    {
        int mills = millis();
        DEBUG_PRINTLN((String) "Time to read packet: " + (mills - previousStatusRuntimeTime));
        // Store packet in memory if is not in the IDs set by the filter or if are no ids stored
        if ((filterIDs.empty()) || (std::binary_search(filterIDs.begin(), filterIDs.end(), DataIN.dataRaw.id)))
        {
            // Serial.println("ADDED");
            DataIN.addPacket(DataIN.dataRaw);
        }
        after = millis();
        DEBUG_PRINTLN((String) "Time to add packet: " + (after - mills));
        mills = after;
        DEBUG_PRINTLN((String) "Rx ID: " + DataIN.dataRaw.id);
        
        numRXPaqOK++;
    }
}

// Calculates and writes the masks and filters to the MCP2515 registers given a set of IDs
bool CAN_BUS::setFilters(const unsigned long ids[], unsigned size)
{
    for (int i = 0; i < size; i++)
    {
        filterIDs.push_back(ids[i]);
    }
    std::sort(filterIDs.begin(), filterIDs.end());
    return true;
}

void CAN_BUS::printFilters()
{
    // configurator.printCalculatedValues();
}

void CAN_BUS::testFilters(const std::vector<uint16_t> &testIds)
{
    // configurator.testFilters(testIds);
}

void CAN_BUS::setPacketTimer(unsigned long packetID, unsigned long time)
{
    // Check if the timer already exists and update it
    for (auto &timer : packetTimers)
    {
        if (timer.packetID == packetID)
        {
            timer.interval = time;
            return;
        }
    }

    // If not found, add a new timer
    packetTimers.push_back({packetID, time});
}

//** CAN STATUS **//
void CAN_BUS::setCANStatusData()
{
    int d0[2]; // runtimeTime,numTxPaqError
    int d1[2]; // numRXPaqOK,numTXPaqOK
    int d2[2] = {0, 0};
    d0[0] = runtimeTime;
    d0[1] = numTxPaqError / numCurrentSamples;
    d1[0] = numRXPaqOK / numCurrentSamples;
    d1[1] = numTXPaqOK / numCurrentSamples;

    DEBUG_PRINTLN("Offset: ");
    DEBUG_PRINTLN(statusPacketOffset);

    // this->setPacket(statusPacketOffset, d0);
    // this->setPacket(statusPacketOffset + 1, d1);
    // this->setPacket(statusPacketOffset + 2, d2);
}

void CAN_BUS::printReceivedIds()
{
    DataIN.printAllPacketsIDs();
}

bool CAN_BUS::rebootBusFromError()
{
#if defined(ESP32) || defined(ESP32S3)
    twai_status_info_t status;
    twai_get_status_info(&status);

    if (status.state != TWAI_STATE_BUS_OFF) {
        return true;
    }

    twai_initiate_recovery();
    delay(10);

    //Comprobar si se ha recuperado
    twai_get_status_info(&status);
    if (status.state != TWAI_STATE_BUS_OFF) {
                return true;
    }

    return false; // no recuperado
#elif defined(STM32G4xx)
    uint32_t state = HAL_FDCAN_GetError(&hfdcan);
    if ((state & FDCAN_PSR_BO) == 0) {
        return true; // Not in bus-off
    }

    // Recover from bus-off
    CLEAR_BIT(hfdcan.Instance->CCCR, FDCAN_CCCR_INIT);
    delay(10);

    state = HAL_FDCAN_GetError(&hfdcan);
    if ((state & FDCAN_PSR_BO) == 0) {
        return true;
    }

    return false;
#endif
}

/**
 * Configura los temporizadores de envío de paquetes según su prioridad inversa (ID).
 * IDs más bajos (alta prioridad) se enviarán con intervalos más largos,
 * para no monopolizar el bus.
 */
void CAN_BUS::configurePacketTimersByPriority()
{
    const unsigned long ids[] = {
        10, 11, 12, 33, 65, 97, 129, 161, 193, 225, 257, 289, 321,
        353, 385, 386, 387, 388, 389, 390, 391, 392, 393, 400,
        1025, 1057, 1089, 1121, 1153, 1160, 1161, 1162, 1163,
        1164, 1165, 1166, 1167, 1168, 1169, 1170
    };

    const size_t numIds = sizeof(ids) / sizeof(ids[0]);
    const unsigned long minInterval = 50;    // ms
    const unsigned long maxInterval = 800;   // ms

    // Encontrar el ID mínimo y máximo reales
    unsigned long minId = ids[0];
    unsigned long maxId = ids[0];
    for (size_t i = 1; i < numIds; ++i) {
        if (ids[i] < minId) minId = ids[i];
        if (ids[i] > maxId) maxId = ids[i];
    }

    for (size_t i = 0; i < numIds; ++i) {
        // Escalar según el valor de ID
        float ratio = static_cast<float>(ids[i] - minId) / (maxId - minId);
        ratio = 1.0f - ratio;

        unsigned long interval = minInterval + static_cast<unsigned long>(ratio * (maxInterval - minInterval));

        setPacketTimer(ids[i], interval);

        DEBUG_PRINTLN((String)"[TIMER] ID " + ids[i] + " -> " + interval + " ms");
    }
}

#if defined(STM32G4xx)
HAL_StatusTypeDef CAN_BUS::initSTM32FDCAN(unsigned int speed)
{
    // Enable GPIO and FDCAN clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // RX on PA11, TX on PA12 (Standard Nucleo G474RE pins)
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hfdcan.Instance = FDCAN1;

    // FDCAN clock = 24 MHz (STM32G474RE con STM32duino, HSI=16MHz, PLLQ o HSE)
    // Formula: Baudrate = FDCAN_CLK / (Prescaler * (1 + TimeSeg1 + TimeSeg2))
    // Usando TimeSeg1=12, TimeSeg2=3 -> BTQ=16, Sample Point=81.25%
    //   125k  -> Prescaler = 24MHz / (125k  * 16) = 12   -> exacto
    //   250k  -> Prescaler = 24MHz / (250k  * 16) = 6    -> exacto
    //   500k  -> Prescaler = 24MHz / (500k  * 16) = 3    -> exacto
    //   1000k -> Prescaler = 24MHz / (1000k * 16) = 1.5  -> usar BTQ=12 (TimeSeg1=9, TimeSeg2=2), Prescaler=2
    hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan.Init.AutoRetransmission = ENABLE;
    hfdcan.Init.TransmitPause = DISABLE;
    hfdcan.Init.ProtocolException = DISABLE;

    switch(speed) {
        case 125:
            hfdcan.Init.NominalPrescaler = 12;  // 24MHz / (12 * 16) = 125000 bps
            hfdcan.Init.NominalTimeSeg1 = 12;
            hfdcan.Init.NominalTimeSeg2 = 3;
            break;
        case 250:
            hfdcan.Init.NominalPrescaler = 6;   // 24MHz / (6 * 16) = 250000 bps
            hfdcan.Init.NominalTimeSeg1 = 12;
            hfdcan.Init.NominalTimeSeg2 = 3;
            break;
        case 500:
            hfdcan.Init.NominalPrescaler = 3;   // 24MHz / (3 * 16) = 500000 bps
            hfdcan.Init.NominalTimeSeg1 = 12;
            hfdcan.Init.NominalTimeSeg2 = 3;
            break;
        case 1000:
            hfdcan.Init.NominalPrescaler = 2;   // 24MHz / (2 * 12) = 1000000 bps
            hfdcan.Init.NominalTimeSeg1 = 9;
            hfdcan.Init.NominalTimeSeg2 = 2;
            break;
        default:
            hfdcan.Init.NominalPrescaler = 12;  // Default to 125k
            hfdcan.Init.NominalTimeSeg1 = 12;
            hfdcan.Init.NominalTimeSeg2 = 3;
    }

    hfdcan.Init.NominalSyncJumpWidth = 1;
    
    // ExtBitTime
    hfdcan.Init.DataPrescaler = 1;
    hfdcan.Init.DataSyncJumpWidth = 1;
    hfdcan.Init.DataTimeSeg1 = 1;
    hfdcan.Init.DataTimeSeg2 = 1;

    hfdcan.Init.StdFiltersNbr = 1;
    hfdcan.Init.ExtFiltersNbr = 0;
    hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Configure standard filter based on profile
    configSTM32FDCANFilter(filterProfile);

    // Start the FDCAN module
    if (HAL_FDCAN_Start(&hfdcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void CAN_BUS::configSTM32FDCANFilter(int profile)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    
    // FilterID1 = ID base, FilterID2 = Mask (0x000 = todos los bits se ignoran -> acepta TODO)
    // En modo MASK: se acepta un frame si (frame_id & FilterID2) == (FilterID1 & FilterID2)
    // Con FilterID2=0x000 la máscara ignora todos los bits -> acepta cualquier ID
    switch (profile) {
        case 4: // VCU - solo acepta 0x401
            sFilterConfig.FilterID1 = 0x401;
            sFilterConfig.FilterID2 = 0x7FF; // máscara exacta
            break;
        case 2: // PDM - solo acepta 0x401
            sFilterConfig.FilterID1 = 0x401;
            sFilterConfig.FilterID2 = 0x7FF;
            break;
        case 3: // BMS
        default:
            // Acepta absolutamente todos los IDs estándar
            sFilterConfig.FilterID1 = 0x000;
            sFilterConfig.FilterID2 = 0x000; // máscara 0 -> acepta todo
            break;
    }
    
    HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
    
    // Las tramas que NO coincidan con ningún filtro van a FIFO0 en vez de rechazarse
    // Esto es clave para debug: garantiza que nada se descarte inesperadamente
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan,
        FDCAN_ACCEPT_IN_RX_FIFO0,   // Non-matching std frames -> FIFO0
        FDCAN_ACCEPT_IN_RX_FIFO0,   // Non-matching ext frames -> FIFO0
        FDCAN_FILTER_REMOTE,        // Remote std frames -> según filtro
        FDCAN_FILTER_REMOTE         // Remote ext frames -> según filtro
    );
}
#endif