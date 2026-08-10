//********MART CAN CONVERTER LIBRARY
#ifndef MARTCAN_H
#define MARTCAN_H

#include <Arduino.h>

#include <bitset>
#include <cstring>
#include <utility>


#include "CAN_DATA.h"
#include "common.h"

#if defined(ARDUINO_MICRO)
#include <ArduinoSTL.h>
#endif

#if defined(ESP32) || defined(ESP32S3)
#include <vector>
#include <algorithm>
#include <functional>
#include <ESP32-TWAI-CAN.hpp>
#elif defined(STM32G4xx)
#include <vector>
#include <algorithm>
#include <functional>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#endif

#define MCP_SPEED_500 500
#define MCP_SPEED_1000 1000

enum class HardwareType
{
    Transciever
};
class CAN_BUS
{

public:
    // CONVERTER converter;
    CAN_DATA DataIN, DataOUT;
    HardwareType type;
    int error;
    int RX, TX;
    int timeout;
    struct Config
    {
        bool simulating;              // Set to true for testing when no MCP2515 are connected to the microcontroller
        bool autoRemoveStoredFilters; // Removes the stored masks and filters when applied to the MCP2515 registers to save memory
        bool sendStatusData;          // Send status data such as runtime time, number of sent, received and collided paquets.
    } config;

#if defined(STM32G4xx)
    FDCAN_HandleTypeDef hfdcan;
    FDCAN_TxHeaderTypeDef TxHeader;
    FDCAN_RxHeaderTypeDef RxHeader;
#endif

    // Destructor
    ~CAN_BUS() {}

    void setupCANHardware(unsigned int speed, uint16_t txQueue = 10, uint16_t rxQueue = 10, int timeoutRead = 100, bool loopback = false)
    {
        error = 0;
        timeout = timeoutRead;
        // this->type = type; --> Cambiado, está en el constructor para que no de error a la hora de llamarlo en el main
        
        if (this->type == HardwareType::Transciever)
        {
#if defined(ESP32) || defined(ESP32S3)
            // ESP32Can.setPins(RX, TX);
            ESP32Can.setSpeed(ESP32Can.convertSpeed(speed));
            twai_filter_config_t filter = createFilterFromProfile(filterProfile);
            //**CORREGIR3: El tamaño de la cola debe de ser genérica y configurable por parámetro, podéis usar unos parámetros por defecto como en el caso del constructor
            if (!ESP32Can.begin(ESP32Can.convertSpeed(speed), TX, RX, txQueue, rxQueue, &filter))
            {
                Serial.println("Error Initializing ESP32Can...");
                error = 1;
            }
#elif defined(STM32G4xx)
            if (initSTM32FDCAN(speed, loopback) != HAL_OK)
            {
                Serial.println("Error Initializing FDCAN...");
                error = 1;
            }
#endif
        }
    }
    // Constructor: Initializes the transciever or controller instance and sets up the CAN interface
    CAN_BUS(HardwareType type, unsigned int speed, int _nodeID,  int8_t TX = 5, int8_t RX = 4, uint16_t txQueue = 30, uint16_t rxQueue = 30, bool loopback = false)
    {

        //**CORREGIR3: Sigue sin estar bien. En el caso de que se use el MCP2515 se tendrían que ignorar los argumentos TX y RX y no es posible ya que el pinCs está al final.
        //** Para solucionar esto podéis:
        //**1. Poner el pinCs como cuarto argumento y tx y rx los últimos además de implementar lógica adicional para configurar el controller o el transceiver, ya que aunque el constructor se llame con sólo 4 argumentos (en caso del controller),
        //** no se sabría el tipo de hardware a usar. */  o bien:
        //**2. Crear dos constructores que inicialicen un hardware u otro (con 4 argumentos para el controller y 6 para el transceiver) */
        //**3. Otra forma que se os ocurra a vosotros */
        //**Si veis que los constructores que yo implementé en su día os estan fastidiando y queréis usar otros para que sean más compatibles con vuestra lógica los podéis cambiar, no problem */

        this->type=type;
        this->TX = TX;
        this->RX = RX;
        this->filterProfile = _nodeID;
        timeout = 100;
        setupCANHardware(speed, txQueue, rxQueue, 100, loopback);
        // Default configuration
        config.simulating = false;
        config.autoRemoveStoredFilters = true;
        config.sendStatusData = false;
    }
    // Sends all stored data packets in DataOUT
    bool send();
    // setup for transciever or controller

    int SetupState()
    {
        return error;
    }

    // Sends a specific stored data packet in DataOUT
    bool send(unsigned long id);


    // Receives data packets and stores them in DataIN
    void receive();

    template <typename T>
    std::size_t calculateTotalSize(T &arg)
    {
        return arraySizeInBits(arg);
    }

    template <typename T, typename... Args>
    std::size_t calculateTotalSize(T &first, Args &...rest)
    {
        return arraySizeInBits(first) + calculateTotalSize(rest...);
    }

    //*** NEW SETPACKET AND GETPACKET METHODS */
    template <typename T>
    bool setPacket(uint32_t canId, const T *data, size_t dataSize, bool cmdConversionToBigEndian = true)
    {
        bool ok = true;
        // Calculate total size needed
        size_t dataBytes = sizeof(T) * dataSize;

        // Check for overflow
        if (dataBytes > 8)
        {
            ERROR_LOOP("ERROR, SETPACKET OVERFLOW");
            ok = false;
        }
        else
        {
            DataOUT.dataRaw.size = 8;
            DataOUT.dataRaw.id = canId;
            DataOUT.dataRaw.typeExtendedId = DataOUT.dataRaw.id > 0x7FF;
            uint8_t *outputArray = DataOUT.dataRaw.bytes;
            // Clear the output array
            std::memset(outputArray, 0xFF, 8);

            // Copy the data
            if (data != nullptr && dataSize > 0)
            {
                if (!cmdConversionToBigEndian)
                {
                    std::memcpy(outputArray, data, dataBytes);
                }
                else
                {
                    for (size_t i = 0; i < dataSize; ++i)
                    {
                        T value = data[i];
                        size_t elementSize = sizeof(T);

                        // Calculate starting position for this element in the output array
                        size_t startPos = i * elementSize;

                        // Copy bytes in reverse order (little endian to big endian)
                        for (size_t j = 0; j < elementSize; ++j)
                        {
                            outputArray[startPos + j] = static_cast<byte>((value >> ((elementSize - 1 - j) * 8)) & 0xFF);
                        }
                    }
                }
                DataOUT.addPacket(DataOUT.dataRaw);
            }
            else
            {
                ERROR_LOOP("ERROR, SETPACKET EMPTY");
                ok = false;
            }
        }
        return ok;
    }

    /**
     * Deserializes data from an 8-byte packet.
     *
     * @param inputArray The input 8-byte array
     * @param data Pointer to the data array to populate
     * @param dataSize Number of elements expected in the data array
     * @throws std::overflow_error If trying to extract more data than available
     */
    template <typename T>
    bool getPacket(uint32_t canId, T *data, size_t dataSize, bool cmdConversionToLittleEndian = true)
    {
        bool ok = true;
        // Calculate size
        size_t dataBytes = sizeof(T) * dataSize;

        // Check for overflow
        if (dataBytes > 8)
        {
            ERROR_LOOP("ERROR, GETPACKET OVERFLOW");
            ok = false;
        }
        else
        {
            CanPacketRawData packet;
            if (DataIN.getPacketById(canId, packet))
            {
                if (data != nullptr && dataSize > 0)
                {
                    // Extract the data
                    if (!cmdConversionToLittleEndian)
                    {
                        std::memcpy(data, packet.bytes, dataBytes);
                    }

                    else
                    {
                        for (size_t i = 0; i < dataSize; ++i)
                        {
                            T value = 0;
                            size_t elementSize = sizeof(T);

                            // Calculate starting position for this element in the input array
                            size_t startPos = i * elementSize;

                            // Convert big endian to little endian by reading bytes in reverse order
                            for (size_t j = 0; j < elementSize; ++j)
                            {
                                value = (value << 8) | packet.bytes[startPos + j];
                            }

                            data[i] = value;
                        }
                    }
                }
                else
                {
                    ERROR_PRINTLN("Error: getPacket Empty array");
                    ok = false;
                }
            }
            else
            {
                ERROR_PRINTLN("Error: getPacket No matching packet found.");
                ok = false;
            }
        }

        return (ok || config.simulating);
    }

    template <typename T>
    void packArgumentsRecursive(uint8_t *outputArray, size_t &offset, T &arg)
    {
        offset = packArgument(arg, outputArray, offset);
    }
    template <typename T, typename... Args>
    void packArgumentsRecursive(uint8_t *outputArray, size_t &offset, T &first, Args &...rest)
    {
        offset = packArgument(first, outputArray, offset);
        packArgumentsRecursive(outputArray, offset, rest...);
    }
    // Packs RRF message
    void setPacket(unsigned long canId)
    {
        DataOUT.dataRaw.size = 8;
        DataOUT.dataRaw.id = canId;
        DataOUT.dataRaw.typeExtendedId = (DataOUT.dataRaw.id > 0x7FF);
        DataOUT.addPacket(DataOUT.dataRaw);
    }

    // Function template to calculate size in bits of a single array
    template <typename T, std::size_t N>
    constexpr std::size_t arraySizeInBits(const T (&)[N])
    {
        return sizeof(T) * N * 8; // Size of each element times number of elements times 8 bits/byte
    }
    // Function template to calculate size in bits of a single array
    template <std::size_t N>
    constexpr std::size_t arraySizeInBits(const bool (&)[N])
    {
        return N; // 1 bit for each bool
    }

    // Helper function to pack a single array

    void printByteArray(const uint8_t *array, size_t size)
    {
        for (size_t i = 0; i < size; ++i)
        {
            if (array[i] < 0x10)
                Serial.print("0");
            Serial.print(array[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Template function to print an array of any type and size
    template <typename T, size_t N>
    void printArray(const T (&array)[N])
    {
        Serial.print("[");
        for (size_t i = 0; i < N; ++i)
        {
            if (i > 0)
            {
                Serial.print(", ");
            }
            Serial.print(array[i]);
        }
        Serial.println("]");
    }

    // Calculates and writes the masks and filters to the MCP2515 registers given a set of IDs
    bool setFilters(const unsigned long ids[], unsigned size);

    // Prints the calculated masks and filters
    void printFilters();

    // Test by software if the given IDs are accepted by the created filters or not
    void testFilters(const std::vector<uint16_t> &testIds);

    void setPacketTimer(unsigned long packetID, unsigned long time);
    void printStatusData(unsigned _nodeID);
    void printReceivedIds();

    //** CAN STATUS DATA**//
    void setCANStatusData();

    //** CAN BUS STATUS DATA **//
    unsigned nodeID, statusPacketOffset;                                                             // IDs
    unsigned runtimeTime, numRXPaqOK, numTXPaqOK, numTxPaqError;                                     // Actual data
    unsigned previousStatusIntervalTime, previousStatusRuntimeTime, intervalTime, numCurrentSamples; // Aux data

    //** BUS_OFF REBOOT **//
    bool rebootBusFromError();

    /**
     * @brief ¿El periférico está AHORA en bus-off? Solo consulta, no recupera.
     *
     * rebootBusFromError() devuelve true tanto si el bus estaba sano como si
     * logró recuperarlo, así que desde su retorno NO se pueden contar los
     * episodios de bus-off. Llamar a esto ANTES de recuperar para llevar la
     * cuenta (telemetría/post-mortem: tras un corte, saber que lo hubo).
     */
    bool isBusOff();

    //** PACKET TIMING **//
    void configurePacketTimersByPriority();

private:
    int filterProfile;
    struct PacketTimer
    {
        unsigned long packetID;
        unsigned long interval;
    };
    std::vector<PacketTimer> packetTimers;

    std::vector<unsigned long> filterIDs; // Vector holding INRRFid and OUTRRFid vectors

    bool readBytes();
    bool writeBytes();


    /*
    //Change the speed of the bus while running (ms)
    void ChangeSpeed(unsigned int speed){
        ESP32Can.setSpeed(ESP32Can.convertSpeed(speed));
    }
    */

#if defined(STM32G4xx)
    HAL_StatusTypeDef initSTM32FDCAN(unsigned int speed, bool loopback = false);
    void configSTM32FDCANFilter(int profile);
#endif

#if defined(ESP32) || defined(ESP32S3)
    twai_filter_config_t createFilterFromProfile(int profile)
    {
        switch(profile)
        {
            case 4:  // VCU
                return filter_VCU();

            case 3:  // BMS
                return filter_BMS();

            case 2:  // PDM
                return filter_PDM();
            default:
                return TWAI_FILTER_CONFIG_ACCEPT_ALL();
        }
    }
    
    twai_filter_config_t filter_VCU(){
        twai_filter_config_t f;
        f.single_filter = true;
        f.acceptance_code = (0x401 << 21);
        f.acceptance_mask = 
        (1 << 26) |
        (1<<27) |
        (1<<28);
        
        return f;
    }

    twai_filter_config_t filter_BMS()
    {
        twai_filter_config_t f;
        return f;
    }
    twai_filter_config_t filter_PDM()
    {
        twai_filter_config_t f;
        f.single_filter = true;
        f.acceptance_code = (0x401 << 21);
        f.acceptance_mask = 
        (1 << 26) |
        (1<<27) |
        (1<<28);
        
        return f;
    }
#endif // defined(ESP32) || defined(ESP32S3)
};

#endif