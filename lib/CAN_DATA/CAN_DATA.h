//********MART CAN DATA STORAGE LIBRARY
#ifndef CANDATA_H
#define CANDATA_H
#include <Arduino.h>
#include "common.h"

#if defined(ARDUINO_MICRO)
#include <ArduinoSTL.h>
#endif

// CAN_DATA usa std::vector/find_if/lower_bound/remove_if/memcpy SIEMPRE.
// El guard original solo los incluia en ESP32 (+ bug `(ESP32S3)` y typo
// `ARDUNO_MICRO`) -> NO compilaba en STM32. Includes incondicionales
// (las cabeceras STL ya tienen su propio include guard).
#include <vector>
#include <algorithm>
#include <functional>
#include <cstring>

class CAN_DATA
{
private:
    std::vector<CanPacketRawData> packets;   // Vector to store CAN packets
    // Si true, getPacketById() BORRA el paquete al leerlo (consume-on-read):
    // cada paquete se entrega UNA sola vez por cada vez que entra (RX) o se
    // encola (TX). Es el comportamiento por defecto de esta lib.
    bool allIdsRemovable;
public:
    CAN_DATA()
    {
        // ⚠ OJO: consume-on-read. getPacketById() elimina el paquete tras
        // devolverlo, así que NO se puede leer el mismo ID dos veces sin que
        // vuelva a llegar/encolarse. (El comentario antiguo decía lo contrario
        // —"persist… read multiple times"— y era falso.)
        allIdsRemovable = true;
    }

    CanPacketRawData dataRaw; // Raw data of CAN packet

    // Adds a CAN packet to the storage, keeping packets sorted by their ID
    void addPacket(const CanPacketRawData &packet)
    {
        // Use std::find_if to check if a packet with the same id already exists
        auto it = std::find_if(packets.begin(), packets.end(), [&packet](const CanPacketRawData &a)
                               { return a.id == packet.id; });

        if (it != packets.end())
        {
            // If a packet with the same id is found, update its information
            it->size = packet.size;
            memcpy(it->bytes, packet.bytes, sizeof(packet.bytes));
        }
        else
        {
            // If no packet with the same id exists, add the new packet in sorted order
            auto insertIt = std::lower_bound(packets.begin(), packets.end(), packet,
                                             [](const CanPacketRawData &a, const CanPacketRawData &b)
                                             {
                                                 return a.id < b.id;
                                             });
            auto insertedIt = packets.insert(insertIt, packet); // Insert and get iterator to the new element
            DEBUG_PRINTLN("Paquete anadido");
        }
    }

    // Removes a CAN packet from the storage by its ID
    void removePacket(unsigned long id)
    {
        packets.erase(std::remove_if(packets.begin(), packets.end(),
                                     [id](const CanPacketRawData &packet)
                                     {
                                         return packet.id == id;
                                     }),
                      packets.end());
    }

    // Devuelve por copia el paquete con ese ID y, si allIdsRemovable (default),
    // lo BORRA del vector (consume-on-read). Devuelve false si no existe.
    // ⚠ Efecto secundario: leer = consumir. No es un "get" idempotente.
    bool getPacketById(unsigned long id, CanPacketRawData &outPacket)
    {
        auto it = std::find_if(packets.begin(), packets.end(),
                               [id](const CanPacketRawData &packet)
                               { return packet.id == id; });
        if (it != packets.end())
        {
            outPacket = *it;
            // Check if this ID should be auto-removed or if all IDs are removable
            if (allIdsRemovable)
            {
                packets.erase(it); // Remove the packet
            }
            return true;
        }
        else
        {
            return false; // Packet not found
        }
    }

    // Executes a provided function on each packet
    template <typename Func>
    void forEachPacket(Func func)
    {
        for (auto &packet : packets)
        {
            func(packet);
        }
    }

    void printAllPacketsIDs() const
    {
        unsigned cont = 1;
        if (packets.empty())
        {
            Serial.println("No packets stored.");
            return;
        }
        else
        {
            for (const auto &packet : packets)
            {
                Serial.println((String) "Packet " + cont);
                Serial.print("ID = ");
                Serial.println(packet.id); // Assuming ID is hexadecimal
                Serial.println(); // New line after printing each packet
                cont++;
            }
        }
    }
};

#endif
