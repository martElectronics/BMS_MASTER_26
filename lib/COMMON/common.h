#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

// HARDENED para BMS de seguridad: el original era while(true){...delay}
// -> un setPacket con overflow/empty congelaria TODA la seguridad del
// BMS para siempre (sin V/T, sin BMS_OK). Ahora loguea y SIGUE; el
// caller ya gestiona el fallo (pone ok=false y retorna).
#define ERROR_LOOP(msg)  do { Serial.println(msg); } while (0)

struct CanPacketRawData {
    unsigned long id;
    byte size;
    byte bytes[8];
    byte typeExtendedId;
    unsigned long nextSendTime = 0;
};



#endif // COMMON_H