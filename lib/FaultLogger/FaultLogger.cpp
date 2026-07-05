/**
 * @file    FaultLogger.cpp
 * @brief   Driver del MB85RC256V (FRAM I²C 32 KB) + ring buffer de eventos.
 *
 * Protocolo MB85RC256V:
 *   Write: ST | slave+W | addrH | addrL | data... | SP
 *   Read : ST | slave+W | addrH | addrL | RS | slave+R | data... | SP
 *   Direccionamiento: 15 bits (0..32767). El MSb del addrH se ignora.
 *
 * Wire en STM32duino: setSCL/setSDA antes de begin(); endTransmission(false)
 * genera un repeated-start válido para el secuencia read.
 */

#include "FaultLogger.h"
#include <Wire.h>

// ============================================================================
//  begin() — init I²C, probe device, leer o formatear header
// ============================================================================
bool FaultLogger::begin()
{
    Wire.setSCL(PB9);
    Wire.setSDA(PB8);
    Wire.begin();
    Wire.setClock(400000);    // Fast Mode (la chip aguanta hasta 1 MHz)

    // Probe: dirigir un write vacío al slave; si no responde → no hay FRAM.
    Wire.beginTransmission(I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("[FRAM] MB85RC256V no responde (I2C 0x50)"));
        _ready = false;
        return false;
    }

    if (!_readHeader()) {
        // Header inválido (primera vez, o FRAM corrupta) → formatear.
        Serial.println(F("[FRAM] Header invalido, formateando..."));
        _formatLog();
    }

    _ready = true;
    {
        char b[96];
        snprintf(b, sizeof(b),
            "[FRAM] OK — capacidad=%u eventos, writeIdx=%u, rollovers=%u, total=%lu",
            (unsigned)CAPACITY, _writeIdx, _rollovers, (unsigned long)_totalEv);
        Serial.println(b);
    }
    return true;
}

// ============================================================================
//  log() — añade un evento, avanza writeIdx, persiste header
// ============================================================================
bool FaultLogger::log(const FaultRecord& rec)
{
    if (!_ready) return false;

    const uint32_t ts = millis();
    uint8_t buf[REC_SIZE];

    // Big-endian: legible al dump byte a byte
    buf[0]  = (uint8_t)(ts >> 24);
    buf[1]  = (uint8_t)(ts >> 16);
    buf[2]  = (uint8_t)(ts >>  8);
    buf[3]  = (uint8_t)(ts      );
    buf[4]  = rec.eventType;
    buf[5]  = rec.firstFault;
    buf[6]  = rec.flagsConf;
    buf[7]  = rec.flagsHall;
    buf[8]  = rec.flagsSnap;
    buf[9]  = rec.flagsState;
    buf[10] = rec.resetCause;
    buf[11] = (uint8_t)(rec.minV_mV >> 8);
    buf[12] = (uint8_t)(rec.minV_mV     );
    buf[13] = (uint8_t)(rec.maxV_mV >> 8);
    buf[14] = (uint8_t)(rec.maxV_mV     );
    buf[15] = (uint8_t)rec.maxT_C;

    const uint16_t addr = HDR_SIZE + _writeIdx * REC_SIZE;
    // Orden: 1º registro, 2º header. Si se corta la corriente entre los
    // dos, el slot queda con datos válidos pero writeIdx aún apunta a él
    // → el próximo evento lo sobrescribe sin corrupción global.
    if (!_writeBytes(addr, buf, REC_SIZE)) return false;

    _writeIdx = (_writeIdx + 1) % CAPACITY;
    if (_writeIdx == 0) _rollovers++;
    if (_totalEv < UINT32_MAX) _totalEv++;

    return _writeHeader();
}

// ============================================================================
//  dumpToSerial() — vuelca todos los eventos en orden cronológico
// ============================================================================
void FaultLogger::dumpToSerial(void (*tick)())
{
    if (!_ready) {
        Serial.println(F("[LOG] FRAM no ready"));
        return;
    }

    Serial.println(F("\n========== FAULT LOG =========="));
    {
        char b[96];
        snprintf(b, sizeof(b),
            "Total eventos: %lu | Rollovers: %u | WriteIdx: %u | Capacidad: %u",
            (unsigned long)_totalEv, _rollovers, _writeIdx, (unsigned)CAPACITY);
        Serial.println(b);
    }

    const uint16_t count = (_totalEv < CAPACITY) ? (uint16_t)_totalEv : CAPACITY;
    const uint16_t start = (_totalEv < CAPACITY) ? 0                  : _writeIdx;

    if (count == 0) {
        Serial.println(F("(vacío)"));
        Serial.println(F("================================\n"));
        return;
    }

    Serial.println(F(""));
    Serial.println(F("  #     Time(ms)  Type   FF  B0   B1   B2   B3   Rst   MinV  MaxV  MaxT"));
    Serial.println(F("  ----  ----------  -----  --  ----  ----  ----  ----  ----  ----  ----  ----"));

    uint8_t rec[REC_SIZE];
    for (uint16_t i = 0; i < count; i++) {
        const uint16_t idx  = (start + i) % CAPACITY;
        const uint16_t addr = HDR_SIZE + idx * REC_SIZE;
        if (!_readBytes(addr, rec, REC_SIZE)) {
            Serial.printf("  %4u  read-error\n", (unsigned)i);
            continue;
        }
        const uint32_t ts  = ((uint32_t)rec[0] << 24) | ((uint32_t)rec[1] << 16)
                           | ((uint32_t)rec[2] <<  8) |  (uint32_t)rec[3];
        const uint16_t mnV = ((uint16_t)rec[11] << 8) | rec[12];
        const uint16_t mxV = ((uint16_t)rec[13] << 8) | rec[14];
        const int8_t   mxT = (int8_t)rec[15];

        char line[140];
        snprintf(line, sizeof(line),
            "  %4u  %10lu  %-5s  %2u  0x%02X  0x%02X  0x%02X  0x%02X  0x%02X  %4u  %4u  %4d",
            (unsigned)i, (unsigned long)ts, _eventName(rec[4]), rec[5],
            rec[6], rec[7], rec[8], rec[9], rec[10],
            mnV, mxV, mxT);
        Serial.println(line);

        // Refresco del watchdog cada 16 records (el dump completo de 2047
        // records a 115200 baud puede tardar ~18 s; IWDG = 8 s).
        if (tick && (i & 0x0F) == 0x0F) tick();
    }
    Serial.println(F("================================\n"));
}

// ============================================================================
//  clearLog()
// ============================================================================
void FaultLogger::clearLog()
{
    if (!_ready) return;
    _writeIdx  = 0;
    _rollovers = 0;
    _totalEv   = 0;
    _writeHeader();
    Serial.println(F("[FRAM] log limpiado."));
}

// ============================================================================
//  Privadas — I/O bruto
// ============================================================================
bool FaultLogger::_writeBytes(uint16_t addr, const uint8_t* data, size_t len)
{
    Wire.beginTransmission(I2C_ADDR);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    for (size_t i = 0; i < len; i++) Wire.write(data[i]);
    return Wire.endTransmission() == 0;
}

bool FaultLogger::_readBytes(uint16_t addr, uint8_t* data, size_t len)
{
    Wire.beginTransmission(I2C_ADDR);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;   // repeated-start

    const size_t got = Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)len);
    if (got != len) return false;
    for (size_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        data[i] = Wire.read();
    }
    return true;
}

bool FaultLogger::_readHeader()
{
    uint8_t hdr[HDR_SIZE];
    if (!_readBytes(0, hdr, HDR_SIZE)) return false;

    const uint32_t magic = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16)
                         | ((uint32_t)hdr[2] <<  8) |  (uint32_t)hdr[3];
    if (magic != MAGIC)            return false;
    if (hdr[4] != VERSION)         return false;
    if (hdr[5] != REC_SIZE)        return false;
    const uint16_t cap = ((uint16_t)hdr[6] << 8) | hdr[7];
    if (cap != CAPACITY)           return false;

    _writeIdx  = ((uint16_t)hdr[8]  << 8) | hdr[9];
    _rollovers = ((uint16_t)hdr[10] << 8) | hdr[11];
    _totalEv   = ((uint32_t)hdr[12] << 24) | ((uint32_t)hdr[13] << 16)
               | ((uint32_t)hdr[14] <<  8) |  (uint32_t)hdr[15];

    // Saneo: si writeIdx fuera de rango (corrupción), reset.
    if (_writeIdx >= CAPACITY) {
        _writeIdx = 0;
        return false;
    }
    return true;
}

bool FaultLogger::_writeHeader()
{
    uint8_t hdr[HDR_SIZE];
    hdr[0]  = (uint8_t)(MAGIC >> 24);
    hdr[1]  = (uint8_t)(MAGIC >> 16);
    hdr[2]  = (uint8_t)(MAGIC >>  8);
    hdr[3]  = (uint8_t)(MAGIC      );
    hdr[4]  = VERSION;
    hdr[5]  = REC_SIZE;
    hdr[6]  = (uint8_t)(CAPACITY  >> 8);
    hdr[7]  = (uint8_t)(CAPACITY      );
    hdr[8]  = (uint8_t)(_writeIdx >> 8);
    hdr[9]  = (uint8_t)(_writeIdx     );
    hdr[10] = (uint8_t)(_rollovers >> 8);
    hdr[11] = (uint8_t)(_rollovers     );
    hdr[12] = (uint8_t)(_totalEv >> 24);
    hdr[13] = (uint8_t)(_totalEv >> 16);
    hdr[14] = (uint8_t)(_totalEv >>  8);
    hdr[15] = (uint8_t)(_totalEv      );
    return _writeBytes(0, hdr, HDR_SIZE);
}

void FaultLogger::_formatLog()
{
    _writeIdx  = 0;
    _rollovers = 0;
    _totalEv   = 0;
    _writeHeader();
}

const char* FaultLogger::_eventName(uint8_t type)
{
    switch (type) {
        case EVT_BOOT:           return "BOOT ";
        case EVT_BMS_OK_FALL:    return "FALL ";
        case EVT_BMS_OK_RISE:    return "RISE ";
        case EVT_REINIT_TRY:     return "REINI";
        case EVT_PRECHARGE_FAIL: return "PREFL";
        default:                 return "?    ";
    }
}
