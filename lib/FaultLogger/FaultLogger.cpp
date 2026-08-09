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
    Wire.setSCL(PC8);   // I2C3_SCL (PB8 ahora es TSON_FAIL en la PCB nueva)
    Wire.setSDA(PC9);   // I2C3_SDA (PB9 ahora es TSON_BTN)
    Wire.begin();
    Wire.setClock(400000);    // Fast Mode (la chip aguanta hasta 1 MHz)

    // Probe: dirigir un write vacío al slave; si no responde → no hay FRAM.
    Wire.beginTransmission(I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        // Sin FRAM montada → usar la flash interna del STM32 (persiste igual).
        Serial.println(F("[LOG] FRAM no responde (I2C 0x50) -> usando FLASH interna"));
        return _flashBegin();
    }

    if (!_readHeader()) {
        // Header inválido (primera vez, o FRAM corrupta) → formatear.
        Serial.println(F("[FRAM] Header invalido, formateando..."));
        _formatLog();
    }

    _ready   = true;
    _backend = BK_FRAM;
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

    uint8_t buf[REC_SIZE];
    _serialize(rec, millis(), buf);

    if (_backend == BK_FLASH) return _flashLog(buf);

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
        Serial.println(F("[LOG] sin almacenamiento (ni FRAM ni FLASH)"));
        return;
    }
    if (_backend == BK_FLASH) { _flashDump(tick); return; }

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
        _printRec(i, rec);

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
    if (_backend == BK_FLASH) {
        Serial.println(F("[FLASH] borrando log (bloquea ~40 ms)..."));
        Serial.println(_flashErase() ? F("[FLASH] log limpiado.")
                                     : F("[FLASH] ERROR al borrar."));
        return;
    }
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

// ============================================================================
//  Serialización / impresión — compartidas por los dos backends
// ============================================================================
void FaultLogger::_serialize(const FaultRecord& rec, uint32_t ts, uint8_t out[16])
{
    // Big-endian: legible al dump byte a byte
    out[0]  = (uint8_t)(ts >> 24);
    out[1]  = (uint8_t)(ts >> 16);
    out[2]  = (uint8_t)(ts >>  8);
    out[3]  = (uint8_t)(ts      );
    out[4]  = rec.eventType;
    out[5]  = rec.firstFault;
    out[6]  = rec.flagsConf;
    out[7]  = rec.flagsHall;
    out[8]  = rec.flagsSnap;
    out[9]  = rec.flagsState;
    out[10] = rec.resetCause;
    out[11] = (uint8_t)(rec.minV_mV >> 8);
    out[12] = (uint8_t)(rec.minV_mV     );
    out[13] = (uint8_t)(rec.maxV_mV >> 8);
    out[14] = (uint8_t)(rec.maxV_mV     );
    out[15] = (uint8_t)rec.maxT_C;
}

void FaultLogger::_printRec(uint16_t idx, const uint8_t* rec)
{
    const uint32_t ts  = ((uint32_t)rec[0] << 24) | ((uint32_t)rec[1] << 16)
                       | ((uint32_t)rec[2] <<  8) |  (uint32_t)rec[3];
    const uint16_t mnV = ((uint16_t)rec[11] << 8) | rec[12];
    const uint16_t mxV = ((uint16_t)rec[13] << 8) | rec[14];
    const int8_t   mxT = (int8_t)rec[15];

    char line[140];
    snprintf(line, sizeof(line),
        "  %4u  %10lu  %-5s  %2u  0x%02X  0x%02X  0x%02X  0x%02X  0x%02X  %4u  %4u  %4d",
        (unsigned)idx, (unsigned long)ts, _eventName(rec[4]), rec[5],
        rec[6], rec[7], rec[8], rec[9], rec[10],
        mnV, mxV, mxT);
    Serial.println(line);
}

// ============================================================================
//  Backend FLASH interna (fallback si no hay FRAM) — append-only
// ============================================================================
// Busca el punto de escritura: el primer slot sin programar (todo 0xFF).
// Todo lo anterior son eventos válidos en orden cronológico.
bool FaultLogger::_flashBegin()
{
    _flashSlot = FLASH_LOG_SLOTS;                     // por defecto: lleno
    for (uint16_t i = 0; i < FLASH_LOG_SLOTS; i++) {
        const volatile uint64_t* p =
            (const volatile uint64_t*)(FLASH_LOG_ADDR + (uint32_t)i * REC_SIZE);
        if (p[0] == 0xFFFFFFFFFFFFFFFFULL && p[1] == 0xFFFFFFFFFFFFFFFFULL) {
            _flashSlot = i;
            break;
        }
    }

    _ready   = true;
    _backend = BK_FLASH;
    {
        char b[96];
        snprintf(b, sizeof(b),
                 "[FLASH] log OK @0x%08lX — capacidad=%u eventos, usados=%u",
                 (unsigned long)FLASH_LOG_ADDR, (unsigned)FLASH_LOG_SLOTS,
                 (unsigned)_flashSlot);
        Serial.println(b);
    }
    return true;
}

// Programa un registro (2 doublewords). Si el log está lleno, borra y reinicia:
// se pierde el histórico, pero se sigue capturando lo nuevo (que es lo que
// interesa tras un incidente). Volcar con 'd' antes de que se llene.
bool FaultLogger::_flashLog(const uint8_t* rec)
{
    if (_flashSlot >= FLASH_LOG_SLOTS) {
        Serial.println(F("[FLASH] log lleno -> borrando y reiniciando."));
        if (!_flashErase()) return false;
    }

    uint64_t dw[2];
    memcpy(dw, rec, REC_SIZE);                        // 16 B = 2 doublewords
    const uint32_t addr = FLASH_LOG_ADDR + (uint32_t)_flashSlot * REC_SIZE;

    HAL_FLASH_Unlock();
    bool ok = (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr,     dw[0]) == HAL_OK)
           && (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8, dw[1]) == HAL_OK);
    HAL_FLASH_Lock();

    if (ok) _flashSlot++;
    return ok;
}

void FaultLogger::_flashDump(void (*tick)())
{
    Serial.println(F("\n========== FAULT LOG (FLASH) =========="));
    {
        char b[96];
        snprintf(b, sizeof(b), "Eventos: %u / %u",
                 (unsigned)_flashSlot, (unsigned)FLASH_LOG_SLOTS);
        Serial.println(b);
    }
    if (_flashSlot == 0) {
        Serial.println(F("(vacío)"));
        Serial.println(F("================================\n"));
        return;
    }

    Serial.println(F(""));
    Serial.println(F("  #     Time(ms)  Type   FF  B0   B1   B2   B3   Rst   MinV  MaxV  MaxT"));
    Serial.println(F("  ----  ----------  -----  --  ----  ----  ----  ----  ----  ----  ----  ----"));

    for (uint16_t i = 0; i < _flashSlot; i++) {
        _printRec(i, (const uint8_t*)(FLASH_LOG_ADDR + (uint32_t)i * REC_SIZE));
        if (tick && (i & 0x0F) == 0x0F) tick();       // refresca el IWDG
    }
    Serial.println(F("================================\n"));
}

// Borra la zona del log. BLOQUEA la CPU ~20-40 ms (no se puede leer flash del
// mismo banco mientras). Detecta dual/single-bank en runtime: la dirección
// base está alineada a ambos tamaños de página.
bool FaultLogger::_flashErase()
{
    FLASH_EraseInitTypeDef er = {};
    er.TypeErase = FLASH_TYPEERASE_PAGES;

    if ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0) {      // dual-bank: páginas de 2 KB
        er.Banks   = FLASH_BANK_2;
        er.Page    = (FLASH_LOG_ADDR - 0x08040000UL) / 2048UL;   // 126
        er.NbPages = 2;                                          // 126 y 127
    } else {                                          // single-bank: páginas de 4 KB
        er.Banks   = FLASH_BANK_1;
        er.Page    = (FLASH_LOG_ADDR - 0x08000000UL) / 4096UL;   // 127
        er.NbPages = 1;
    }

    uint32_t pageErr = 0;
    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&er, &pageErr);
    HAL_FLASH_Lock();

    _flashSlot = (st == HAL_OK) ? 0 : FLASH_LOG_SLOTS;
    return st == HAL_OK;
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
