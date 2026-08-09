/**
 * @file    FaultLogger.h
 * @brief   Logger de eventos de fallo en FRAM externa MB85RC256V (I²C).
 *
 * Pensado para diagnóstico post-mortem cuando el CAN falla o el coche se
 * apaga: registra cada transición de BMS_OK, cada reInit, cada boot y
 * la causa de reset en una FRAM no volátil. Lectura por comando serie.
 *
 * Hardware:
 *   MB85RC256V (Fujitsu / Adafruit breakout) — 32 KB I²C @ 400 kHz.
 *   SCL = PC8  SDA = PC9   (I²C3 en la PCB nueva; PB8/PB9 son TSON).
 *   Address = 0x50 (A0/A1/A2 a GND). WP a GND.
 *
 * Estructura en FRAM (ver doc):
 *   [0..15]      Header  (magic "MART", version, writeIdx, totalEv, ...)
 *   [16..32767]  Ring buffer de 2047 eventos × 16 bytes
 *
 * Endurance FRAM = 10^12 ciclos → loguear cada evento es seguro durante
 * toda la vida del coche.
 */

#ifndef FAULT_LOGGER_H
#define FAULT_LOGGER_H

#include <Arduino.h>

/**
 * Registro de evento (16 bytes en FRAM). Los flags_* son una copia de
 * los bytes 0/1/2/3 del CAN ID 15 (ver ARQUITECTURA.md / CAN_Solo2DL.md).
 */
struct FaultRecord {
    uint8_t  eventType;     ///< Enum FaultLogger::EVT_*
    uint8_t  firstFault;    ///< Primer trigger del episodio (0..6)
    uint8_t  flagsConf;     ///< B0 del CAN ID 15 (drivers de bmsFault)
    uint8_t  flagsHall;     ///< B1 del CAN ID 15 (sub-fallos Hall)
    uint8_t  flagsSnap;     ///< B2 del CAN ID 15 (V/T snapshot)
    uint8_t  flagsState;    ///< B3 del CAN ID 15 (state)
    uint8_t  resetCause;    ///< B7 del CAN ID 15 (RCC->CSR — útil solo en BOOT)
    uint16_t minV_mV;       ///< Tensión celda más baja (mV)
    uint16_t maxV_mV;       ///< Tensión celda más alta (mV)
    int8_t   maxT_C;        ///< Temp máx del pack (°C, signed)
};

class FaultLogger {
public:

    // Tipo de evento (1 byte). Mantener la numeración estable: estos
    // códigos quedan persistidos en FRAM y se interpretan al volcar.
    enum EventType : uint8_t {
        EVT_NONE           = 0,
        EVT_BOOT           = 1,   ///< setup() arrancó (con resetCause válido)
        EVT_BMS_OK_FALL    = 2,   ///< bmsFault: false → true
        EVT_BMS_OK_RISE    = 3,   ///< bmsFault: true → false (despeje)
        EVT_REINIT_TRY     = 4,   ///< intento de bms.reInit() (rate-limited)
        EVT_PRECHARGE_FAIL = 5,   ///< timeout 5 s de precarga
    };

    /// Almacén en uso tras begin().
    enum Backend : uint8_t {
        BK_NONE  = 0,   ///< sin almacenamiento: log() es no-op
        BK_FRAM  = 1,   ///< MB85RC256V por I²C (32 KB, ring con header)
        BK_FLASH = 2,   ///< flash interna del STM32 (4 KB, append-only)
    };

    /**
     * Inicializa el log. Intenta primero la FRAM (I²C3 PC8/PC9 @ 400 kHz);
     * si el MB85RC256V no responde, cae automáticamente a la FLASH INTERNA
     * del STM32 (últimos 4 KB), que persiste igual entre apagados.
     * @return true si algún backend quedó listo para loguear.
     */
    bool begin();

    bool    isReady() const { return _ready; }
    Backend backend() const { return _backend; }

    /**
     * Añade un evento al ring buffer. Avanza writeIdx, incrementa
     * rollover si se da la vuelta, actualiza el header.
     * El timestamp_ms lo pone esta función (millis()).
     */
    bool log(const FaultRecord& rec);

    /// Vuelca todos los eventos a Serial en orden cronológico (oldest→newest).
    /// @param tick Callback opcional llamado cada N records. Útil para
    ///             refrescar el IWDG durante volcados largos (>8 s).
    void dumpToSerial(void (*tick)() = nullptr);

    /// Resetea writeIdx=0, rollovers=0, totalEv=0 (no borra los datos).
    void clearLog();

    uint16_t writeIndex()    const { return _writeIdx; }
    uint16_t rolloverCount() const { return _rollovers; }
    uint32_t totalEvents()   const { return _totalEv; }

private:
    bool _ready = false;
    uint16_t _writeIdx   = 0;
    uint16_t _rollovers  = 0;
    uint32_t _totalEv    = 0;

    static constexpr uint8_t  I2C_ADDR   = 0x50;
    static constexpr uint16_t REC_SIZE   = 16;
    static constexpr uint16_t HDR_SIZE   = 16;
    static constexpr uint16_t FRAM_SIZE  = 32768;
    static constexpr uint16_t CAPACITY   = (FRAM_SIZE - HDR_SIZE) / REC_SIZE; // 2047
    static constexpr uint32_t MAGIC      = 0x4D415254UL; ///< "MART" big-endian
    static constexpr uint8_t  VERSION    = 1;

    // ── Backend FLASH interna ────────────────────────────────────────────────
    // Últimos 4 KB de la flash del STM32G474RE (512 KB): 0x0807F000-0x0807FFFF.
    // La dirección está alineada tanto a página de 2 KB (dual-bank, el default
    // del G474: páginas 126-127 del banco 2) como de 4 KB (single-bank: página
    // 127); _flashErase() detecta el modo en runtime vía FLASH_OPTR_DBANK.
    // El firmware ocupa ~16 % de la flash, así que esta zona queda muy lejos
    // del código. Diferencias con la FRAM que fuerzan otro diseño:
    //   · Solo se escribe en doubleword (64 bits) alineado.
    //   · NO se puede reescribir sin borrar la página entera → NADA de header
    //     (que habría que reescribir en cada log): el log es APPEND-ONLY y el
    //     punto de escritura se busca al arrancar (primer slot todo 0xFF).
    //   · El borrado bloquea la CPU ~20-40 ms → solo al llenarse (1 vez cada
    //     FLASH_LOG_SLOTS eventos), nunca en un log normal.
    static constexpr uint32_t FLASH_LOG_ADDR  = 0x0807F000UL;  ///< base del log en flash
    static constexpr uint16_t FLASH_LOG_SLOTS = 256;           ///< 4096 B / 16 B por registro

    Backend  _backend   = BK_NONE;
    uint16_t _flashSlot = 0;        ///< próximo slot libre (== FLASH_LOG_SLOTS si lleno)

    bool _flashBegin();
    bool _flashLog(const uint8_t* rec);
    void _flashDump(void (*tick)());
    bool _flashErase();

    bool _readBytes (uint16_t addr, uint8_t* data, size_t len);
    bool _writeBytes(uint16_t addr, const uint8_t* data, size_t len);
    bool _readHeader();
    bool _writeHeader();
    void _formatLog();

    static void _serialize(const FaultRecord& rec, uint32_t ts, uint8_t out[16]);
    static void _printRec(uint16_t idx, const uint8_t* rec);

    static const char* _eventName(uint8_t type);
};

#endif // FAULT_LOGGER_H
