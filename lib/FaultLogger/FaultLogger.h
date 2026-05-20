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
 *   SCL = PB8  SDA = PB9   (I²C1 alt en NUCLEO-G474RE).
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

    /**
     * Inicializa I²C1 en PB8/PB9 @ 400 kHz, comprueba que el MB85RC256V
     * responde, lee/valida el header. Si el magic está corrupto o no
     * existe (primera vez), formatea el log.
     * @return true si la FRAM responde y queda lista para loguear.
     */
    bool begin();

    bool isReady() const { return _ready; }

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

    bool _readBytes (uint16_t addr, uint8_t* data, size_t len);
    bool _writeBytes(uint16_t addr, const uint8_t* data, size_t len);
    bool _readHeader();
    bool _writeHeader();
    void _formatLog();

    static const char* _eventName(uint8_t type);
};

#endif // FAULT_LOGGER_H
