/**
 * @file    TelemetryDashboard.h
 * @brief   Telemetría EN VIVO en JSON por el Serial (para tools/TelemetryWeb).
 *
 * Cada refreshMs emite una línea JSON con el resumen del pack (V/T min/max,
 * corriente, SOC, fan, fallos) y las tensiones/temperaturas de cada módulo.
 * La página tools/TelemetryWeb la lee por Web Serial y la pinta.
 *
 * ── Uso ──────────────────────────────────────────────────────────────────
 *   · DESACTIVADO por defecto. Se activa/desactiva con la tecla 'j'.
 *   · Mientras está activo, el firmware no imprime su estado periódico.
 *
 * ── Diseño ───────────────────────────────────────────────────────────────
 *   · Clase independiente: solo LEE los getters ya cacheados del BQ / Hall /
 *     SOC / Fan (NO dispara lecturas nuevas — de eso se ocupa el main).
 *   · El mapeo módulo→celda/NTC (que vive en el main) se le pasa como dos
 *     punteros a función (DashCellFn), para no duplicar la topología aquí.
 *   · Los flags de fallo (ya debounced en el main) se pasan por frame en un
 *     DashFaults, para no acoplar la clase a los FaultTimer internos.
 *
 * ⚠ Cada línea son ~1,5 kB: a 115200 baud ocupa el Serial ~130 ms por frame.
 */
#pragma once

#include <Arduino.h>
#include <stdint.h>

// Tipos del proyecto — solo se usan por referencia aquí; el .cpp incluye los
// headers reales. Forward-declare para no acoplar el header a sus cabeceras.
class BQ79606;
class HallSensor;
class SocEstimator;
class FanController;

/// Getter de una celda/NTC de un módulo. module=0-based; idx=1-based.
///   cellV(m, n)  con n = 1..cellsPerMod  -> tensión de la celda (V)
///   cellT(m, k)  con k = 1..ntcPerMod    -> temperatura del NTC (°C)
using DashCellFn = float (*)(int module, int idx);

/// Configuración fija (se pasa una vez en el constructor).
struct DashConfig {
    int        numModules  = 10;      ///< módulos a enviar (= NUM_MODULES)
    int        cellsPerMod = 11;      ///< celdas por módulo
    int        ntcPerMod   = 9;       ///< NTC por módulo
    DashCellFn cellV       = nullptr; ///< mapeo V (p.ej. &modCellV del main)
    DashCellFn cellT       = nullptr; ///< mapeo T (p.ej. &modCellT del main)
    uint32_t   refreshMs   = 500;     ///< periodo de envío
    float      currentSign = 1.0f;    ///< signo de la corriente ENVIADA: +1 = descarga
                                      ///< positiva (coche); -1 = carga positiva (cargador).
                                      ///< Solo visualización: el Hall no se toca.
};

/// Estado de fallos del frame (ya confirmados/debounced en el main).
struct DashFaults {
    bool bmsOk  = true;   ///< BMS_OK = HIGH (true) / LOW (false)
    bool fV     = false;  ///< fallo de tensión confirmado
    bool fT     = false;  ///< fallo de temperatura confirmado
    bool fNtc   = false;  ///< NTC abierto confirmado
    bool fComm  = false;  ///< comms BQ caídas confirmado
    bool fHall  = false;  ///< fallo del amperímetro
    bool initOk = true;   ///< BQ inicializado OK
};

class TelemetryDashboard {
public:
    TelemetryDashboard(Stream& io, BQ79606& bms, HallSensor& hall,
                       SocEstimator& soc, FanController& fan,
                       const DashConfig& cfg);

    /// Activa/desactiva el envío (llamar al pulsar 'j').
    void toggle() { active_ = !active_; lastMs_ = 0; }

    /// ¿Está enviando ahora mismo?
    bool active() const { return active_; }

    /**
     * @brief Hook opcional para añadir campos propios al JSON.
     *
     * Se llama al final de render_(), justo antes de cerrar el objeto, con
     * una coma ya emitida. La función debe escribir pares `"clave":valor`
     * SIN llave de cierre ni coma final. Ej.:
     *     io.print("\"chg\":{\"on\":true}");
     *
     * Existe para que cada firmware publique lo suyo sin acoplar esta clase:
     * el charger añade el estado de carga (§ setJsonExtra en charger.cpp) y
     * main no registra nada, así que su JSON no cambia.
     */
    using DashExtraFn = void (*)(Stream& io);
    void setJsonExtra(DashExtraFn fn) { jsonExtra_ = fn; }

    /// Llamar cada loop. Si está activo y venció el periodo, envía una línea.
    /// Si está inactivo no hace nada (coste ~0).
    void update(const DashFaults& f);

private:
    void render_(const DashFaults& f);

    Stream&        io_;
    BQ79606&       bms_;
    HallSensor&    hall_;
    SocEstimator&  soc_;
    FanController& fan_;
    DashConfig     cfg_;
    bool           active_ = false;
    uint32_t       lastMs_ = 0;
    DashExtraFn    jsonExtra_ = nullptr;   ///< campos extra del firmware (ver setJsonExtra)
};
