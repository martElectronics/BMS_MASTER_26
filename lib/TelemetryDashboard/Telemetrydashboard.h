/**
 * @file    TelemetryDashboard.h
 * @brief   Dashboard de telemetría EN VIVO sobre el Serial existente (ANSI).
 *
 * En vez de volcar líneas que hacen scroll, redibuja en el sitio una pantalla
 * fija (rejilla V + rejilla T + resumen + fallos) usando secuencias ANSI, de
 * forma que se ve como un panel que se refresca en tiempo real. No necesita
 * hardware extra: usa el mismo puerto Serial.
 *
 * ── Uso ──────────────────────────────────────────────────────────────────
 *   · Está DESACTIVADO por defecto (no interfiere con los comandos serie).
 *   · Se activa/desactiva con la tecla 'm'. Cualquier tecla lo cierra.
 *   · Requiere un terminal con soporte ANSI (pio monitor, minicom, PuTTY,
 *     screen…) y ~34 filas de alto (maximizar la ventana).
 *
 * ── Diseño ───────────────────────────────────────────────────────────────
 *   · Clase independiente: solo LEE los getters ya cacheados del BQ / Hall /
 *     SOC / Fan (NO dispara lecturas nuevas — de eso se ocupa el main).
 *   · El mapeo módulo→celda/NTC (que vive en el main) se le pasa como dos
 *     punteros a función (DashCellFn), para no duplicar la topología aquí.
 *   · Los flags de fallo (ya debounced en el main) se pasan por frame en un
 *     DashFaults, para no acoplar la clase a los FaultTimer internos.
 *
 * ⚠ El render bloquea el Serial ~150-200 ms por frame a 115200 baud. Es una
 *   herramienta de banco/diagnóstico: tenerla ON retrasa ligeramente el
 *   muestreo. Está OFF por defecto → la operación normal no se ve afectada.
 *   Para un refresco más fino, subir el baudrate del Serial (p.ej. 921600).
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
    int        numModules  = 10;      ///< módulos a pintar (= NUM_MODULES)
    int        cellsPerMod = 11;      ///< celdas por módulo
    int        ntcPerMod   = 9;       ///< NTC por módulo
    DashCellFn cellV       = nullptr; ///< mapeo V (p.ej. &modCellV del main)
    DashCellFn cellT       = nullptr; ///< mapeo T (p.ej. &modCellT del main)
    float      uvV = 2.8f;            ///< umbral UV para colorear
    float      ovV = 4.2f;            ///< umbral OV para colorear
    float      utC = -20.0f;          ///< umbral UT para colorear
    float      otC = 60.0f;           ///< umbral OT para colorear
    uint32_t   refreshMs   = 500;     ///< periodo de refresco del panel
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

    /// Activa/desactiva el panel (llamar al pulsar 'm').
    void toggle();

    /// ¿Está el panel activo ahora mismo?
    bool active() const { return active_; }

    /// Modo JSON para dashboard web
    void setJsonMode(bool json) { jsonMode_ = json; }
    bool isJsonMode() const { return jsonMode_; }

    /**
     * @brief Hook opcional para añadir campos propios al JSON.
     *
     * Se llama al final de renderJson_(), justo antes de cerrar el objeto, con
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

    /// Llamar cada loop. Si está activo y venció el periodo, redibuja.
    /// Si está inactivo no hace nada (coste ~0).
    void update(const DashFaults& f);

private:
    void enter_();
    void leave_();
    void render_(const DashFaults& f);
    void renderJson_(const DashFaults& f);
    void sepLine_();

    Stream&        io_;
    BQ79606&       bms_;
    HallSensor&    hall_;
    SocEstimator&  soc_;
    FanController& fan_;
    DashConfig     cfg_;
    bool           active_ = false;
    bool           jsonMode_ = false;
    uint32_t       lastMs_ = 0;
    DashExtraFn    jsonExtra_ = nullptr;   ///< campos extra del firmware (ver setJsonExtra)
};