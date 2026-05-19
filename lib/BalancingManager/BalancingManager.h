/**
 * @file    BalancingManager.h
 * @brief   Máquina de estados de balanceo para el BQ79606A-Q1.
 *
 * Encapsula TODA la lógica de balanceo, separándola del main:
 * cálculo de timers (proporcional al desequilibrio), guardias de
 * seguridad (T/V/NTC abierto), detección de fase, snapshot OCV y
 * re-evaluación entre ciclos.
 *
 * El main solo necesita:
 *   BalancingManager bal(bms, BAL_CFG);
 *   bal.enable() / bal.disable() / bal.tick();   // tick() en cada loop()
 *   + getters para mostrar estado (display.*).
 *
 * ── DISEÑO (refactor 2026-05-18) ─────────────────────────────────────────────
 * Plano tomado de la rama Pruebas-optimización, PERO con la lógica
 * VALIDADA EN HW de `main`:
 *   · Timer PROPORCIONAL al delta con tope/min (NO el modelo físico
 *     cellCapacity/rBal/timerSafety del blueprint — se descartó: no
 *     resuelve la lentitud, usa pendiente OCV lineal irreal).
 *   · Guardia de NTC abierto (bms.hasOpenNtc()) integrada.
 *   · phaseDuration_ms = 120000 (CB_CONFIG[DUTY]=2min real del
 *     datasheet; el blueprint tenía 60000, que era el bug "60s").
 * La capa de seguridad de celda/NTC (evalSafetyVoltage/Temperature)
 * sigue en el driver BQ79606 y la llama el main; aquí solo van las
 * guardias propias del balanceo.
 *
 * ── MÁQUINA DE ESTADOS ───────────────────────────────────────────────────────
 *
 *   IDLE ──[enable()]──► SETTLING ──[≥settle_ms reposo]──► MEASURING
 *                                                              │
 *                        SETTLING ◄──[CB_DONE todos]──────── RUNNING
 *                            │                                   ▲
 *                            └──[OCV medido, hay desequilibrio]──┘
 *
 *   · SETTLING:  FETs apagados. Espera ≥settle_ms para OCV real.
 *   · MEASURING: Lee OCV (isVoltageReadingReliable()==true). Calcula
 *                timer/celda ∝ delta. Sin desequilibrio → IDLE (fin).
 *   · RUNNING:   HW balanceando. Espera CB_DONE. Guardias T/V/NTC y
 *                timeout. Al acabar → SETTLING (re-evalúa).
 */

#pragma once
#include "BQ79606.h"

// ── Configuración del balanceo (single source of truth) ──────────────────────
// Valores por defecto = los VALIDADOS EN HW en `main` (2026-05-18).
struct BalConfig {
    float    threshold_V      = 0.003f;     ///< Delta mín (V) para que una celda necesite balanceo
    float    minVoltage_V     = 3.0f;       ///< V mín de celda durante balanceo (guardia)
    float    maxVoltage_V     = 4.15f;      ///< V máx de celda durante balanceo (guardia)
    float    minTemp_C        = 5.0f;       ///< T mín durante balanceo (guardia)
    float    maxTemp_C        = 45.0f;      ///< T máx durante balanceo (guardia)
    uint32_t settle_ms        = 5000;       ///< Reposo antes de medir OCV
    uint32_t timeout_ms       = 1800000;    ///< Timeout de seguridad del ciclo (30 min)
    uint32_t phaseDuration_ms = 120000;     ///< Fase de duty = CB_CONFIG[DUTY] = 2 min (SOLO display "Xs restantes")
    // Timer PROPORCIONAL al desequilibrio (conservador, validado en HW):
    //   timer_min = clamp( round(deltaMv * minPerMv), minTimerMin, maxTimerMin )
    float    minPerMv         = 0.8f;       ///< Min de balanceo activo por mV de delta (subir con cuidado)
    uint8_t  maxTimerMin      = 12;         ///< Tope/celda·ciclo (min). 2×este < timeout_ms
    uint8_t  minTimerMin      = 1;          ///< Mínimo si la celda necesita balanceo (min)
    uint8_t  maxCommFails     = 5;          ///< Fallos de comms CONSECUTIVOS (OCV/T) tolerados antes de abortar fail-safe. Afinar en HW vs tasa de CRC del bus de 20 ICs.

    /// Invariante de seguridad CRÍTICO: el reloj de pared del ciclo
    /// (≈2× el timer máx por el duty odd/even) debe caber dentro del
    /// timeout de seguridad, o cada ciclo lo cortaría el timeout antes
    /// de CB_DONE (balanceo nunca completa + aborto de seguridad
    /// disparado en operación normal). constexpr → comprobable en
    /// compilación para los defaults (static_assert abajo).
    constexpr bool timerFitsTimeout() const {
        return 2UL * maxTimerMin * 60000UL < timeout_ms;
    }

    /// Valida TODOS los invariantes (también configs hechas a mano en
    /// runtime). Si devuelve false y why!=nullptr, *why apunta a un
    /// literal estático con el primer invariante violado.
    bool isValid(const char** why = nullptr) const;
};

// Los defaults DEBEN cumplir el invariante crítico (atrapado en
// compilación; el resto y las configs custom las valida isValid()).
static_assert(BalConfig{}.timerFitsTimeout(),
              "BalConfig: 2*maxTimerMin*60000 >= timeout_ms "
              "(el ciclo no cabe en el timeout de seguridad)");

enum class BalState {
    IDLE,       ///< Balanceo desactivado
    SETTLING,   ///< FETs apagados, esperando OCV estable
    MEASURING,  ///< Midiendo OCV y calculando timers
    RUNNING     ///< HW balanceando, esperando CB_DONE
};

enum class BalStopReason {
    NONE,
    USER,           ///< Parado por el usuario (x / toggle B)
    EQUILIBRATED,   ///< Pack equilibrado (delta < threshold) → fin normal
    TIMEOUT,        ///< Timeout de seguridad del ciclo
    OVER_TEMP,      ///< Guardia T alta
    UNDER_TEMP,     ///< Guardia T baja
    OVER_VOLTAGE,   ///< Guardia V alta (snapshot OCV)
    UNDER_VOLTAGE,  ///< Guardia V baja (snapshot OCV)
    NTC_OPEN,       ///< NTC abierto/inválido detectado (bms.hasOpenNtc())
    COMM_ERROR,     ///< Error de comunicación arrancando el HW de balanceo
    CONFIG_INVALID, ///< BalConfig no pasa isValid() — balanceo no arranca
    COMM_LOST       ///< Bus BQ ilegible ≥maxCommFails ticks seguidos (OCV/T) → fail-safe
};

/**
 * @brief Máquina de estados de balanceo. Compone (no hereda) un BQ79606&.
 *
 * No toca la capa de seguridad de celda/NTC del driver (evalSafety*),
 * que sigue gestionando el main. Aquí solo: cálculo de timers,
 * guardias propias del balanceo y secuenciado de la SM.
 */
class BalancingManager {
public:
    BalancingManager(BQ79606& bms, const BalConfig& cfg);

    void enable();    ///< Arranca la sesión de balanceo (→ SETTLING)
    void disable();   ///< Para y vuelve a IDLE (parada por usuario)
    void tick();      ///< Llamar en CADA loop(). No hace nada si IDLE.

    // ── Getters (para display.* / diagnóstico) ──────────────────────────────
    BalState      getState()             const { return _state; }
    bool          isEnabled()            const { return _enabled; }
    bool          isHwRunning()          const { return _bms.isBalHwRunning(); }
    bool          isOcvValid()           const { return _ocvValid; }
    BalStopReason getLastReason()        const { return _lastReason; }
    int           getTripBoard()         const { return _tripBoard; }  ///< Board que disparó la última parada; -1 = no localizado
    int           getTripCell()          const { return _tripCell; }   ///< Celda 0-based; -1 = no localizado
    unsigned long getStateElapsedMs()    const { return millis() - _stateTimer; }
    unsigned long getSettleRemainingMs() const;
    int           getPhaseRemainingSec() const;
    float         getOcvSnapshot(uint8_t board, uint8_t cell) const;
    float         getThreshold()         const { return _cfg.threshold_V; }
    static const char* reasonStr(BalStopReason r);

private:
    BQ79606&  _bms;
    BalConfig _cfg;

    BalState      _state      = BalState::IDLE;
    bool          _enabled    = false;
    BalStopReason _lastReason = BalStopReason::NONE;
    int8_t        _tripBoard  = -1;   ///< Board que disparó la parada (-1 = no localizado)
    int8_t        _tripCell   = -1;   ///< Celda 0-based de la parada (-1 = no localizado)

    uint8_t       _commFails  = 0;   ///< Fallos de comms CONSECUTIVOS (política compartida OCV/T)

    unsigned long _stateTimer = 0;   ///< Inicio del estado actual (ms)
    unsigned long _phaseStart = 0;   ///< Inicio de la fase de duty actual (display)
    uint8_t       _prevMask   = 0xFF;///< Máscara de balanceo previa (detección de cambio de fase)

    // Snapshot OCV (FETs apagados, fiable) usado en RUNNING para la
    // guardia de V y para el resumen del ciclo.
    float _ocvSnapshot[TOTALBOARDS][6] = {};
    bool  _ocvValid                    = false;
    float _ocvInitial[TOTALBOARDS][6]  = {};
    bool  _ocvInitialValid             = false;

    void _stopAndReset(BalStopReason reason);   ///< stopAllBalancing + → IDLE + razón
    bool _checkGuards();                         ///< T/V/NTC; true si paró
    /// Política de fallo de comms compartida (OCV en MEASURING, T en
    /// RUNNING). Actualiza _commFails: éxito lo resetea, fallo lo
    /// incrementa. Devuelve true si se alcanzó _cfg.maxCommFails
    /// fallos CONSECUTIVOS → el caller debe abortar fail-safe.
    bool _commFault(bool readOk);
    bool _computeTimers(uint8_t timers[TOTALBOARDS][6]); ///< proporcional; true si hay trabajo
    void _updateOcvSnapshot();
    void _printOCV()   const;
    void _printTimers(const uint8_t timers[TOTALBOARDS][6]) const;
    void _printCycleResult() const;
    static int _nCells(int b) { return CELLS_FOR_BOARD(b); }  ///< modelo even/odd de main
};
