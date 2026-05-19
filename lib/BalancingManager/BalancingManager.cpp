/**
 * @file    BalancingManager.cpp
 * @brief   Implementación de la máquina de estados de balanceo.
 *
 * Lógica VALIDADA EN HW (rama main, 20 ICs) extraída tal cual del
 * main: timer proporcional al delta, guardias T/V/NTC y secuenciado
 * SETTLING→MEASURING→RUNNING→SETTLING. Los #define del main pasan a
 * ser campos de BalConfig (_cfg). Sin cambios de comportamiento.
 */

#include "BalancingManager.h"
#include <math.h>
#include <string.h>

// ── Validación de configuración ──────────────────────────────────────────────

bool BalConfig::isValid(const char** why) const
{
    const char* err = nullptr;

    if      (minVoltage_V >= maxVoltage_V)        err = "minVoltage_V >= maxVoltage_V";
    else if (minTemp_C    >= maxTemp_C)           err = "minTemp_C >= maxTemp_C";
    else if (threshold_V  <= 0.0f)                err = "threshold_V <= 0";
    else if (minPerMv     <= 0.0f)                err = "minPerMv <= 0";
    else if (minTimerMin  <  1)                   err = "minTimerMin < 1";
    else if (minTimerMin  >  maxTimerMin)         err = "minTimerMin > maxTimerMin";
    else if (settle_ms    == 0)                   err = "settle_ms == 0";
    else if (timeout_ms   == 0)                   err = "timeout_ms == 0";
    else if (phaseDuration_ms == 0)               err = "phaseDuration_ms == 0";
    else if (maxCommFails < 1)                    err = "maxCommFails < 1 (abortaria sin tolerar glitches)";
    else if (!timerFitsTimeout())                 err = "2*maxTimerMin*60000 >= timeout_ms "
                                                        "(ciclo no cabe en el timeout)";

    if (why) *why = err;
    return err == nullptr;
}

BalancingManager::BalancingManager(BQ79606& bms, const BalConfig& cfg)
    : _bms(bms), _cfg(cfg) {}

// ── Control de sesión ────────────────────────────────────────────────────────

void BalancingManager::enable()
{
    // Fail-safe: una config rota NO debe arrancar el balanceo. Atrapa
    // también configs construidas a mano (el static_assert del header
    // solo cubre los defaults).
    const char* why = nullptr;
    if (!_cfg.isValid(&why)) {
        Serial.printf("[BAL] CONFIG INVALIDA: %s. Balanceo NO arranca.\n",
                      why ? why : "?");
        _lastReason = BalStopReason::CONFIG_INVALID;
        _tripBoard  = -1;
        _tripCell   = -1;
        _enabled    = false;
        _state      = BalState::IDLE;
        return;
    }

    _bms.stopAllBalancing();
    _stateTimer      = millis();
    _ocvValid        = false;
    _ocvInitialValid = false;
    _prevMask        = 0xFF;
    _commFails       = 0;
    _lastReason      = BalStopReason::NONE;
    _tripBoard       = -1;
    _tripCell        = -1;
    _enabled         = true;
    _state           = BalState::SETTLING;
    Serial.printf("BAL: activado -> SETTLING (%lus)...\n", _cfg.settle_ms / 1000);
}

void BalancingManager::disable()
{
    _bms.stopAllBalancing();
    _ocvValid        = false;
    _ocvInitialValid = false;
    _state           = BalState::IDLE;
    _enabled         = false;
    _lastReason      = BalStopReason::USER;
    _tripBoard       = -1;
    _tripCell        = -1;
    // No imprime: el texto depende del comando (B vs x). Lo pone el caller.
}

void BalancingManager::_stopAndReset(BalStopReason reason)
{
    _bms.stopAllBalancing();
    _ocvValid        = false;
    _ocvInitialValid = false;
    _state           = BalState::IDLE;
    _enabled         = false;
    _lastReason      = reason;
    // Por defecto no localizado; el caller que sepa la celda lo fija
    // DESPUÉS de esta llamada (p.ej. la guardia de V en RUNNING).
    _tripBoard       = -1;
    _tripCell        = -1;
}

// ── Política de fallo de comms compartida (OCV/T) ────────────────────────────
// Un éxito resetea el contador (tolera glitches CRC aislados del bus,
// frecuentes en una cadena de 20 ICs). maxCommFails fallos CONSECUTIVOS
// → true: el caller aborta fail-safe. Como la SM solo hace una lectura
// guardada por tick (OCV en MEASURING, T en RUNNING) y toda transición
// de estado pasa por un éxito que lo resetea, el contador es de facto
// por-episodio aunque sea compartido.
bool BalancingManager::_commFault(bool readOk)
{
    if (readOk) { _commFails = 0; return false; }
    if (_commFails < 255) _commFails++;
    return _commFails >= _cfg.maxCommFails;
}

// ── Guardia global (cada tick, valores cacheados) ────────────────────────────

bool BalancingManager::_checkGuards()
{
    const float tMax = _bms.getMaxTemp();
    const float tMin = _bms.getMinTemp();
    const float vMax = _bms.getMaxVoltage();
    const float vMin = _bms.getMinVoltage();

    const char*   reason = nullptr;
    BalStopReason r      = BalStopReason::NONE;

    if      (_bms.hasOpenNtc())          { reason = "NTC abierto"; r = BalStopReason::NTC_OPEN; }
    else if (tMax > _cfg.maxTemp_C)      { reason = "T alta";  r = BalStopReason::OVER_TEMP; }
    else if (tMin < _cfg.minTemp_C)      { reason = "T baja";  r = BalStopReason::UNDER_TEMP; }
    else if (vMax > _cfg.maxVoltage_V)   { reason = "V alta";  r = BalStopReason::OVER_VOLTAGE; }
    else if (vMin < _cfg.minVoltage_V)   { reason = "V baja";  r = BalStopReason::UNDER_VOLTAGE; }

    if (!reason) return false;

    Serial.printf("BAL: parado — %s (T=%.1f/%.1fC V=%.3f/%.3fV)\n",
                  reason, tMin, tMax, vMin, vMax);
    _stopAndReset(r);
    return true;
}

// ── Timer proporcional al desequilibrio ──────────────────────────────────────

bool BalancingManager::_computeTimers(uint8_t timers[TOTALBOARDS][6])
{
    // Una celda necesita balanceo si supera al mínimo del pack en más de
    // _cfg.threshold_V. timer ∝ delta, ganancia conservadora (sin overshoot),
    // con tope y mínimo. Quedarse corto se auto-corrige al re-medir OCV.
    bool anyNeeds = false;

    float vMinPack = 9999.0f;
    for (int b = 0; b < TOTALBOARDS; b++) {
        int nCells = _nCells(b);
        for (int c = 0; c < nCells; c++) {
            float v = _bms.getVoltage(b, c);
            if (v > _cfg.minVoltage_V && v < vMinPack) vMinPack = v;
        }
    }
    if (vMinPack > 9000.0f) return false;

    for (int b = 0; b < TOTALBOARDS; b++) {
        int nCells = _nCells(b);
        for (int c = 0; c < 6; c++) {
            if (c >= nCells || _bms.getVoltage(b, c) <= _cfg.minVoltage_V) {
                timers[b][c] = 0;
                continue;
            }
            float deltaMv = (_bms.getVoltage(b, c) - vMinPack) * 1000.0f;
            if (deltaMv > (_cfg.threshold_V * 1000.0f)) {
                int t = (int)lroundf(deltaMv * _cfg.minPerMv);
                if (t < _cfg.minTimerMin) t = _cfg.minTimerMin;
                if (t > _cfg.maxTimerMin) t = _cfg.maxTimerMin;
                timers[b][c] = (uint8_t)t;
                anyNeeds = true;
            } else {
                timers[b][c] = 0;
            }
        }
    }
    return anyNeeds;
}

void BalancingManager::_updateOcvSnapshot()
{
    for (int b = 0; b < TOTALBOARDS; b++)
        for (int c = 0; c < 6; c++)
            _ocvSnapshot[b][c] = _bms.getVoltage(b, c);
    _ocvValid = true;

    // Snapshot inicial: solo la primera vez (no en re-mediciones).
    if (!_ocvInitialValid) {
        for (int b = 0; b < TOTALBOARDS; b++)
            for (int c = 0; c < 6; c++)
                _ocvInitial[b][c] = _bms.getVoltage(b, c);
        _ocvInitialValid = true;
    }
}

// ── Tick de la máquina de estados ────────────────────────────────────────────

void BalancingManager::tick()
{
    if (!_enabled || _state == BalState::IDLE) return;

    if (_checkGuards()) return;   // guardia global por-tick

    unsigned long now = millis();

    switch (_state) {

    case BalState::SETTLING:
        // FETs apagados — esperar reposo para OCV real
        if ((now - _stateTimer) >= _cfg.settle_ms) {
            _state = BalState::MEASURING;
            Serial.println(F("BAL: reposo OK -> MEASURING"));
        }
        break;

    case BalState::MEASURING: {
        // Guardia de arranque (mensaje descriptivo de por qué no arranca)
        {
            const char*   reason = nullptr;
            BalStopReason r      = BalStopReason::NONE;
            float         val    = 0;
            if      (_bms.hasOpenNtc())                       { reason = "NTC abierto"; r = BalStopReason::NTC_OPEN;     val = _bms.getOpenNtcCount(); }
            else if (_bms.getMaxTemp()    > _cfg.maxTemp_C)   { reason = "T alta";  r = BalStopReason::OVER_TEMP;    val = _bms.getMaxTemp(); }
            else if (_bms.getMinTemp()    < _cfg.minTemp_C)   { reason = "T baja";  r = BalStopReason::UNDER_TEMP;   val = _bms.getMinTemp(); }
            else if (_bms.getMaxVoltage() > _cfg.maxVoltage_V){ reason = "V alta";  r = BalStopReason::OVER_VOLTAGE; val = _bms.getMaxVoltage(); }
            else if (_bms.getMinVoltage() < _cfg.minVoltage_V){ reason = "V baja";  r = BalStopReason::UNDER_VOLTAGE;val = _bms.getMinVoltage(); }
            if (reason) {
                Serial.printf("BAL: no arranca — %s (%.3f). Esperando...\n", reason, val);
                // Paridad con main validado en HW: aquí los FETs ya están
                // apagados → NO stopAllBalancing (evita tráfico SPI/serie
                // innecesario). _lastReason es solo estado interno (diag 'q').
                _lastReason = r;
                _tripBoard  = -1;   // guardia por agregado: no localiza celda
                _tripCell   = -1;
                _state      = BalState::IDLE;
                _enabled    = false;
                break;
            }
        }

        // Leer OCV — aquí isVoltageReadingReliable() es true.
        // Política de comms compartida: glitch aislado → reintenta;
        // maxCommFails seguidos → aborta fail-safe (no colgar en
        // MEASURING para siempre inundando el serie).
        BQResult res = _bms.readVoltages();
        if (_commFault(res == BQResult::OK)) {
            Serial.printf("BAL: OCV ilegible %u veces seguidas — abortando (comms).\n",
                          _commFails);
            _stopAndReset(BalStopReason::COMM_LOST);
            break;
        }
        if (res != BQResult::OK) {
            Serial.printf("BAL: error leyendo OCV (%u/%u), reintentando...\n",
                          _commFails, _cfg.maxCommFails);
            break;   // permanece en MEASURING, reintenta el próximo tick
        }

        _printOCV();

        uint8_t timers[TOTALBOARDS][6];
        memset(timers, 0, sizeof(timers));

        if (!_computeTimers(timers)) {
            Serial.println(F("BAL: equilibrado (delta<umbral). Fin."));
            // Paridad con main validado: FETs ya apagados → sin
            // stopAllBalancing. Fin normal de sesión.
            _lastReason = BalStopReason::EQUILIBRATED;
            _tripBoard  = -1;   // fin normal, no es una parada localizada
            _tripCell   = -1;
            _state      = BalState::IDLE;
            _enabled    = false;
            break;
        }

        _printTimers(timers);

        // Snapshot OCV antes de arrancar el hardware
        _updateOcvSnapshot();

        // Arrancar HW por board.
        // ⚠ CB_CONFIG=0x0A alterna odds (1,3,5) / evens (2,4,6) en fases de
        // 2min — el IC NUNCA activa dos celdas adyacentes a la vez.
        bool ok        = true;
        int  failBoard = -1;
        for (int b = 0; b < TOTALBOARDS; b++) {
            uint8_t maskOdds = 0, maskEvens = 0;
            for (int c = 0; c < 6; c++) {
                if (timers[b][c] > 0) {
                    if (c % 2 == 0) maskOdds  |= (1 << c);
                    else            maskEvens |= (1 << c);
                }
            }
            uint8_t maskCombined = maskOdds | maskEvens;
            if (maskCombined != 0) {
                Serial.printf("BAL: B%d odds=0x%02X evens=0x%02X combined=0x%02X\n",
                              b, maskOdds, maskEvens, maskCombined);
                for (int c = 0; c < 6; c++)
                    Serial.printf("  C%d=%dmin ", c + 1, timers[b][c]);
                Serial.println();
                if (_bms.startBalancingTimers(b, timers[b]) != BQResult::OK) {
                    Serial.print(F("BAL: error board ")); Serial.println(b);
                    ok = false;
                    if (failBoard < 0) failBoard = b;   // primer board que falló
                }
            }
        }

        if (ok) {
            _stateTimer = now;
            _state      = BalState::RUNNING;
            Serial.printf("BAL: HW activo -> RUNNING (%s)\n",
                          _ocvInitialValid ? "ciclo" : "primer ciclo");
        } else {
            _stopAndReset(BalStopReason::COMM_ERROR);
            _tripBoard = (int8_t)failBoard;   // tras _stopAndReset (que pone -1)
        }
        break;
    }

    case BalState::RUNNING: {
        // Timeout de seguridad del ciclo
        if ((now - _stateTimer) > _cfg.timeout_ms) {
            Serial.printf("BAL: timeout %lumin. Parando.\n", _cfg.timeout_ms / 60000);
            _stopAndReset(BalStopReason::TIMEOUT);
            break;
        }

        // Temperatura fresca dentro del tick (no se ve afectada por los FETs).
        // El balanceo DISIPA CALOR: perder visibilidad térmica mientras
        // se balancea es justo cuando hay que fallar seguro. Glitch
        // aislado → se tolera (sigue con T cacheada, debounce); pero
        // maxCommFails seguidos → abortar (no balancear a ciegas).
        if (_commFault(_bms.readTemperatures() == BQResult::OK)) {
            Serial.printf("BAL: T ilegible %u veces seguidas — abortando (comms).\n",
                          _commFails);
            _stopAndReset(BalStopReason::COMM_LOST);
            break;
        }

        bool          guardStop   = false;
        const char*   guardReason = nullptr;
        BalStopReason r           = BalStopReason::NONE;
        int           tb = -1, tc = -1;   // board/celda que dispara (V); -1 = no localiza

        if      (_bms.hasOpenNtc())                  { guardStop = true; guardReason = "NTC abierto"; r = BalStopReason::NTC_OPEN; }
        else if (_bms.getMaxTemp() > _cfg.maxTemp_C) { guardStop = true; guardReason = "T alta";  r = BalStopReason::OVER_TEMP; }
        else if (_bms.getMinTemp() < _cfg.minTemp_C) { guardStop = true; guardReason = "T baja";  r = BalStopReason::UNDER_TEMP; }

        // Voltaje vía snapshot OCV — más fiable con FETs activos
        if (!guardStop && _ocvValid) {
            for (int b = 0; b < TOTALBOARDS && !guardStop; b++) {
                int nCells = _nCells(b);
                for (int c = 0; c < nCells && !guardStop; c++) {
                    if      (_ocvSnapshot[b][c] < _cfg.minVoltage_V) { guardStop = true; guardReason = "V baja"; r = BalStopReason::UNDER_VOLTAGE; tb = b; tc = c; }
                    else if (_ocvSnapshot[b][c] > _cfg.maxVoltage_V) { guardStop = true; guardReason = "V alta"; r = BalStopReason::OVER_VOLTAGE; tb = b; tc = c; }
                }
            }
        }

        if (guardStop) {
            if (tc >= 0)   // parada localizada (V): incluir B/C en el aviso
                Serial.printf("BAL: PARADO — %s B%d C%d (T=%.1f/%.1fC Vmin=%.3fV Vmax=%.3fV)\n",
                              guardReason, tb, tc + 1,
                              _bms.getMinTemp(), _bms.getMaxTemp(),
                              _bms.getMinVoltage(), _bms.getMaxVoltage());
            else
                Serial.printf("BAL: PARADO — %s (T=%.1f/%.1fC Vmin=%.3fV Vmax=%.3fV)\n",
                              guardReason,
                              _bms.getMinTemp(), _bms.getMaxTemp(),
                              _bms.getMinVoltage(), _bms.getMaxVoltage());
            _stopAndReset(r);
            if (tc >= 0) { _tripBoard = (int8_t)tb; _tripCell = (int8_t)tc; }  // tras _stopAndReset
            break;
        }

        // Detección de cambio de fase (odd↔even) para el contador de display.
        {
            uint8_t maskNow = 0;
            for (int b = 0; b < TOTALBOARDS; b++)
                maskNow |= _bms.getBalancingMask(b);
            if (maskNow != 0 && maskNow != _prevMask)
                _phaseStart = now;
            _prevMask = maskNow;
        }

        // CB_DONE en todos los boards → fin de ciclo, re-evaluar
        bool allDone = true;
        for (int b = 0; b < TOTALBOARDS; b++)
            if (!_bms.isBalancingDone(b)) { allDone = false; break; }

        if (allDone) {
            unsigned long cicloSeg = (now - _stateTimer) / 1000;
            Serial.printf("BAL: ciclo completado (%lus) — parando para re-evaluar...\n",
                          cicloSeg);
            _bms.stopAllBalancing();

            if (_bms.readVoltages() == BQResult::OK && _ocvInitialValid)
                _printCycleResult();

            Serial.println(F("BAL: esperando reposo antes de re-medir OCV..."));
            _ocvValid        = false;
            _ocvInitialValid = false;
            _stateTimer      = now;
            _state           = BalState::SETTLING;   // NO se deshabilita: re-evalúa
        }
        break;
    }

    default: break;
    }
}

// ── Getters de display ───────────────────────────────────────────────────────

unsigned long BalancingManager::getSettleRemainingMs() const
{
    long rem = (long)_cfg.settle_ms - (long)(millis() - _stateTimer);
    return rem < 0 ? 0UL : (unsigned long)rem;
}

int BalancingManager::getPhaseRemainingSec() const
{
    unsigned long elapsed = millis() - _phaseStart;
    unsigned long cap     = (elapsed < _cfg.phaseDuration_ms)
                                ? elapsed : _cfg.phaseDuration_ms;
    return (int)((_cfg.phaseDuration_ms - cap) / 1000);
}

float BalancingManager::getOcvSnapshot(uint8_t board, uint8_t cell) const
{
    if (board >= TOTALBOARDS || cell >= 6) return 0.0f;
    return _ocvSnapshot[board][cell];
}

const char* BalancingManager::reasonStr(BalStopReason r)
{
    switch (r) {
        case BalStopReason::NONE:          return "—";
        case BalStopReason::USER:          return "usuario";
        case BalStopReason::EQUILIBRATED:  return "equilibrado";
        case BalStopReason::TIMEOUT:       return "timeout";
        case BalStopReason::OVER_TEMP:     return "T alta";
        case BalStopReason::UNDER_TEMP:    return "T baja";
        case BalStopReason::OVER_VOLTAGE:  return "V alta";
        case BalStopReason::UNDER_VOLTAGE: return "V baja";
        case BalStopReason::NTC_OPEN:      return "NTC abierto";
        case BalStopReason::COMM_ERROR:    return "error comunicacion (arranque HW)";
        case BalStopReason::CONFIG_INVALID:return "config invalida";
        case BalStopReason::COMM_LOST:     return "bus BQ perdido (OCV/T)";
    }
    return "?";
}

// ── Impresión de diagnóstico ─────────────────────────────────────────────────

void BalancingManager::_printOCV() const
{
    Serial.println(F("BAL: OCV:"));
    for (int b = 0; b < TOTALBOARDS; b++) {
        int nCells = _nCells(b);
        Serial.print(F("  B")); Serial.print(b); Serial.print(F(": "));
        for (int c = 0; c < nCells; c++) {
            Serial.print(_bms.getVoltage(b, c), 3); Serial.print(F("V "));
        }
        Serial.println();
    }
    Serial.print(F("  Delta="));
    Serial.print(_bms.getVoltageDelta(), 1);
    Serial.println(F("mV"));
}

void BalancingManager::_printTimers(const uint8_t timers[TOTALBOARDS][6]) const
{
    Serial.println(F("BAL: timers (min):"));
    for (int b = 0; b < TOTALBOARDS; b++) {
        int nCells = _nCells(b);
        Serial.print(F("  B")); Serial.print(b); Serial.print(F(": "));
        for (int c = 0; c < nCells; c++) {
            Serial.print(F("C")); Serial.print(c + 1);
            Serial.print(F("=")); Serial.print(timers[b][c]); Serial.print(F(" "));
        }
        Serial.println();
    }
}

void BalancingManager::_printCycleResult() const
{
    Serial.println(F("BAL: resultado ciclo (inicial -> actual):"));
    for (int b = 0; b < TOTALBOARDS; b++) {
        int nCells = _nCells(b);
        Serial.printf("  B%d:", b);
        bool anyChange = false;
        for (int c = 0; c < nCells; c++) {
            float diff = (_ocvInitial[b][c] - _bms.getVoltage(b, c)) * 1000.0f;
            if (fabsf(diff) >= 0.5f) {
                Serial.printf(" C%d:%.3f->%.3fV (-%dmV)",
                              c + 1, _ocvInitial[b][c], _bms.getVoltage(b, c),
                              (int)diff);
                anyChange = true;
            }
        }
        if (!anyChange) Serial.print(F(" sin cambio"));
        Serial.println();
    }
}
