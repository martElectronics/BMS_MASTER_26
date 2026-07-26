/**
 * @file    HallSensor.cpp
 * @brief   Implementación del driver para sensor Hall DHAB S/118.
 *
 * Correcciones vs. versión original (subida desde el BMS antiguo):
 *   1. begin() fuerza analogReadResolution(12) — STM32duino es 10-bit
 *      por defecto; sin esto las lecturas salían ~4× mal. [CRÍTICO]
 *   2. "Stuck" se evalúa sobre el dither RAW del ADC del canal activo,
 *      no sobre ΔI (antes daba falso stuck con corriente real estable).
 *   3. "Noisy" usa umbral propio (HALL_WD_NOISE_DELTA_A), desacoplado
 *      de la histéresis de rango.
 *   4. Desconexión detecta también el fallo de UN solo canal (raíl del
 *      canal activo sin corroboración del otro).
 *   5. El watchdog ignora el ciclo de cambio de rango (discontinuidad
 *      de canal) para no generar falso stuck/noisy.
 *   6. La protección de sobre-I usa la corriente SIN filtrar (no comerse
 *      el presupuesto de 500ms de EV5.8 con el lag del EMA).
 * Umbrales [TUNE] pendientes de ajuste con el ruido real del HW.
 */

#include "HallSensor.h"

// ============================================================================
//  CICLO DE VIDA
// ============================================================================

void HallSensor::begin()
{
    // CRÍTICO para STM32: analogRead() es 10-bit por defecto en STM32duino
    // (ESP32 es 12-bit). Forzar 12-bit para que HALL_ADC_RES=4095 sea válido.
    analogReadResolution(12);

    pinMode(_pin30A,  INPUT);
    pinMode(_pin350A, INPUT);

    Serial.println(F("[HALL] Calibrando offset... (manten el vehiculo en reposo)"));

    float sum30  = 0.0f;
    float sum350 = 0.0f;

    for (int i = 0; i < HALL_CALIB_SAMPLES; i++) {
        int raw30  = analogRead(_pin30A);
        int raw350 = analogRead(_pin350A);

        sum30  += (raw30  * HALL_VREF) / HALL_ADC_RES;
        sum350 += (raw350 * HALL_VREF) / HALL_ADC_RES;

        delayMicroseconds(HALL_CALIB_DELAY_US);
    }

    _offset30Init  = sum30  / HALL_CALIB_SAMPLES;
    _offset350Init = sum350 / HALL_CALIB_SAMPLES;

    _offsetDyn30  = _offset30Init;
    _offsetDyn350 = _offset350Init;

    // Inicializar watchdog: timestamp actual + raw activo, para no generar
    // un falso "stuck" en el primer arranque.
    _lastChangeTime = micros();
    _lastUpdateUs   = micros();   // para la normalización dt de los EMA
    _lastActiveRaw  = analogRead(_usingLowRange ? _pin30A : _pin350A);

    Serial.printf("[HALL] Offset 30A=%.4fV  Offset 350A=%.4fV\n",
                  _offset30Init, _offset350Init);

    // Sanity check: a 0 A el offset debe caer cerca de Vcc/2 × divisor (~1.62V).
    // Fuera de ventana = sensor desconectado/railado/mal alimentado al arrancar.
    // isOK() devolverá false hasta que se recalibre con el sensor sano (reinit).
    _offsetValid =
        (_offset30Init  > HALL_OFFSET_MIN_V && _offset30Init  < HALL_OFFSET_MAX_V) &&
        (_offset350Init > HALL_OFFSET_MIN_V && _offset350Init < HALL_OFFSET_MAX_V);

    if (_offsetValid) {
        Serial.println(F("[HALL] Calibracion completada."));
    } else {
        Serial.println(F("[HALL] *** WARN: offset fuera de ~1.62V -> sensor "
                         "desconectado/mal al boot. isOK()=false hasta reinit. ***"));
    }
}


// ============================================================================
//  UPDATE — llamar en cada ciclo del loop()
// ============================================================================

void HallSensor::update()
{
    // dt desde la última update() (us → s) para normalizar los EMA. micros()
    // es unsigned → la resta es correcta aun con wrap (~71 min).
    unsigned long nowUs = micros();
    float dt_s = (nowUs - _lastUpdateUs) / 1.0e6f;
    _lastUpdateUs = nowUs;

    // ── 1. Leer ADC ──────────────────────────────────────────────────────────
    int raw30  = analogRead(_pin30A);
    int raw350 = analogRead(_pin350A);

    // ── 2. Detección de saturación ADC ───────────────────────────────────────
    _adcSat30  = (raw30  > (HALL_ADC_RES - HALL_ADC_SAT_MARGIN)) ||
                 (raw30  < HALL_ADC_SAT_MARGIN);
    _adcSat350 = (raw350 > (HALL_ADC_RES - HALL_ADC_SAT_MARGIN)) ||
                 (raw350 < HALL_ADC_SAT_MARGIN);

    // ── 3. Convertir a voltaje ────────────────────────────────────────────────
    float v30  = (raw30  * HALL_VREF) / HALL_ADC_RES;
    float v350 = (raw350 * HALL_VREF) / HALL_ADC_RES;

    // ── 4. Aplicar offset dinámico ────────────────────────────────────────────
    float v30_corr  = v30  - _offsetDyn30;
    float v350_corr = v350 - _offsetDyn350;

    // ── 5. Convertir a corriente ──────────────────────────────────────────────
    float i30  = v30_corr  / HALL_SENS_30A;
    float i350 = v350_corr / HALL_SENS_350A;

    // ── 6. Selección de rango con histéresis ──────────────────────────────────
    bool prevLowRange = _usingLowRange;
    if (_usingLowRange) {
        if (_adcSat30 || fabsf(i30) > HALL_THRESH_UP) {
            _usingLowRange = false;
        }
    } else {
        // No volver al rango 30A si ESE canal está railado (p.ej. desconectado):
        // el !_adcSat30 evita el ping-pong de rango que rompía el debounce de
        // desconexión (un canal 30A roto saturaba → cambiaba a 350A → volvía a
        // 30A → ... y el flag D nunca aguantaba los 500 ms). Bug de banco.
        if (!_adcSat350 && !_adcSat30 && fabsf(i350) < HALL_THRESH_DOWN) {
            _usingLowRange = true;
        }
    }
    _rangeJustChanged = (prevLowRange != _usingLowRange);

    _currentRaw = _usingLowRange ? i30 : i350;

    // ── 7. Watchdog del sensor ────────────────────────────────────────────────
    _updateWatchdog(raw30, raw350, i30, i350);

    // ── 8. Offset dinámico ────────────────────────────────────────────────────
    // Solo se adapta con corriente baja y sensor sano.
    if (fabsf(_currentRaw) < HALL_ZERO_THRESHOLD && !_sensorDisconnected) {
        float aZero = dt_s / HALL_ZERO_TAU_S;
        if (aZero > 1.0f) aZero = 1.0f;
        _offsetDyn30  += aZero * (v30  - _offsetDyn30);
        _offsetDyn350 += aZero * (v350 - _offsetDyn350);
    }

    // ── 9. Filtro EMA (solo para la salida pública / display) ─────────────────
    // alpha normalizada por dt → filtrado independiente de la frecuencia del loop.
    // Si el sensor está desconectado (raw railado) se CONGELA (último valor bueno)
    // para no arrastrar la telemetría al rail durante el latcheo del fallo.
    if (!_sensorDisconnected) {
        float aFilt = dt_s / HALL_FILTER_TAU_S;
        if (aFilt > 1.0f) aFilt = 1.0f;
        _currentFiltered += aFilt * (_currentRaw - _currentFiltered);
    }

    // ── 10. Evaluar timers de fallo ───────────────────────────────────────────
    _updateFaultTimers();
}


// ============================================================================
//  WATCHDOG INTERNO
// ============================================================================

void HallSensor::_updateWatchdog(int raw30, int raw350, float i30, float i350)
{
    unsigned long now = micros();

    const int activeRaw = _usingLowRange ? raw30 : raw350;

    // ── Desconexión (robusta, sin depender del canal activo ni del ruido) ──────
    // Hardware: un canal DESCONECTADO se va a ~0 cuentas (el 33k del divisor
    // tira el pin a GND). En ESTA aplicación ninguna corriente legítima lleva un
    // canal tan bajo (carga máx -7 A → raw ~1640; un raíl bajo implicaría
    // < -38 A en el canal 30A o < -600 A en el 350A). Por eso:
    //   railado BAJO = cable roto, SIN corroboración → inmune al ruido.
    // (El canal 350A es de baja resolución: su corriente calculada salta ±15 A,
    //  lo que con la corroboración antigua hacía parpadear D y rompía el
    //  debounce de 500 ms. Bug encontrado en banco.)
    const bool ch30Low   = (raw30  < HALL_WD_ADC_MIN);
    const bool ch350Low  = (raw350 < HALL_WD_ADC_MIN);
    const bool ch30High  = (raw30  > HALL_WD_ADC_MAX);
    const bool ch350High = (raw350 > HALL_WD_ADC_MAX);

    // Railado ALTO sí puede ser descarga legítima (el canal 30A satura por
    // arriba a >~39 A, antes de cambiar al de 350A). Solo es desconexión (corto
    // a Vcc) si el OTRO canal NO corrobora una corriente alta real.
    const bool ch30HighDisc  = ch30High  && (i350 < HALL_WD_DISC_CORROB_A);
    const bool ch350HighDisc = ch350High && (i30  < HALL_WD_DISC_CORROB_A);

    _sensorDisconnected = ch30Low || ch350Low || ch30HighDisc || ch350HighDisc;

    // En el ciclo de cambio de rango el canal activo cambia: hay una
    // discontinuidad legítima en raw/corriente. No medir stuck/noise aquí.
    if (_rangeJustChanged) {
        _lastChangeTime = now;          // no acumular hacia "stuck"
        _sensorStuck    = false;
        _sensorNoisy    = false;
        _lastActiveRaw  = activeRaw;
        _lastCurrentRaw = _currentRaw;
        return;
    }

    // ── Sensor congelado (stuck) ──────────────────────────────────────────────
    // Sobre el RAW del ADC del canal activo: un canal vivo siempre tiene
    // ≥ unas LSB de dither; uno congelado/pegado da variación ~0.
    if (abs(activeRaw - _lastActiveRaw) > HALL_WD_STUCK_ADC_LSB) {
        _lastChangeTime = now;
        _sensorStuck    = false;
    } else if ((now - _lastChangeTime) > HALL_WD_STUCK_US) {
        _sensorStuck = true;
    }

    // ── Ruido extremo ─────────────────────────────────────────────────────────
    // Salto de corriente del canal activo entre dos lecturas. El timer de
    // fallo de 500ms (en _updateFaultTimers) descarta un salto aislado.
    _sensorNoisy = fabsf(_currentRaw - _lastCurrentRaw) > HALL_WD_NOISE_DELTA_A;

    _lastActiveRaw  = activeRaw;
    _lastCurrentRaw = _currentRaw;
}


// ============================================================================
//  TIMERS DE FALLO (500ms según norma FS EV 5.8)
// ============================================================================

void HallSensor::_updateFaultTimers()
{
    unsigned long now = millis();
    _faultConfirmed   = false;

    // ── Desconexión ───────────────────────────────────────────────────────────
    if (_sensorDisconnected) {
        if (_tFaultDisconnected == 0) _tFaultDisconnected = now;
        if ((now - _tFaultDisconnected) >= HALL_FAULT_MS) _faultConfirmed = true;
    } else {
        _tFaultDisconnected = 0;
    }

    // ── Sensor congelado ──────────────────────────────────────────────────────
    if (_sensorStuck) {
        if (_tFaultStuck == 0) _tFaultStuck = now;
        if ((now - _tFaultStuck) >= HALL_FAULT_MS) _faultConfirmed = true;
    } else {
        _tFaultStuck = 0;
    }

    // ── Ruido extremo ─────────────────────────────────────────────────────────
    if (_sensorNoisy) {
        if (_tFaultNoisy == 0) _tFaultNoisy = now;
        if ((now - _tFaultNoisy) >= HALL_FAULT_MS) _faultConfirmed = true;
    } else {
        _tFaultNoisy = 0;
    }

    // ── Sobreintensidad ───────────────────────────────────────────────────────
    // Sobre la corriente SIN filtrar: el lag del EMA (~130ms) NO debe
    // comerse el presupuesto de 500ms de EV5.8. Descarga > 170A,
    // carga < -7A (negativo = carga).
    bool overNow = (_currentRaw > HALL_I_MAX_DISCHARGE) ||
                   (_currentRaw < HALL_I_MAX_CHARGE);

    if (overNow) {
        if (_tFaultOverCurrent == 0) _tFaultOverCurrent = now;
        // Ventana propia (más corta que HALL_FAULT_MS): ver HallSensor.h.
        if ((now - _tFaultOverCurrent) >= HALL_OC_FAULT_MS) {
            _faultConfirmed = true;
            _overCurrent    = true;
        }
    } else {
        _tFaultOverCurrent = 0;
        _overCurrent       = false;
    }
}


// ============================================================================
//  DEBUG
// ============================================================================

void HallSensor::printStatus() const
{
    Serial.println(F("\n=== HallSensor STATUS ==="));
    Serial.printf("Corriente:    %.2f A (filt) | %.2f A (raw)  (%s)\n",
                  _currentFiltered, _currentRaw,
                  _usingLowRange ? "rango 30A" : "rango 350A");
    Serial.printf("isOK():       %s\n",  isOK()              ? "SI" : "NO (FALLO)");
    Serial.printf("Desconexion:  %s\n",  _sensorDisconnected ? "SI" : "no");
    Serial.printf("Congelado:    %s\n",  _sensorStuck        ? "SI" : "no");
    Serial.printf("Ruido:        %s\n",  _sensorNoisy        ? "SI" : "no");
    Serial.printf("Sobreintens.: %s\n",  _overCurrent        ? "SI" : "no");
    Serial.printf("ADC sat. 30A: %s  |  ADC sat. 350A: %s\n",
                  _adcSat30  ? "SI" : "no",
                  _adcSat350 ? "SI" : "no");
    Serial.printf("Offset din.   30A=%.4fV  350A=%.4fV\n",
                  _offsetDyn30, _offsetDyn350);
    Serial.println(F("========================"));
}
