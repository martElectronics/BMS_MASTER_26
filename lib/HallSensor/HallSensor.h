/**
 * @file    HallSensor.h
 * @brief   Driver para sensor de corriente Hall DHAB S/118 con doble rango.
 *
 * El DHAB S/118 tiene dos salidas analógicas:
 *   · Canal 30A:  alta resolución, rango ±30A,  sensibilidad 66mV/A
 *   · Canal 350A: baja resolución, rango ±350A, sensibilidad 4mV/A
 *
 * La biblioteca selecciona automáticamente el rango óptimo con histéresis
 * para evitar oscilaciones en el cambio. Incluye:
 *   · Autocalibración del offset al arrancar (500 muestras)
 *   · Corrección dinámica de offset a corriente baja
 *   · Filtro EMA (Exponential Moving Average)
 *   · Watchdog: detección de desconexión (incl. fallo de UN canal),
 *     sensor congelado y ruido extremo
 *   · Timer de fallo de 500ms según norma Formula Student EV 5.8
 *
 * ── PORTABILIDAD (IMPORTANTE) ───────────────────────────────────────────────
 *   begin() llama analogReadResolution(12). STM32duino usa 10-bit por
 *   defecto (ESP32 12-bit); sin esto las lecturas saldrían ~4× mal.
 *
 * ── ESTADO: PENDIENTE DE AJUSTE EN BANCO ────────────────────────────────────
 *   La lógica está corregida, pero los umbrales del watchdog DEPENDEN del
 *   ruido real del HW y NO se ajustan a ojo. Marcados con [TUNE] abajo:
 *     · HALL_WD_STUCK_ADC_LSB   (congelado vs. dither real del ADC)
 *     · HALL_WD_NOISE_DELTA_A   (ruido/EMI vs. dI/dt legítimo de un acelerón)
 *     · HALL_WD_DISC_CORROB_A   (raíl por desconexión vs. por sobre-rango real)
 *   Validar/afinar con el sensor montado y el coche en condiciones reales.
 *
 * Uso:
 * @code
 *   HallSensor hall(PIN_AMP_30A, PIN_AMP_350A);
 *   void setup() { hall.begin(); }            // autocalibración (~1s en reposo)
 *   void loop()  { hall.update();             // en cada ciclo
 *                  float I = hall.getCurrent();
 *                  bool ok = hall.isOK(); }   // false si fallo confirmado >500ms
 * @endcode
 *
 * LÍMITES (Formula Student):
 *   · Corriente máxima de descarga: 170A
 *   · Corriente máxima de carga:      7A
 *   · Tiempo de fallo:               500ms (norma EV 5.8)
 */

#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <Arduino.h>

// ============================================================================
//  CONFIGURACIÓN DEL SENSOR
// ============================================================================

// Divisor de entrada al ADC: 18k serie + 33k a GND. Teórico = 0.647.
// VALIDADO en banco (2026-05-21) por DOS métodos independientes que coinciden:
//   · offset 0 A: salida sensor 2.49 V → ADC 1.623 V → 1.623/2.49 = 0.652
//   · ganancia 3 puntos: 0 A→0, 5 A→5.0, 10 A→10.0 → sens real ≈ 0.0430 V/A
//     = 0.066 nativo × 0.652
// ⚠ El begin() DEBE calibrar con 0 A reales: si el cero sale mal, toda la
//   escala se desplaza (un boot previo dio offset 1.4975 V en vez de ~1.62 V
//   y leía ~10 % alto). Comprobar que en reposo el offset ≈ 1.62 V.
#define HALL_DIVIDER     0.652f   ///< 18k/33k validado en banco (teórico 0.647)
#define HALL_SENS_30A    (0.066f * HALL_DIVIDER)  ///< ≈ 0.0430 V/A en el pin (±30A)
#define HALL_SENS_350A   (0.004f * HALL_DIVIDER)  ///< ≈ 0.00261 V/A en el pin (±350A)

// Tensión de referencia y resolución del ADC
#define HALL_VREF        3.3f     ///< Tensión de referencia del ADC (V)
#define HALL_ADC_RES     4095     ///< Resolución del ADC (12 bits). begin() fuerza 12-bit.

// Histéresis de cambio de rango
#define HALL_THRESH_UP   30.0f    ///< Umbral para pasar de 30A → 350A (A)
#define HALL_THRESH_DOWN 25.0f    ///< Umbral para volver de 350A → 30A (A)

// Autocalibración
#define HALL_CALIB_SAMPLES    500
#define HALL_CALIB_DELAY_US   2000

// Corrección dinámica de offset (se aplica solo cuando |I| < ZERO_THRESHOLD).
// tau normalizada por dt dentro de update() → independiente de la cadencia del
// loop (antes era alpha=0.001 por llamada asumiendo update cada 20ms ≈ tau 20s).
#define HALL_ZERO_TAU_S      20.0f   ///< Constante de tiempo del offset dinámico (s)
#define HALL_ZERO_THRESHOLD  2.0f    ///< Umbral (A) por debajo del cual se ajusta el offset

// Filtro EMA de la corriente de salida (display/telemetría). tau normalizada por
// dt → independiente de la frecuencia del loop. Antes alpha=0.15 @ 20ms (≈130ms),
// pero el loop gira a ~kHz sin gate → el filtro casi no filtraba.
#define HALL_FILTER_TAU_S    0.13f   ///< Constante de tiempo del filtro de corriente (s)

// Protección ADC — margen de saturación
#define HALL_ADC_SAT_MARGIN  50     ///< Margen antes del rail para considerar saturación

// ── Watchdog ────────────────────────────────────────────────────────────────
#define HALL_WD_ADC_MIN      150       ///< ADC por debajo de esto → raíl bajo (=cable roto). Margen: un canal suelto da 0-48 cuentas; la corriente legítima más baja (-7A) da raw ~1640.
#define HALL_WD_ADC_MAX      4045      ///< ADC por encima de esto → en el raíl alto
#define HALL_WD_STUCK_US     500000UL  ///< Ventana sin dither de ADC → congelado (us)

// [TUNE] Congelado: el canal ACTIVO se considera congelado si su lectura
// RAW de ADC (en cuentas) no varía más que esto durante HALL_WD_STUCK_US.
// Un canal vivo siempre tiene ≥ unas pocas LSB de ruido de ADC; uno
// congelado/saturado a un valor fijo da variación 0. Antes se medía sobre
// la corriente (ΔI≤0.15A) → falso "stuck" con corriente real estable.
#define HALL_WD_STUCK_ADC_LSB   2      ///< [TUNE] cuentas ADC de dither mínimo de un canal vivo

// [TUNE] Ruido extremo: salto de corriente entre dos update() del canal
// activo mayor que esto. Desacoplado de THRESH_UP (antes 2×THRESH_UP=60A
// acoplaba histéresis de rango con detección de ruido). Debe quedar por
// ENCIMA del dI/dt máximo legítimo (acelerón) y por debajo del de EMI.
#define HALL_WD_NOISE_DELTA_A   60.0f  ///< [TUNE] ΔI (A) entre lecturas → ruido/EMI

// [TUNE] Desconexión de UN canal: si el canal activo está en el raíl pero
// el OTRO canal NO corrobora una corriente grande real (|I_otro| < esto),
// el raíl se debe a desconexión, no a sobre-rango legítimo.
#define HALL_WD_DISC_CORROB_A   20.0f  ///< [TUNE] corriente mínima que corrobora un raíl real

// Ventana de validez del offset en reposo. La salida del DHAB a 0 A es Vcc/2;
// tras el divisor ≈ 1.62 V en el ADC. Si el begin() calibra fuera de esta
// ventana, el sensor está desconectado/railado/mal alimentado al arrancar →
// no fiarse (isOK()=false hasta reinit con sensor sano).
#define HALL_OFFSET_MIN_V       1.40f  ///< V en el ADC (mínimo plausible a 0 A)
#define HALL_OFFSET_MAX_V       1.85f  ///< V en el ADC (máximo plausible a 0 A)

// Límites de corriente (Formula Student)
#define HALL_I_MAX_DISCHARGE  500.0f  ///< Corriente máxima de descarga (A)
#define HALL_I_MAX_CHARGE      -500.0f  ///< Corriente máxima de carga (A, negativa = carga)

// Tiempo de fallo según norma FS EV 5.8
#define HALL_FAULT_MS        250UL    ///< Fallo confirmado si persiste >500ms


// ============================================================================
//  CLASE HALLSENSOR
// ============================================================================

class HallSensor {
public:

    /**
     * @param pin30A   Pin analógico conectado al canal de 30A del DHAB S/118
     * @param pin350A  Pin analógico conectado al canal de 350A del DHAB S/118
     */
    HallSensor(uint8_t pin30A, uint8_t pin350A)
        : _pin30A(pin30A), _pin350A(pin350A) {}

    // ─── Ciclo de vida ────────────────────────────────────────────────────────

    /**
     * @brief Fija resolución ADC a 12-bit, inicializa y autocalibra el offset.
     * ⚠ Vehículo en reposo y sin corriente. Duración ~1 s.
     */
    void begin();

    /**
     * @brief Lee, filtra y evalúa watchdog/fallos. Llamar en cada loop().
     */
    void update();

    // ─── Getters de datos ─────────────────────────────────────────────────────

    /** @return Corriente FILTRADA (A). Positivo=descarga, negativo=carga. */
    float getCurrent() const { return _currentFiltered; }

    /** @return Corriente SIN filtrar (A). Usada internamente para protección. */
    float getCurrentRaw() const { return _currentRaw; }

    /** @return true si el rango activo es el de 30A (alta resolución). */
    bool  isLowRange() const { return _usingLowRange; }

    // ─── Estado de fallos ─────────────────────────────────────────────────────

    /** @return true si NO hay fallo confirmado Y el offset del boot es válido.
     *  Usar para gobernar BMS_OK. */
    bool isOK() const { return _offsetValid && !_faultConfirmed; }

    /** @return true si el offset calibrado al arrancar cayó en rango plausible. */
    bool isOffsetValid() const { return _offsetValid; }

    /** @return true si el sensor parece desconectado (raíl sin corroborar). */
    bool isDisconnected() const { return _sensorDisconnected; }

    /** @return true si el canal activo está congelado (ADC sin dither 500ms). */
    bool isStuck() const { return _sensorStuck; }

    /** @return true si ΔI entre lecturas supera HALL_WD_NOISE_DELTA_A. */
    bool isNoisy() const { return _sensorNoisy; }

    /** @return true si la corriente sale de los límites FS >500ms. */
    bool isOverCurrent() const { return _overCurrent; }

    /** @return true si algún canal ADC ha saturado (margen de raíl). */
    bool isAdcSaturated() const { return _adcSat30 || _adcSat350; }

    /** @brief Imprime el estado completo por Serial (diagnóstico). */
    void printStatus() const;


private:

    // ─── Pines ───────────────────────────────────────────────────────────────
    uint8_t _pin30A;
    uint8_t _pin350A;

    // ─── Calibración ─────────────────────────────────────────────────────────
    float _offset30Init  = 0.0f;
    float _offset350Init = 0.0f;
    float _offsetDyn30   = 0.0f;
    float _offsetDyn350  = 0.0f;

    // ─── Datos de corriente ───────────────────────────────────────────────────
    float _currentFiltered = 0.0f;  ///< Salida pública filtrada (EMA)
    float _currentRaw      = 0.0f;  ///< Corriente del canal activo sin filtrar
    bool  _usingLowRange   = true;  ///< true = rango 30A activo
    bool  _rangeJustChanged = false;///< true el ciclo en que cambia el rango
    unsigned long _lastUpdateUs = 0;///< micros() de la última update() (normalización dt de los EMA)

    // ─── ADC ─────────────────────────────────────────────────────────────────
    bool _adcSat30  = false;
    bool _adcSat350 = false;

    // ─── Watchdog ─────────────────────────────────────────────────────────────
    unsigned long _lastChangeTime = 0;   ///< Último dither significativo (us)
    int           _lastActiveRaw  = 0;   ///< RAW ADC previo del canal activo
    float         _lastCurrentRaw = 0.0f;///< I previa del canal activo (para ruido)
    bool          _sensorDisconnected = false;
    bool          _sensorStuck        = false;
    bool          _sensorNoisy        = false;

    // ─── Timers de fallo (500ms) ──────────────────────────────────────────────
    unsigned long _tFaultDisconnected = 0;
    unsigned long _tFaultStuck        = 0;
    unsigned long _tFaultNoisy        = 0;
    unsigned long _tFaultOverCurrent  = 0;

    bool _faultConfirmed = false;
    bool _overCurrent    = false;
    bool _offsetValid    = false;   ///< true si el offset del begin() es plausible

    // ─── Funciones privadas ───────────────────────────────────────────────────
    void _updateWatchdog(int raw30, int raw350, float i30, float i350);
    void _updateFaultTimers();
};

#endif // HALL_SENSOR_H
