/**
 * @file    SocEstimator.h
 * @brief   Estimador de State-of-Charge (SOC) — coulomb counting + OCV.
 *
 * Celda: Samsung SDI INR21700-40T (New 40T / 40T3)
 *   · Capacidad nominal: 4.0 Ah  · V nom 3.6  · carga 4.2V  · corte 2.5V
 *   · DC-IR ≈ 15 mΩ  · química NCA
 *
 * MÉTODO (híbrido, lo estándar en automoción/FS):
 *   1. Motor = COULOMB COUNTING: integra la corriente del pack (Hall)
 *      sobre la capacidad. Preciso en dinámico, pero deriva.
 *   2. Corrección = OCV en REPOSO: cuando |I| es bajo y estable un rato
 *      y los voltajes son fiables, re-ajusta el SOC hacia el que indica
 *      la curva OCV→SOC (suavizado por EMA para no dar saltos).
 *   3. Inicialización: al arrancar (coche en reposo) SOC = OCV→SOC.
 *
 * ── ESTADO: APROXIMADO, PENDIENTE DE CARACTERIZACIÓN ────────────────────────
 *   El datasheet NO trae curva OCV–SOC (sus curvas son bajo carga, no
 *   OCV en reposo). La tabla `_OCV_SOC` de abajo es una curva NCA 21700
 *   GENÉRICA/típica, NO medida en estas celdas. Para precisión real hay
 *   que CARACTERIZAR: reposar el pack a SOC conocidos y registrar OCV,
 *   y sustituir la tabla. Hasta entonces el SOC es orientativo (el
 *   coulomb counting acota la deriva entre reposos).
 *
 *   `SOC_PACK_CAPACITY_AH` debe ser la capacidad REAL del pack
 *   = 4.0 Ah (celda) × Np (celdas en paralelo). [AJUSTAR a la topología]
 */

#ifndef SOC_ESTIMATOR_H
#define SOC_ESTIMATOR_H

// El tiempo se INYECTA (begin/update reciben `now`=millis()) en vez de leer el
// reloj, así la clase no depende de Arduino y es testeable en nativo
// (test/test_socestimator/). Solo necesita math/stdint.
#include <math.h>     // fabsf, lroundf
#include <stdint.h>   // uint8_t

// Capacidad del PACK (Ah). El SOC% depende del ratio ∫I·dt / Q, con
// I = corriente de pack (Hall) y Q = capacidad de pack. La celda 40T
// son 4.0 Ah; el pack = 4.0 × Np. [AJUSTAR a la topología real]
#ifndef SOC_PACK_CAPACITY_AH
#define SOC_PACK_CAPACITY_AH   4.0f
#endif

// Detección de reposo para confiar el OCV (IR despreciable a baja I).
#define SOC_REST_CURRENT_A     2.0f      ///< |I| < esto = candidato a reposo
#define SOC_REST_MS            30000UL   ///< estable este tiempo → reposo fiable
#define SOC_RECAL_PER_S        0.02f     ///< Coef. de re-ajuste hacia OCV POR SEGUNDO (tau ~50 s). Time-normalizado → no depende de la frecuencia del loop.

class SocEstimator {
public:
    explicit SocEstimator(float packCapacityAh = SOC_PACK_CAPACITY_AH)
        : _capAh(packCapacityAh) {}

    /**
     * @brief Inicializa el SOC desde la OCV (coche en REPOSO al arrancar).
     * @param minCellV  Tensión de la celda más baja (≈OCV si I≈0).
     * @param now       Tiempo actual (millis()). Se inyecta para testabilidad.
     */
    void begin(float minCellV, unsigned long now)
    {
        _soc      = _ocvToSoc(minCellV);
        _lastMs   = now;
        _restMs   = 0;
        _restSince = 0;
        _initDone = true;
    }

    /**
     * @brief Actualiza el SOC. Llamar periódicamente (cada loop vale).
     * @param packCurrentA  Corriente de pack (Hall). + = descarga, - = carga.
     * @param minCellV      Celda más baja (para re-snap OCV en reposo).
     * @param voltsReliable bms.isVoltageReadingReliable() (false si balanceo HW).
     * @param iReliable     hall.isOK(): si false, NO integra la corriente (railada).
     * @param now           Tiempo actual (millis()). Se inyecta para testabilidad.
     */
    void update(float packCurrentA, float minCellV, bool voltsReliable, bool iReliable, unsigned long now)
    {
        if (!_initDone) { begin(minCellV, now); return; }

        // ── Coulomb counting ────────────────────────────────────────────────
        float dt_s = (now - _lastMs) / 1000.0f;
        _lastMs = now;
        // Solo integrar si la corriente es fiable: si el Hall falla, getCurrent()
        // se va al rail y arrastraría el SOC en la ventana previa a confirmar el
        // fallo (mismo criterio que voltsReliable para el re-snap, eje corriente).
        if (iReliable && dt_s > 0.0f && dt_s < 5.0f) {  // descarta saltos raros
            // descarga (I>0) baja SOC; carga (I<0) sube.
            _soc -= (packCurrentA * dt_s) / (_capAh * 3600.0f) * 100.0f;
        }

        // ── Re-ajuste por OCV en reposo ─────────────────────────────────────
        bool atRest = voltsReliable && (fabsf(packCurrentA) < SOC_REST_CURRENT_A);
        if (atRest) {
            if (_restSince == 0) _restSince = now;
            if ((now - _restSince) >= SOC_REST_MS) {
                float ocvSoc = _ocvToSoc(minCellV);
                // Nudge hacia el OCV normalizado por tiempo: alpha_eff = coef/s × dt_s.
                // Así la convergencia es independiente de la frecuencia del loop
                // (antes, con alpha por iteración, saltaba al OCV en una fracción
                // de segundo y metía saltos en la telemetría).
                float a = SOC_RECAL_PER_S * dt_s;
                if (a > 1.0f) a = 1.0f;
                _soc += a * (ocvSoc - _soc);
            }
        } else {
            _restSince = 0;
        }

        if (_soc < 0.0f)   _soc = 0.0f;
        if (_soc > 100.0f) _soc = 100.0f;
    }

    float   socF() const { return _soc; }                        ///< % (float)
    uint8_t soc()  const { return (uint8_t)lroundf(_soc); }       ///< % (CAN UINT8)
    bool    ready() const { return _initDone; }

private:
    float         _capAh;
    float         _soc       = 50.0f;   ///< arranca 50% hasta begin()
    bool          _initDone  = false;
    unsigned long _lastMs    = 0;
    unsigned long _restSince = 0;
    unsigned long _restMs    = 0;

    /**
     * Curva OCV→SOC GENÉRICA NCA 21700 (NO caracterizada en estas celdas).
     * OCV en reposo (V) por celda → SOC (%). Interpolación lineal.
     * ⚠ SUSTITUIR por la curva medida del pack real (ver cabecera).
     */
    static float _ocvToSoc(float v)
    {
        // {OCV(V), SOC(%)} de 0 a 100, NCA típica.
        static const float V[]  = {3.00f,3.45f,3.54f,3.61f,3.67f,3.74f,
                                    3.82f,3.90f,3.99f,4.08f,4.18f};
        static const float S[]  = {0.f, 10.f, 20.f, 30.f, 40.f, 50.f,
                                    60.f, 70.f, 80.f, 90.f, 100.f};
        const int N = 11;
        if (v <= V[0])     return S[0];
        if (v >= V[N - 1]) return S[N - 1];
        for (int i = 1; i < N; i++) {
            if (v < V[i]) {
                float t = (v - V[i - 1]) / (V[i] - V[i - 1]);
                return S[i - 1] + t * (S[i] - S[i - 1]);
            }
        }
        return S[N - 1];
    }
};

#endif // SOC_ESTIMATOR_H
