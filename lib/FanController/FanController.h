/**
 * @file    FanController.h
 * @brief   Control de ventiladores del pack — curva sobre Tmax + feed-forward I.
 *
 * Variable de control: Tmax del pack (la celda más caliente manda).
 *
 * Curva (lineal por tramos, con histéresis):
 *   · Tmax < FAN_T_OFF        → OFF (0 %)         (no enfriar pack frío)
 *   · Tmax ≥ FAN_T_ON         → ON                (histéresis FAN_T_ON/OFF)
 *   · FAN_T_ON → FAN_T_FULL   → rampa lineal FAN_MIN_DUTY % → 100 %
 *   · Tmax ≥ FAN_T_FULL       → 100 %
 *
 * FAN_T_FULL (50 °C) deja margen al corte FS de 60 °C (CELL_OT_C): los
 * ventiladores saturan mucho antes de cualquier fallo térmico → si
 * BMS_OK cae por T, es un fallo REAL de refrigeración. Los ventiladores
 * son gestión térmica, NO la función de seguridad (el corte 60 °C/1 s
 * sigue siendo el backstop).
 *
 * Feed-forward por corriente: con I de pack alta (cerca del límite de
 * la celda) se fuerza un piso de duty AUNQUE Tmax no haya subido aún
 * (anticipa la inercia térmica).
 *
 * HW: ventilador 2/3 hilos → PWM de BAJA frecuencia modulando la
 * alimentación vía driver/MOSFET (FAN_PWM_HZ). 8-bit (0..255).
 *
 * ── PENDIENTE [TUNE] ────────────────────────────────────────────────────────
 *   · FAN_FF_CURRENT_A depende de la corriente continua del pack
 *     (= límite celda × Np). Ajustar a la topología real.
 *   · FAN_PWM_HZ según el driver de ventilador concreto.
 *   · Umbrales de la curva: afinar con el comportamiento térmico real.
 */

#ifndef FAN_CONTROLLER_H
#define FAN_CONTROLLER_H

// El cálculo del duty (update) es lógica pura → testeable en nativo. Las
// llamadas a hardware (begin/_apply) van bajo #ifdef ARDUINO; en nativo son
// no-op. Fuera de Arduino solo necesita math/stdint.
#include <math.h>     // fabsf, lroundf
#include <stdint.h>   // uint8_t, uint32_t
#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifndef FAN_T_OFF
#define FAN_T_OFF        32.0f   ///< °C: histéresis apagado
#endif
#ifndef FAN_T_ON
#define FAN_T_ON         35.0f   ///< °C: arranque
#endif
#ifndef FAN_T_FULL
#define FAN_T_FULL       50.0f   ///< °C: 100 % (margen 10 °C al corte 60 °C)
#endif
#ifndef FAN_MIN_DUTY
#define FAN_MIN_DUTY     30      ///< % mínimo útil (debajo no mueve aire)
#endif
#ifndef FAN_FF_CURRENT_A
#define FAN_FF_CURRENT_A 100.0f  ///< [TUNE] |I| pack que dispara feed-forward
#endif
#ifndef FAN_FF_DUTY
#define FAN_FF_DUTY      50      ///< % piso de duty por feed-forward
#endif
#ifndef FAN_PWM_HZ
#define FAN_PWM_HZ       1000u   ///< [TUNE] frecuencia PWM (2/3 hilos vía driver)
#endif

class FanController {
public:
    explicit FanController(uint32_t pin) : _pin(pin) {}

    void begin()
    {
#ifdef ARDUINO
        pinMode(_pin, OUTPUT);
#if defined(STM32G4xx) || defined(ARDUINO_ARCH_STM32)
        analogWriteFrequency(FAN_PWM_HZ);   // STM32 core: setter global
#endif
        analogWriteResolution(8);
        _apply(0);
#endif
    }

    /**
     * @brief Recalcula y aplica el duty. Llamar periódicamente.
     * @param tmax          Tmax del pack (°C).
     * @param packCurrentA  Corriente de pack (Hall), para feed-forward.
     * @return duty aplicado (%).
     */
    uint8_t update(float tmax, float packCurrentA, bool failSafe = false)
    {
        // Fail-safe: si la T no es fiable/fresca (lo decide el caller),
        // refrigerar al máximo en vez de fiarse de una Tmax rancia.
        if (failSafe) { _on = true; _dutyPct = 100; _apply(100); return 100; }

        // Histéresis ON/OFF sobre Tmax
        if (_on) { if (tmax <  FAN_T_OFF) _on = false; }
        else     { if (tmax >= FAN_T_ON)  _on = true;  }

        uint8_t duty = 0;
        if (_on) {
            if (tmax >= FAN_T_FULL) {
                duty = 100;
            } else {
                float r = (tmax - FAN_T_ON) / (FAN_T_FULL - FAN_T_ON);
                if (r < 0.0f) r = 0.0f;
                duty = (uint8_t)(FAN_MIN_DUTY +
                        lroundf(r * (100.0f - FAN_MIN_DUTY)));
            }
        }

        // Feed-forward por corriente: piso de duty + fuerza ON (anticipa
        // la inercia térmica). Se auto-desactiva al bajar la corriente.
        if (fabsf(packCurrentA) > FAN_FF_CURRENT_A) {
            _on = true;
            if (duty < FAN_FF_DUTY) duty = FAN_FF_DUTY;
        }

        _dutyPct = duty;
        _apply(duty);
        return duty;
    }

    uint8_t duty() const { return _dutyPct; }
    bool    isOn() const { return _on; }

private:
    uint32_t _pin;
    uint8_t  _dutyPct = 0;
    bool     _on      = false;

    void _apply(uint8_t pct)
    {
#ifdef ARDUINO
        analogWrite(_pin, (int)lroundf(pct * 255.0f / 100.0f));
#else
        (void)pct;   // nativo: sin hardware (update() sigue devolviendo el duty)
#endif
    }
};

#endif // FAN_CONTROLLER_H
