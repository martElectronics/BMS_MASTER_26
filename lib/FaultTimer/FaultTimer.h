#pragma once
//==============================================================================
// FaultTimer — Debounce K-de-N consecutivas + ventana de tiempo de pared.
//
// Pieza central del antirruido del BMS: una condición de fallo solo se
// "confirma" si persiste K muestras malas CONSECUTIVAS *Y* una ventana de
// tiempo (windowMs). Un único "OK" rompe la serie. Así una muestra ruidosa
// aislada no dispara BMS_OK, pero un fallo persistente sí, a los windowMs
// exactos (FS-compliant, EV5.8).
//
// Es lógica PURA (sin Arduino/HW) → testeable en nativo. Ver
// test/test_faulttimer/. Tiempos en `unsigned long` para casar con millis().
//==============================================================================
#include <stdint.h>

struct FaultTimer {
    bool          cond   = false;   ///< última muestra (true = bad)
    uint8_t       badRun = 0;       ///< muestras malas consecutivas (satura a 255)
    unsigned long tStart = 0;       ///< wall-time del primer bad de la serie (ms)

    // Procesa una muestra. `bad`=true cuenta como fallo; `now`=millis().
    void sample(bool bad, unsigned long now) {
        cond = bad;
        if (bad) {
            if (badRun == 0) tStart = now;   // primer bad de la serie
            if (badRun < 255) badRun++;      // satura (NO da la vuelta)
        } else {
            badRun = 0;                      // un OK borra la serie
            tStart = 0;
        }
    }

    // Confirma si hay >=k malas consecutivas Y la ventana se cumplió.
    // La resta (now - tStart) en unsigned es correcta aun con overflow de millis().
    bool confirmed(unsigned long now, unsigned long windowMs,
                   uint8_t k = 2) const {
        return (badRun >= k) && ((now - tStart) >= windowMs);
    }
};
