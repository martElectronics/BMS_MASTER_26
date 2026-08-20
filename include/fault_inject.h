#pragma once
// =============================================================================
//  fault_inject.h — Ganchos de INYECCIÓN DE FALLOS para el banco de validación
//
//  Permiten forzar, en tiempo real y por serie, cada una de las condiciones que
//  disparan bmsFault, para medir sobre HW REAL que las ventanas de debounce
//  (FAULT_V_MS / FAULT_T_MS / FAULT_NTC_MS / FAULT_COMM_MS) y el escalado de
//  recuperación de comms se comportan como dice el diseño.
//
//  ⚠ CLAVE: main.cpp NO se duplica. El banco ejecuta el firmware de PRODUCCIÓN
//    tal cual (mismo sampleAndEvaluate, mismo updateBmsOk, mismas constantes de
//    bms_timing.h) y solo intercepta las ENTRADAS. Si el banco pasa, lo que ha
//    pasado es el código que va al coche.
//
//  Sin -D BMS_FAULT_INJECT todo esto son funciones inline vacías que devuelven
//  su argumento: el compilador no emite NADA y el binario de producción es
//  byte a byte idéntico (verificado por md5 del firmware.bin).
//
//  ⚠⚠ El binario del banco fuerza BMS_OK a LOW a propósito. NO SUBIR AL COCHE.
// =============================================================================
#include <stdint.h>
#include "BQ79606.h"    // BQResult

// Identificadores de las señales que se pueden forzar / observar.
enum FiSig : uint8_t {
    FI_V = 0,     ///< fallo de tensión de celda (fV)
    FI_T,         ///< fallo de temperatura de celda (fT)
    FI_NTC,       ///< NTC abierto (fNtc)
    FI_COMM,      ///< comms BQ caídas (fComm)
    FI_INIT,      ///< init BQ fallido (fInit)
    FI_HALL,      ///< fallo del amperímetro
    FI_BMSFAULT,  ///< bmsFault agregado (= BMS_OK LOW)
    FI_NSIG
};

/// Estado que main.cpp entrega al banco una vez por loop (solo lectura).
struct FiState {
    bool fault[FI_NSIG];   ///< fallos CONFIRMADOS ahora (índices FiSig)
    uint8_t  commBadRun;   ///< fComm.badRun
    unsigned long commBadMs; ///< ms desde la primera lectura mala (0 = sin racha)
    bool bmsInitOk;
    bool prechargeRunning;
};

#ifdef BMS_FAULT_INJECT

void     fiBegin();                                  ///< desde setup()
bool     fiCommand(char c);                          ///< desde handleSerial(); true si lo consumió
BQResult fiRead(BQResult real, uint8_t path);        ///< path 0=V 1=T — puede forzar COMM_ERROR
bool     fiBad(bool real, uint8_t sig);              ///< fuerza badV/badT/badNtc a true
bool     fiPrecharge(bool real);                     ///< fuerza prechargeRunning
void     fiReinit(unsigned long t0, unsigned long t1, bool ok);  ///< traza de un reInit
void     fiObserve(unsigned long now, const FiState& st);        ///< una vez por loop

#else   // ---- producción: todo desaparece ----

static inline void     fiBegin() {}
static inline bool     fiCommand(char)                  { return false; }
static inline BQResult fiRead(BQResult real, uint8_t)   { return real; }
static inline bool     fiBad(bool real, uint8_t)        { return real; }
static inline bool     fiPrecharge(bool real)           { return real; }
static inline void     fiReinit(unsigned long, unsigned long, bool) {}
static inline void     fiObserve(unsigned long, const FiState&)     {}

#endif
