// =============================================================================
//  fault_inject.cpp — BANCO DE VALIDACIÓN EN TIEMPO REAL de los tiempos de fallo
//
//  Se compila SOLO en el env `fault_bench` (platformio.ini), junto al main.cpp
//  de PRODUCCIÓN y con -D BMS_FAULT_INJECT. No duplica ninguna lógica: se limita
//  a interceptar las ENTRADAS de sampleAndEvaluate() (resultado de las lecturas,
//  condiciones badV/badT/badNtc, flag de precarga) y a observar las SALIDAS
//  (fallos confirmados). Las ventanas contra las que se compara salen de
//  include/bms_timing.h, las MISMAS que usa el firmware.
//
//  ⚠⚠ ESTE BINARIO FUERZA BMS_OK A LOW A PROPÓSITO. NO SUBIRLO AL COCHE.
//
//  ── USO ─────────────────────────────────────────────────────────────────────
//  Escenarios automáticos (miden y dan PASS/FAIL):
//    1 debounce V (500 ms)         6 fase blanda: 1er auto-address a 1 s
//    2 debounce T (1000 ms)        7 cadencia dura: auto-address cada 2 s
//    3 NTC abierto (1000 ms)       8 glitch V < ventana → NO debe fallar
//    4 ventana comms (30 s)        9 rearme V tras hueco de comms
//    5 reset del reloj por lectura buena
//    0 precarga: auto-address al primer error
//    A = todos en secuencia (~2 min)      X = abortar y limpiar
//  Inyección MANUAL (toggles, para trastear a mano):
//    Q badV   W badT   E badNtc   Y lectura V falla   U lectura T falla
//    P precarga
//  Otros:  Z traza on/off    ? ayuda
// =============================================================================
#ifdef BMS_FAULT_INJECT

#include <Arduino.h>
#include <stdarg.h>
#include <stdlib.h>
#include "fault_inject.h"
#include "bms_timing.h"

// ── Estado de inyección ──────────────────────────────────────────────────────
static constexpr uint32_t INJ_BADV   = 1u << 0;
static constexpr uint32_t INJ_BADT   = 1u << 1;
static constexpr uint32_t INJ_BADNTC = 1u << 2;
static constexpr uint32_t INJ_READV  = 1u << 3;
static constexpr uint32_t INJ_READT  = 1u << 4;
static constexpr uint32_t INJ_PRECHG = 1u << 5;
static constexpr uint32_t INJ_READS  = INJ_READV | INJ_READT;

static uint32_t inj     = 0;
static bool     traceOn = true;

// ── Detección de flancos sobre el estado que publica main.cpp ────────────────
static const char* const SIG[FI_NSIG] =
    { "faultV", "faultT", "faultNTC", "faultCOMM", "faultINIT", "faultHALL", "bmsFault" };
static bool          prevF[FI_NSIG]  = {false};
static unsigned long tRise[FI_NSIG]  = {0};
static bool          rose [FI_NSIG]  = {false};   ///< hubo flanco 0→1 desde el último clear

// ── Trazas de reInit (las alimenta main.cpp desde el camino de comms) ────────
static unsigned long tReinitLast  = 0;   ///< millis del inicio del último reInit
static unsigned long reinitGap    = 0;   ///< separación con el reInit anterior
static uint16_t      reinitCount  = 0;
static unsigned long reinitBlockMax = 0;
static bool          reinitSeen   = false;

// ── Runner de escenarios ─────────────────────────────────────────────────────
static char          scn      = 0;      ///< escenario en curso (0 = ninguno)
static uint8_t       fase     = 0;
static unsigned long tScn     = 0;      ///< origen de tiempos del escenario
static unsigned long tFase    = 0;      ///< origen de tiempos de la fase actual
static const char*   scnCola  = nullptr;///< resto de la secuencia de 'A'
static uint8_t       nPass = 0, nFail = 0;

void fiLanzar(char c);   // fwd: finEscenario() encadena la secuencia de 'A'
static bool ultimoInitOk = false;   ///< lo publica fiObserve; guarda el arranque de escenarios

static unsigned long ms() { return millis(); }

/// Los escenarios que miden CUANDO salta el primer auto-address (6 y 0) solo
/// son limpios si el rate-limit de COMM_REINIT_RETRY_MS ya esta cumplido: si el
/// episodio arranca dentro de la sombra de un auto-address anterior, el primero
/// se retrasa hasta que expira, y la medida saldria alta por una razon legitima
/// que no es la que se quiere probar. tReinitLast=0 significa "ninguno visto",
/// y entonces manda millis() (en el coche siempre es grande; en frio, no).
static bool rateLimitLibre(unsigned long now) {
    return (now - tReinitLast) >= (COMM_REINIT_RETRY_MS + 200);
}
static long rel(unsigned long t) { return (long)(t - tScn); }

static void tr(const char* fmt, ...) {
    if (!traceOn) return;
    char b[128];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    Serial.printf("[FI] %+7ld ms  %s\n", scn ? rel(ms()) : (long)ms(), b);
}

/// Compara medido vs esperado y contabiliza. `nota` sale siempre para poder
/// juzgar a ojo aunque el veredicto sea PASS.
static void veredicto(const char* nombre, long medido, long esperado, long tol) {
    bool ok = (medido >= 0) && (labs(medido - esperado) <= tol);
    ok ? nPass++ : nFail++;
    Serial.printf("[FI] === #%c %-34s %s  medido=%ld ms  esperado=%ld +-%ld\n",
                  scn, nombre, ok ? "PASS" : "FAIL", medido, esperado, tol);
}
static void veredictoBool(const char* nombre, bool ok, const char* detalle) {
    ok ? nPass++ : nFail++;
    Serial.printf("[FI] === #%c %-34s %s  %s\n", scn, nombre, ok ? "PASS" : "FAIL", detalle);
}

static void limpiar() { inj = 0; }

static void finEscenario() {
    limpiar();
    scn = 0; fase = 0;
    if (scnCola && *scnCola) {
        char sig = *scnCola++;
        fiLanzar(sig);
    } else if (scnCola) {
        scnCola = nullptr;
        Serial.printf("\n[FI] ############ SECUENCIA COMPLETA: %u PASS / %u FAIL ############\n\n",
                      nPass, nFail);
    }
}

void fiLanzar(char c) {
    scn   = c;
    fase  = 0;
    tScn  = ms();
    tFase = tScn;
    inj   = 0;
    for (int i = 0; i < FI_NSIG; i++) rose[i] = false;
    reinitSeen = false; reinitCount = 0; reinitBlockMax = 0; reinitGap = 0;
    Serial.printf("\n[FI] ---------- ESCENARIO #%c ----------\n", c);
}

// ── Ganchos que llama main.cpp ───────────────────────────────────────────────
BQResult fiRead(BQResult real, uint8_t path) {
    if (path == 0 && (inj & INJ_READV)) return BQResult::COMM_ERROR;
    if (path == 1 && (inj & INJ_READT)) return BQResult::COMM_ERROR;
    return real;
}
bool fiBad(bool real, uint8_t sig) {
    if (sig == FI_V   && (inj & INJ_BADV))   return true;
    if (sig == FI_T   && (inj & INJ_BADT))   return true;
    if (sig == FI_NTC && (inj & INJ_BADNTC)) return true;
    return real;
}
bool fiPrecharge(bool real) { return real || (inj & INJ_PRECHG); }

void fiReinit(unsigned long t0, unsigned long t1, bool ok) {
    unsigned long bloqueo = t1 - t0;
    reinitGap   = reinitCount ? (t0 - tReinitLast) : 0;
    tReinitLast = t0;
    reinitCount++;
    reinitSeen = true;
    if (bloqueo > reinitBlockMax) reinitBlockMax = bloqueo;
    // tr() sella con el instante ACTUAL, que aqui ya es el FINAL del bloqueo
    // (reInit acaba de volver). El dato que importa para juzgar el escalado es
    // cuando ARRANCO, asi que se imprime explicitamente.
    tr("REINIT #%u  ini=%+ld ms  BLOQUEO=%lu ms  separacion=%lu ms  ok=%d",
       reinitCount, scn ? (long)(t0 - tScn) : (long)t0, bloqueo, reinitGap, ok);
}

void fiBegin() {
    Serial.println(F("\n################################################################"));
    Serial.println(F("#  BANCO DE INYECCION DE FALLOS — BMS_OK SE FUERZA A LOW.       #"));
    Serial.println(F("#  *** NO SUBIR ESTE BINARIO AL COCHE ***                       #"));
    Serial.println(F("################################################################"));
    Serial.printf("Ventanas activas: V=%lu T=%lu NTC=%lu COMM=%lu INIT=%lu ms\n",
                  FAULT_V_MS, FAULT_T_MS, FAULT_NTC_MS, FAULT_COMM_MS, FAULT_INIT_MS);
    Serial.printf("Comms: blanda=%lu ms  cadencia=%lu ms  intentos=%d  rearme=%lu ms\n",
                  COMM_SOFT_RETRY_MS, COMM_REINIT_RETRY_MS, COMM_REINIT_ATTEMPTS,
                  FAULT_REARM_GAP_MS);
    Serial.printf("Muestreo: V=%lu ms T=%lu ms\n", SAMPLE_V_MS, SAMPLE_T_MS);
    Serial.println(F("Escenarios 1-9,0 | A=todos | X=abortar | Z=traza | ?=ayuda"));
    Serial.println(F("REQUIERE la cadena BQ conectada e inicializada: si no, sampleAndEvaluate()"));
    Serial.println(F("hace return antes de leer y no hay entradas que interceptar.\n"));
}

static void ayuda() {
    Serial.println(F("\n[FI] ESCENARIOS (miden y dan PASS/FAIL)"));
    Serial.println(F("  1 debounce V (500 ms)          6 fase blanda: 1er auto-addr a 1 s"));
    Serial.println(F("  2 debounce T (1000 ms)         7 cadencia dura: auto-addr cada 2 s"));
    Serial.println(F("  3 NTC abierto (1000 ms)        8 glitch V corto -> NO debe fallar"));
    Serial.println(F("  4 ventana comms (30 s)         9 rearme V tras hueco de comms"));
    Serial.println(F("  5 reset del reloj por lectura buena"));
    Serial.println(F("  0 precarga: auto-address al primer error"));
    Serial.println(F("  A todos en secuencia (~2 min)  X abortar y limpiar"));
    Serial.println(F("[FI] INYECCION MANUAL (toggle)"));
    Serial.println(F("  Q badV  W badT  E badNtc  Y lectura V falla  U lectura T falla  P precarga"));
    Serial.println(F("[FI] Z traza on/off   ? esta ayuda\n"));
}

bool fiCommand(char c) {
    switch (c) {
    case '1': case '2': case '3': case '4': case '5':
    case '6': case '7': case '8': case '9': case '0':
        if (!ultimoInitOk) { Serial.println(F("[FI] ABORTADO: cadena BQ sin inicializar. "
            "sampleAndEvaluate() hace return antes de leer, asi que no hay nada que medir. "
            "Conecta la cadena y usa 'i'.")); return true; }
        scnCola = nullptr; nPass = nFail = 0; fiLanzar(c); return true;
    case 'A':
        if (!ultimoInitOk) { Serial.println(F("[FI] ABORTADO: cadena BQ sin inicializar.")); return true; }
        nPass = nFail = 0;
        scnCola = "234567890";         // arranca por el 1 y sigue con la cola
        fiLanzar('1');
        Serial.println(F("[FI] secuencia completa lanzada (~2 min)"));
        return true;
    case 'X':
        limpiar(); scn = 0; scnCola = nullptr;
        Serial.println(F("[FI] inyeccion LIMPIADA, escenario abortado."));
        return true;
    case 'Z': traceOn = !traceOn;
        Serial.printf("[FI] traza %s\n", traceOn ? "ON" : "OFF"); return true;
    case '?': ayuda(); return true;
    // --- toggles manuales ---
    case 'Q': inj ^= INJ_BADV;   Serial.printf("[FI] badV=%d\n",  !!(inj&INJ_BADV));   return true;
    case 'W': inj ^= INJ_BADT;   Serial.printf("[FI] badT=%d\n",  !!(inj&INJ_BADT));   return true;
    case 'E': inj ^= INJ_BADNTC; Serial.printf("[FI] badNtc=%d\n",!!(inj&INJ_BADNTC)); return true;
    case 'Y': inj ^= INJ_READV;  Serial.printf("[FI] lecturaV_falla=%d\n", !!(inj&INJ_READV)); return true;
    case 'U': inj ^= INJ_READT;  Serial.printf("[FI] lecturaT_falla=%d\n", !!(inj&INJ_READT)); return true;
    case 'P': inj ^= INJ_PRECHG; Serial.printf("[FI] precarga=%d\n", !!(inj&INJ_PRECHG)); return true;
    default: return false;
    }
}

// ── Observador: una pasada por loop, desde updateBmsOk() ─────────────────────
void fiObserve(unsigned long now, const FiState& st) {
    ultimoInitOk = st.bmsInitOk;
    // Flancos de subida de cada fallo confirmado.
    for (int i = 0; i < FI_NSIG; i++) {
        if (st.fault[i] && !prevF[i]) {
            tRise[i] = now; rose[i] = true;
            tr("SUBE %s%s", SIG[i], i == FI_BMSFAULT ? "   (BMS_OK -> LOW)" : "");
        } else if (!st.fault[i] && prevF[i]) {
            tr("BAJA %s%s", SIG[i], i == FI_BMSFAULT ? "   (BMS_OK -> HIGH)" : "");
        }
        prevF[i] = st.fault[i];
    }
    if (!scn) return;

    unsigned long t  = now - tScn;      // desde el inicio del escenario
    unsigned long tf = now - tFase;     // desde el inicio de la fase
    long          tV = rose[FI_V]    ? (long)(tRise[FI_V]    - tScn) : -1;
    long          tT = rose[FI_T]    ? (long)(tRise[FI_T]    - tScn) : -1;
    long          tN = rose[FI_NTC]  ? (long)(tRise[FI_NTC]  - tScn) : -1;
    long          tC = rose[FI_COMM] ? (long)(tRise[FI_COMM] - tScn) : -1;

    switch (scn) {

    // ── 1/2/3: ventanas de debounce de celda ────────────────────────────────
    case '1':
        if (fase == 0) { inj |= INJ_BADV; tr("INYECTA badV=1"); fase = 1; }
        else if (rose[FI_V]) { veredicto("debounce V", tV, FAULT_V_MS, 250); finEscenario(); }
        else if (t > FAULT_V_MS + 2500) { veredicto("debounce V", -1, FAULT_V_MS, 250); finEscenario(); }
        break;
    case '2':
        if (fase == 0) { inj |= INJ_BADT; tr("INYECTA badT=1"); fase = 1; }
        else if (rose[FI_T]) { veredicto("debounce T", tT, FAULT_T_MS, 300); finEscenario(); }
        else if (t > FAULT_T_MS + 2500) { veredicto("debounce T", -1, FAULT_T_MS, 300); finEscenario(); }
        break;
    case '3':
        if (fase == 0) { inj |= INJ_BADNTC; tr("INYECTA badNtc=1"); fase = 1; }
        else if (rose[FI_NTC]) { veredicto("NTC abierto", tN, FAULT_NTC_MS, 300); finEscenario(); }
        else if (t > FAULT_NTC_MS + 2500) { veredicto("NTC abierto", -1, FAULT_NTC_MS, 300); finEscenario(); }
        break;

    // ── 4: ventana de comms completa ────────────────────────────────────────
    case '4':
        if (fase == 0) {
            inj |= INJ_READS;
            tr("INYECTA lecturas V y T fallando (ventana %lu ms)", FAULT_COMM_MS);
            fase = 1;
        } else if (rose[FI_COMM]) {
            veredicto("ventana comms", tC, FAULT_COMM_MS, 2500);
            Serial.printf("[FI]     auto-address lanzados=%u  bloqueo max=%lu ms\n",
                          reinitCount, reinitBlockMax);
            finEscenario();
        } else {
            if (tf >= 5000) { tFase = now; tr("... %lu/%lu ms  badRun=%u  reInit=%u",
                                              st.commBadMs, FAULT_COMM_MS,
                                              st.commBadRun, reinitCount); }
            if (t > FAULT_COMM_MS + 6000) { veredicto("ventana comms", -1, FAULT_COMM_MS, 2500); finEscenario(); }
        }
        break;

    // ── 5: una lectura buena reinicia el reloj de 30 s ──────────────────────
    //    fallo 10 s → bueno 2 s → fallo otra vez. El fallo NO debe llegar a los
    //    30 s del PRIMER error, sino 30 s después del reinicio (≈42 s).
    case '5':
        if (fase == 0) { inj |= INJ_READS; tr("FASE A: lecturas fallando 10 s"); fase = 1; tFase = now; }
        else if (fase == 1 && tf >= 10000) {
            inj &= ~INJ_READS; tr("FASE B: lecturas BUENAS 2 s -> debe reiniciar el reloj");
            fase = 2; tFase = now;
        }
        else if (fase == 2 && tf >= 2000) {
            inj |= INJ_READS; tr("FASE C: lecturas fallando otra vez"); fase = 3; tFase = now;
        }
        else if (fase == 3 && rose[FI_COMM]) {
            // esperado: 10000 + 2000 + 30000 = 42000 desde el inicio
            veredicto("reset del reloj por lectura buena", tC, 42000, 3000);
            finEscenario();
        }
        else if (fase >= 1 && rose[FI_COMM] && fase < 3) {
            veredictoBool("reset del reloj por lectura buena", false,
                          "fallo ANTES de tiempo: el reloj no se reinicio");
            finEscenario();
        }
        else if (t > 48000) { veredicto("reset del reloj por lectura buena", -1, 42000, 3000); finEscenario(); }
        break;

    // ── 6: fase blanda — el 1er auto-address no antes de COMM_SOFT_RETRY_MS ─
    case '6':
        if (fase == 0) {
            if (!rateLimitLibre(now)) break;      // esperar a tener el escalado libre
            inj |= INJ_READS; tr("INYECTA lecturas fallando (rate-limit libre)"); fase = 1;
            tScn = now;                            // recolocar el origen tras la espera
        }
        else if (reinitSeen) {
            veredicto("1er auto-address (fase blanda)",
                      (long)(tReinitLast - tScn), COMM_SOFT_RETRY_MS, 400);
            finEscenario();
        }
        else if (fase == 1 && t > COMM_SOFT_RETRY_MS + 4000) {
            veredicto("1er auto-address (fase blanda)", -1, COMM_SOFT_RETRY_MS, 400);
            finEscenario();
        }
        break;

    // ── 7: cadencia de la fase dura ─────────────────────────────────────────
    case '7':
        if (fase == 0) { inj |= INJ_READS; tr("INYECTA lecturas fallando 12 s"); fase = 1; }
        else if (t > 12000) {
            if (reinitCount >= 3)
                veredicto("cadencia auto-address", (long)reinitGap, COMM_REINIT_RETRY_MS, 600);
            else
                veredictoBool("cadencia auto-address", false, "menos de 3 auto-address en 12 s");
            Serial.printf("[FI]     total=%u  bloqueo max=%lu ms\n", reinitCount, reinitBlockMax);
            finEscenario();
        }
        break;

    // ── 8: glitch más corto que la ventana → NO debe confirmar ──────────────
    case '8':
        if (fase == 0) {
            inj |= INJ_BADV;
            tr("INYECTA badV solo %lu ms (< ventana %lu)", FAULT_V_MS - 200, FAULT_V_MS);
            fase = 1; tFase = now;
        } else if (fase == 1 && tf >= FAULT_V_MS - 200) {
            inj &= ~INJ_BADV; tr("glitch retirado; vigilando 2 s que NO confirme");
            fase = 2; tFase = now;
        } else if (fase == 2 && tf >= 2000) {
            veredictoBool("glitch V < ventana", !rose[FI_V],
                          rose[FI_V] ? "CONFIRMO y no debia" : "no confirmo (correcto)");
            finEscenario();
        } else if (rose[FI_V]) {
            veredictoBool("glitch V < ventana", false, "CONFIRMO y no debia");
            finEscenario();
        }
        break;

    // ── 9: rearme del debounce V tras un hueco largo de datos ───────────────
    //    A: badV 400 ms (acumula badRun sin llegar a confirmar)
    //    B: comms caidas 5 s (fV queda CONGELADO con su tStart viejo)
    //    C: comms vuelven con badV aun activo.
    //    Sin rearme confirmaria al INSTANTE (now-tStart >> ventana).
    //    Con rearme debe volver a medir FAULT_V_MS enteros desde C.
    case '9':
        if (fase == 0) { inj |= INJ_BADV; tr("FASE A: badV 200 ms (badRun<k, congelado no puede confirmar)"); fase = 1; tFase = now; }
        else if (fase == 1 && tf >= 200) {
            inj |= INJ_READS; tr("FASE B: comms caidas 5 s (fV congelado)"); fase = 2; tFase = now;
        }
        else if (fase == 2 && tf >= 5000) {
            inj &= ~INJ_READS;
            tr("FASE C: se quita la inyeccion; esperando 1er ciclo bueno REAL");
            fase = 3;
        }
        else if (fase == 3 && st.commBadRun == 0) {
            // Aqui es donde empieza la medida: el reloj de comms ya se reinicio,
            // o sea que ha entrado un ciclo de lectura bueno y fV tiene datos
            // frescos. Medir desde que se quita la inyeccion daria de mas: entre
            // medias cabe un reInit bloqueante (~1,1-1,5 s) sin ninguna lectura.
            tFase = now; fase = 4;
            tr("FASE D: comms recuperadas de verdad; midiendo el rearme de V");
        }
        else if (fase == 4 && rose[FI_V]) {
            long desdeC = (long)(tRise[FI_V] - tFase);
            // Sin rearme confirmaria a ~200 ms (2 muestras completan k=5 con el
            // tStart viejo); con rearme mide los 500 ms enteros. tol=200 discrimina.
            veredicto("rearme V tras hueco (desde recuperacion)", desdeC, FAULT_V_MS, 200);
            if (desdeC < 350)
                Serial.println(F("[FI]     ^ confirmo casi al instante: el REARME NO esta actuando"));
            finEscenario();
        }
        else if (fase == 4 && tf > FAULT_V_MS + 2500) {
            veredicto("rearme V tras hueco (desde recuperacion)", -1, FAULT_V_MS, 200);
            finEscenario();
        }
        else if (fase == 3 && tf > 8000) {
            veredictoBool("rearme V tras hueco", false, "las comms no se recuperaron");
            finEscenario();
        }
        else if (fase < 4 && rose[FI_V]) {
            veredictoBool("rearme V tras hueco", false, "confirmo antes de recuperar las comms");
            finEscenario();
        }
        break;

    // ── 0: precarga — se salta la fase blanda ───────────────────────────────
    case '0':
        if (fase == 0) {
            if (!rateLimitLibre(now)) break;      // idem: medida limpia
            inj |= INJ_PRECHG | INJ_READS;
            tr("INYECTA precarga + lecturas fallando (rate-limit libre)");
            fase = 1;
            tScn = now;
        } else if (reinitSeen) {
            long t1 = (long)(tReinitLast - tScn);
            veredictoBool("precarga: auto-address inmediato",
                          t1 < (long)COMM_SOFT_RETRY_MS,
                          t1 < (long)COMM_SOFT_RETRY_MS ? "sin esperar la fase blanda"
                                                        : "espero la fase blanda (mal)");
            Serial.printf("[FI]     1er auto-address a %ld ms (umbral fase blanda %lu ms)\n",
                          t1, COMM_SOFT_RETRY_MS);
            finEscenario();
        } else if (fase == 1 && t > COMM_SOFT_RETRY_MS + 3000) {
            veredictoBool("precarga: auto-address inmediato", false, "no hubo auto-address");
            finEscenario();
        }
        break;
    }
}

#endif  // BMS_FAULT_INJECT
