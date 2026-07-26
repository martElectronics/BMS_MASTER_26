/**
 * @file    hall_bench.cpp
 * @brief   Sketch de banco para caracterizar el HallSensor (DHAB S/118).
 *
 * No compila en producción: solo en env:hall_bench (ver platformio.ini).
 *
 * Objetivo: ajustar los 3 [TUNE] del HallSensor antes de subir al coche.
 *
 *   a) Verificar resolución ADC (12-bit): RAW ≈2048 a 1.65 V.
 *   b) Validar reproducibilidad del offset entre arranques (begin()).
 *   c) Caracterizar el ruido base del ADC → ajustar HALL_WD_STUCK_ADC_LSB.
 *   d) Probar la detección de desconexión (canal cortado a 0 A).
 *
 * Uso típico de banco:
 *   1. Encender, NO mover. Anotar offset30/350 que sale en begin().
 *   2. Reiniciar 3-4 veces. Confirmar que el offset es reproducible.
 *   3. Comando 'z' (reset stats) → esperar 10-30 s → leer pp30/pp350.
 *      El umbral HALL_WD_STUCK_ADC_LSB debe ser ≈ pp/2 del canal activo
 *      (hoy fijado a 2 en HallSensor.h).
 *   4. Desconectar un canal físicamente → flag "D" debe pasar a 1
 *      al cabo de ≤500 ms (HALL_FAULT_MS).
 *
 * Comandos serie:
 *   p   pausa/reanuda la traza periódica
 *   s   dump completo (hall.printStatus)
 *   z   reset de min/max/media de las stats
 *   n   captura silenciosa de 5 s y resumen
 *   h   ayuda
 */

#include <Arduino.h>
#include <limits.h>
#include "HallSensor.h"

// Pines: mapa central de la PCB (lib/COMMON/board_pins.h). ÚNICA fuente de
// verdad, compartida con main.cpp/charger.cpp. 30A→PA0, 350A→PA1.
#include "board_pins.h"

// Cadencia de la traza periódica
#define PRINT_MS        200
#define NOISE_CAP_MS    5000UL

HallSensor hall(PIN_AMP_30A, PIN_AMP_350A);

// ── Stats de ruido por canal (RAW del ADC) ──────────────────────────────────
struct ChStats {
    int      minRaw = INT_MAX;
    int      maxRaw = INT_MIN;
    long     sum    = 0;
    uint32_t n      = 0;

    void reset() { minRaw = INT_MAX; maxRaw = INT_MIN; sum = 0; n = 0; }
    void sample(int v) {
        if (v < minRaw) minRaw = v;
        if (v > maxRaw) maxRaw = v;
        sum += v;
        n++;
    }
    int   pp()   const { return (n == 0) ? 0   : (maxRaw - minRaw); }
    float mean() const { return (n == 0) ? 0.f : (float)sum / (float)n; }
};

static ChStats st30, st350;
static bool    paused        = false;
static bool    captureMode   = false;
static uint32_t captureStart = 0;

// ── Helpers ────────────────────────────────────────────────────────────────
static void printHelp()
{
    Serial.println(F("\n── HALL BENCH ─────────────────────────────"));
    Serial.println(F(" p   pausa/reanuda la traza periodica"));
    Serial.println(F(" s   status completo (hall.printStatus)"));
    Serial.println(F(" z   reset min/max/media"));
    Serial.println(F(" n   captura silenciosa de 5 s + resumen"));
    Serial.println(F(" h   esta ayuda"));
    Serial.println(F("───────────────────────────────────────────\n"));
}

static void printHeader()
{
    Serial.println(F(
      "   t(ms)  RAW30 RAW350  I_raw(A) I_flt(A) Rng  "
      "D S N O Sat  OK  "
      "pp30 mean30    pp350 mean350"));
    Serial.println(F(
      " (offset inicial impreso por hall.begin(); offset dinamico con 's')"));
}

static void printLine(uint32_t now)
{
    const int   raw30   = analogRead(PIN_AMP_30A);
    const int   raw350  = analogRead(PIN_AMP_350A);
    const float ifilt   = hall.getCurrent();
    const float iraw    = hall.getCurrentRaw();
    const char* rng     = hall.isLowRange() ? "30A " : "350A";
    const char  fD      = hall.isDisconnected() ? 'D' : '.';
    const char  fS      = hall.isStuck()        ? 'S' : '.';
    const char  fN      = hall.isNoisy()        ? 'N' : '.';
    const char  fO      = hall.isOverCurrent()  ? 'O' : '.';
    const char  fSat    = hall.isAdcSaturated() ? '*' : '.';
    const char* ok      = hall.isOK() ? "SI" : "NO";

    char buf[180];
    snprintf(buf, sizeof(buf),
        "%8lu  %5d  %5d   %7.2f %7.2f  %-3s  "
        "%c %c %c %c  %c  %2s  "
        "%4d %7.1f    %4d %7.1f",
        (unsigned long)now,
        raw30, raw350,
        iraw, ifilt, rng,
        fD, fS, fN, fO, fSat, ok,
        st30.pp(),  st30.mean(),
        st350.pp(), st350.mean());
    Serial.println(buf);
}

static void noiseSummary()
{
    Serial.println(F("\n=== Resumen captura ruido ==="));
    char buf[180];
    snprintf(buf, sizeof(buf),
        "30A : n=%lu  min=%d max=%d pp=%d mean=%.2f\n"
        "350A: n=%lu  min=%d max=%d pp=%d mean=%.2f",
        (unsigned long)st30.n,  st30.minRaw,  st30.maxRaw,  st30.pp(),  st30.mean(),
        (unsigned long)st350.n, st350.minRaw, st350.maxRaw, st350.pp(), st350.mean());
    Serial.println(buf);
    Serial.println(F(
        "→ Ajuste sugerido HALL_WD_STUCK_ADC_LSB ≈ pp/2 del canal ACTIVO\n"
        "  (con margen, redondeado al entero inferior; valor actual en .h = 2)\n"));
}

static void handleSerial()
{
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') continue;

        switch (c) {
            case 'p':
                paused = !paused;
                Serial.println(paused ? F("[paused]") : F("[resumed]"));
                break;
            case 's':
                hall.printStatus();
                break;
            case 'z':
                st30.reset();
                st350.reset();
                Serial.println(F("[stats reset]"));
                break;
            case 'n':
                st30.reset();
                st350.reset();
                captureMode  = true;
                captureStart = millis();
                Serial.println(F("[captura 5 s — no toques nada]"));
                break;
            case 'h':
            case '?':
                printHelp();
                break;
            default:
                break;
        }
    }
}

// ── Arduino entry points ────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println(F("\n\n========================================"));
    Serial.println(F("  HALL BENCH  (DHAB S/118 caracterizacion)"));
    Serial.println(F("========================================"));
    Serial.println(F(" Pin 30A  = PA_1   |  Pin 350A = PA_0"));
    Serial.println(F(" 'h' para ayuda."));
    Serial.println(F("----------------------------------------\n"));

    hall.begin();   // autocalibracion (~1 s, sensor en reposo)
    printHeader();
}

void loop()
{
    const uint32_t now = millis();

    // Update siempre (timers y watchdog del driver necesitan cadencia regular).
    hall.update();

    // Stats sobre el RAW de los dos canales — leer directo del ADC:
    // hall.update() no expone los raws, pero el sensor no cambia entre ese
    // analogRead y este — la varianza adicional inducida es despreciable.
    st30.sample(analogRead(PIN_AMP_30A));
    st350.sample(analogRead(PIN_AMP_350A));

    handleSerial();

    // Modo captura: silenciar prints hasta que pasen NOISE_CAP_MS, luego
    // imprimir resumen y volver al modo normal.
    if (captureMode) {
        if (now - captureStart >= NOISE_CAP_MS) {
            captureMode = false;
            noiseSummary();
            printHeader();
        }
        return;
    }

    if (paused) return;

    static uint32_t tPrint = 0;
    if (now - tPrint >= PRINT_MS) {
        tPrint = now;
        printLine(now);
    }
}
