/**
 * @file    TelemetryDashboard.cpp
 * @brief   Render ANSI del panel de telemetría en vivo. Ver TelemetryDashboard.h
 */
#include "TelemetryDashboard.h"

// Headers reales de los objetos que leemos (aquí sí necesitamos el tipo
// completo; en el .h van forward-declared).
#include "BQ79606.h"
#include "HallSensor.h"
#include "SocEstimator.h"
#include "FanController.h"

// ── Secuencias ANSI (color + control de cursor) ─────────────────────────────
#define A_RST      "\033[0m"
#define A_BOLD     "\033[1m"
#define A_RED      "\033[91m"
#define A_GRN      "\033[92m"
#define A_YEL      "\033[93m"
#define A_CYN      "\033[96m"
#define A_DIM      "\033[90m"
#define A_HOME     "\033[H"     // cursor a (1,1)
#define A_CLR      "\033[2J"    // borrar pantalla
#define A_EOL      "\033[K"     // borrar hasta fin de línea
#define A_CLRDOWN  "\033[J"     // borrar de cursor hacia abajo
#define A_HIDE     "\033[?25l"  // ocultar cursor
#define A_SHOW     "\033[?25h"  // mostrar cursor

// Línea separadora (ASCII, ~76 guiones).
static const char* DASH =
    "----------------------------------------------------------------------------";

// ── Helpers de celda (colorean solo lo anómalo → menos bytes y resalta) ─────
// Devuelven la nueva posición dentro del buffer. Ancho visible fijo = 7
// ("%6.3f" + espacio) para alinear con la cabecera de columnas.
static int appV(char* b, int pos, int cap, float v, float uv, float ov) {
    if (pos >= cap) return pos;
    const char* c = "";
    if (v <= 0.05f)                                c = A_DIM;   // sin dato
    else if (v < uv || v > ov)                     c = A_RED;   // fuera de rango
    else if (v <= uv + 0.03f || v >= ov - 0.03f)   c = A_YEL;   // al borde
    int n = (*c) ? snprintf(b + pos, cap - pos, "%s%6.3f%s ", c, v, A_RST)
                 : snprintf(b + pos, cap - pos, "%6.3f ", v);
    return (n > 0) ? pos + n : pos;
}
static int appT(char* b, int pos, int cap, float t, float ut, float ot) {
    if (pos >= cap) return pos;
    const char* c = "";
    if (t < -50.0f || t > 150.0f)   c = A_DIM;   // sin dato / NTC raro
    else if (t < ut || t > ot)      c = A_RED;   // fuera de rango
    else if (t >= ot - 5.0f)        c = A_YEL;   // cerca de OT
    int n = (*c) ? snprintf(b + pos, cap - pos, "%s%6.1f%s ", c, t, A_RST)
                 : snprintf(b + pos, cap - pos, "%6.1f ", t);
    return (n > 0) ? pos + n : pos;
}
static int appFlag(char* b, int pos, int cap, const char* name, bool bad) {
    if (pos >= cap) return pos;
    int n = snprintf(b + pos, cap - pos, "%s%s=%d" A_RST "  ",
                     bad ? A_RED : A_GRN, name, bad ? 1 : 0);
    return (n > 0) ? pos + n : pos;
}

// ── Ciclo de vida ───────────────────────────────────────────────────────────
TelemetryDashboard::TelemetryDashboard(Stream& io, BQ79606& bms, HallSensor& hall,
                                       SocEstimator& soc, FanController& fan,
                                       const DashConfig& cfg)
    : io_(io), bms_(bms), hall_(hall), soc_(soc), fan_(fan), cfg_(cfg) {}

void TelemetryDashboard::toggle() { active_ ? leave_() : enter_(); }

void TelemetryDashboard::enter_() {
    active_ = true;
    lastMs_ = 0;             // fuerza un primer render inmediato
    if (!jsonMode_) {
        io_.print(A_HIDE);
        io_.print(A_CLR);
        io_.print(A_HOME);
    }
}

void TelemetryDashboard::leave_() {
    active_ = false;
    if (!jsonMode_) {
        io_.print(A_RST);
        io_.print(A_SHOW);
        io_.print("\r\n");
    }
}

void TelemetryDashboard::update(const DashFaults& f) {
    if (!active_) return;
    uint32_t now = millis();
    if ((uint32_t)(now - lastMs_) < cfg_.refreshMs) return;
    lastMs_ = now;
    if (jsonMode_) renderJson_(f);
    else           render_(f);
}

void TelemetryDashboard::sepLine_() {
    io_.print("  ");
    io_.print(DASH);
    io_.print(A_EOL "\r\n");
}

// ── Render de un frame completo ─────────────────────────────────────────────
void TelemetryDashboard::render_(const DashFaults& f) {
    char l[512];
    char cn[8];
    int  p;

    io_.print(A_HOME);

    // Título + estado BMS_OK
    {
        const char* okc = f.bmsOk ? A_GRN : A_RED;
        const char* okt = f.bmsOk ? "HIGH (OK)  " : "LOW (FALLO)";
        snprintf(l, sizeof l,
                 A_BOLD A_CYN "  BMS MASTER  -  STM32G474RE" A_RST
                 "     BMS_OK: %s" A_BOLD "%s" A_RST, okc, okt);
        io_.print(l); io_.print(A_EOL "\r\n");
    }
    sepLine_();

    // Resumen (2 líneas)
    snprintf(l, sizeof l,
             "  Vmin %6.3f   Vmax %6.3f   d %5.1f mV      I %+7.2f A (%s)",
             bms_.getMinVoltage(), bms_.getMaxVoltage(), bms_.getVoltageDelta(),
             hall_.getCurrent(), hall_.isLowRange() ? "30A " : "350A");
    io_.print(l); io_.print(A_EOL "\r\n");

    snprintf(l, sizeof l,
             "  Tmin %6.1f   Tmax %6.1f   NTCopen %d       SOC %3u%%   FAN %3u%% (%s)",
             bms_.getMinTemp(), bms_.getMaxTemp(), bms_.getOpenNtcCount(),
             (unsigned)soc_.soc(), (unsigned)fan_.duty(), fan_.isOn() ? "ON " : "off");
    io_.print(l); io_.print(A_EOL "\r\n");
    sepLine_();

    // Fallos (verde=ok / rojo=activo)
    p = snprintf(l, sizeof l, "  FALLOS   ");
    p = appFlag(l, p, sizeof l, "V",    f.fV);
    p = appFlag(l, p, sizeof l, "T",    f.fT);
    p = appFlag(l, p, sizeof l, "NTC",  f.fNtc);
    p = appFlag(l, p, sizeof l, "COMM", f.fComm);
    p = appFlag(l, p, sizeof l, "HALL", f.fHall);
    p = appFlag(l, p, sizeof l, "INIT", !f.initOk);
    io_.print(l); io_.print(A_EOL "\r\n");

    io_.print(A_EOL "\r\n");   // línea en blanco

    // ── VOLTAJES ────────────────────────────────────────────────────────
    io_.print("  " A_BOLD "VOLTAJES (V)" A_RST); io_.print(A_EOL "\r\n");
    p = snprintf(l, sizeof l, "      ");                       // 6 col. cabecera
    for (int n = 1; n <= cfg_.cellsPerMod; n++) {
        snprintf(cn, sizeof cn, "c%d", n);
        p += snprintf(l + p, sizeof l - p, "%6s ", cn);        // ancho 7
    }
    io_.print(l); io_.print(A_EOL "\r\n");
    for (int m = 0; m < cfg_.numModules; m++) {
        p = snprintf(l, sizeof l, " M%-2d  ", m);              // etiqueta ancho 6
        for (int n = 1; n <= cfg_.cellsPerMod; n++)
            p = appV(l, p, sizeof l,
                     cfg_.cellV ? cfg_.cellV(m, n) : 0.0f, cfg_.uvV, cfg_.ovV);
        io_.print(l); io_.print(A_EOL "\r\n");
    }
    sepLine_();

    // ── TEMPERATURAS ────────────────────────────────────────────────────
    io_.print("  " A_BOLD "TEMPERATURAS (C)" A_RST); io_.print(A_EOL "\r\n");
    p = snprintf(l, sizeof l, "      ");
    for (int k = 1; k <= cfg_.ntcPerMod; k++) {
        snprintf(cn, sizeof cn, "n%d", k);
        p += snprintf(l + p, sizeof l - p, "%6s ", cn);
    }
    io_.print(l); io_.print(A_EOL "\r\n");
    for (int m = 0; m < cfg_.numModules; m++) {
        p = snprintf(l, sizeof l, " M%-2d  ", m);
        for (int k = 1; k <= cfg_.ntcPerMod; k++)
            p = appT(l, p, sizeof l,
                     cfg_.cellT ? cfg_.cellT(m, k) : -999.0f, cfg_.utC, cfg_.otC);
        io_.print(l); io_.print(A_EOL "\r\n");
    }
    sepLine_();

    // Pie
    snprintf(l, sizeof l,
             A_DIM "  [cualquier tecla] salir     refresco %lu ms" A_RST,
             (unsigned long)cfg_.refreshMs);
    io_.print(l); io_.print(A_EOL "\r\n");

    io_.print(A_CLRDOWN);   // limpia cualquier resto por debajo del panel
}

void TelemetryDashboard::renderJson_(const DashFaults& f) {
    io_.print("{\"bmsOk\":"); io_.print(f.bmsOk ? "true" : "false");
    io_.print(",\"vMin\":"); io_.print(bms_.getMinVoltage(), 3);
    io_.print(",\"vMax\":"); io_.print(bms_.getMaxVoltage(), 3);
    io_.print(",\"vDelta\":"); io_.print(bms_.getVoltageDelta(), 1);
    io_.print(",\"i\":"); io_.print(hall_.getCurrent(), 2);
    io_.print(",\"tMin\":"); io_.print(bms_.getMinTemp(), 1);
    io_.print(",\"tMax\":"); io_.print(bms_.getMaxTemp(), 1);
    io_.print(",\"ntcOpen\":"); io_.print(bms_.getOpenNtcCount());
    io_.print(",\"soc\":"); io_.print(soc_.soc());
    io_.print(",\"fan\":"); io_.print(fan_.duty());
    io_.print(",\"faults\":{");
    io_.print("\"v\":"); io_.print(f.fV ? "true" : "false");
    io_.print(",\"t\":"); io_.print(f.fT ? "true" : "false");
    io_.print(",\"ntc\":"); io_.print(f.fNtc ? "true" : "false");
    io_.print(",\"comm\":"); io_.print(f.fComm ? "true" : "false");
    io_.print(",\"hall\":"); io_.print(f.fHall ? "true" : "false");
    io_.print(",\"init\":"); io_.print(!f.initOk ? "true" : "false");
    io_.print("},\"modules\":[");
    for (int m = 0; m < cfg_.numModules; m++) {
        io_.print("{\"v\":[");
        for (int n = 1; n <= cfg_.cellsPerMod; n++) {
            io_.print(cfg_.cellV ? cfg_.cellV(m, n) : 0.0f, 3);
            if (n < cfg_.cellsPerMod) io_.print(",");
        }
        io_.print("],\"t\":[");
        for (int k = 1; k <= cfg_.ntcPerMod; k++) {
            io_.print(cfg_.cellT ? cfg_.cellT(m, k) : -999.0f, 1);
            if (k < cfg_.ntcPerMod) io_.print(",");
        }
        io_.print("]}");
        if (m < cfg_.numModules - 1) io_.print(",");
    }
    io_.print("]}\n");
}