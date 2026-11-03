/**
 * @file    TelemetryDashboard.cpp
 * @brief   Salida JSON de la telemetría en vivo. Ver TelemetryDashboard.h
 */
#include "TelemetryDashboard.h"

// Headers reales de los objetos que leemos (aquí sí necesitamos el tipo
// completo; en el .h van forward-declared).
#include "BQ79606.h"
#include "HallSensor.h"
#include "SocEstimator.h"
#include "FanController.h"

TelemetryDashboard::TelemetryDashboard(Stream& io, BQ79606& bms, HallSensor& hall,
                                       SocEstimator& soc, FanController& fan,
                                       const DashConfig& cfg)
    : io_(io), bms_(bms), hall_(hall), soc_(soc), fan_(fan), cfg_(cfg) {}

void TelemetryDashboard::update(const DashFaults& f) {
    if (!active_) return;
    uint32_t now = millis();
    if ((uint32_t)(now - lastMs_) < cfg_.refreshMs) return;
    lastMs_ = now;
    render_(f);
}

void TelemetryDashboard::render_(const DashFaults& f) {
    io_.print("{\"bmsOk\":"); io_.print(f.bmsOk ? "true" : "false");
    io_.print(",\"vMin\":"); io_.print(bms_.getMinVoltage(), 3);
    io_.print(",\"vMax\":"); io_.print(bms_.getMaxVoltage(), 3);
    io_.print(",\"vDelta\":"); io_.print(bms_.getVoltageDelta(), 1);
    io_.print(",\"i\":"); io_.print(hall_.getCurrent() * cfg_.currentSign, 2);
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
    io_.print("]");
    // Campos propios del firmware (el charger publica aquí el estado de carga).
    if (jsonExtra_) { io_.print(","); jsonExtra_(io_); }
    io_.print("}\n");
}
