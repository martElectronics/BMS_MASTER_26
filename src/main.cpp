/**
 * @file    main.cpp
 * @brief   BMS Master — Firmware de producción STM32G474RE (NUCLEO-G474RE)
 *
 * Reúne los componentes VALIDADOS EN HW (banco 20 ICs, 2026-05-19):
 *   · Driver BQ79606 (cadena daisy-chain UART)
 *   · HallSensor (amperímetro DHAB S/118, doble rango)
 *
 * El BALANCEO no forma parte de este firmware: se hace off-car con una
 * herramienta aparte (no se balancea en el coche). Vive validado en el
 * repo BQ_CLASS, no aquí. Por eso V siempre es fiable en este master.
 *
 * ── ARQUITECTURA DE SEGURIDAD (Formula Student EV5.8 / EV6) ──────────────────
 *   · El latch que abre el SDC es HARDWARE no programable (EV6.1.6). Por
 *     tanto BMS_OK es una señal de SALUD: NO-latching, con debounce, y
 *     AUTO-REARMA cuando el fallo se despeja. El HW retiene el SDC abierto
 *     hasta el reset manual; el firmware nunca debe sostener el latch.
 *   · El indicador rojo "AMS" lo enciende el latch HW (EV5.8.12) → SIN
 *     código en firmware.
 *   · Debounce por normativa: V leída/evaluada cada 500 ms y el fallo debe
 *     persistir ≥500 ms; T cada 1000 ms y ≥1000 ms. Un error que se corrige
 *     antes de su ventana NO baja BMS_OK.
 *   · Pérdida de medida (NTC abierto, comms BQ caídas) → fallo (EV5.8.13).
 *   · Sobre-I la gestiona HallSensor (debounce 500 ms propio).
 *
 * ⚠ TOTALBOARDS (en BQ79606.h) debe ser EXACTAMENTE el nº de ICs del HW.
 *   Validado en banco con 20. El pack completo son 24 (12 módulos × 2).
 *   Antes de desplegar en el pack real: TOTALBOARDS=24 y RE-VALIDAR.
 *   Este main deriva NUM_MODULES de TOTALBOARDS → se auto-adapta.
 *
 * ── COMANDOS SERIE ──────────────────────────────────────────────────────────
 *   v=voltajes  t=temps  a=amperimetro  s=status  f=fallos
 *   c=limpiar fallos BQ  i=re-init BQ  r=restart MCU
 *
 * ── PENDIENTE ───────────────────────────────────────────────────────────────
 *   · CAN (lib propia) y PWM ventiladores: marcados TODO, sin lib aún.
 *   · Umbrales de celda (UV/OV/UT/OT) y [TUNE] del HallSensor: confirmar
 *     contra datasheet de la celda / ruido real del HW (gap-analysis P1).
 */

#include <Arduino.h>
#include "BQ79606.h"            // driver de la cadena BQ
#include "HallSensor.h"
#include "MART_CAN.h"           // CAN (FDCAN1 PA11/PA12). Protocolo: TODO (mapa CAN)

// Capacidad del PACK (Ah) = 4.0 (celda 40T) × Np (celdas en paralelo).
// ⚠ [AJUSTAR] Np a la topología real del pack. Debe ir ANTES del include.
#define SOC_PACK_CAPACITY_AH   (4.0f * 1)
#include "SocEstimator.h"
#include "FanController.h"      // ventiladores: curva Tmax + feed-forward I
#include <IWatchdog.h>          // watchdog HW independiente (STM32 IWDG/LSI)

// ============================================================================
//  PINES — STM32G474RE (NUCLEO-G474RE)
// ============================================================================
// BQ79606 (van en BQConfig)
#define PIN_BQ_WAKE     PB_7
#define PIN_BQ_FAULT    PB_2
#define PIN_BQ_RX       PC_5
#define PIN_BQ_TX       PC_4
#define PIN_BMS_OK      PB_5    ///< Señal al SDC. La gestiona el driver (setBmsOk).

// Control SDC / estado (gestionados por el main)
#define PIN_MC_OK           PA_6   ///< HIGH siempre que el micro esté vivo
#define PIN_PRE_FAIL        PA_7   ///< HIGH si la precarga no termina a tiempo
#define PIN_OE_TXS          PB_10  ///< OE del level shifter (gated por VIO_3V3)
#define PIN_VIO_3V3         PB_0   ///< HIGH → activar OE_TXS
#define PIN_PRECHARGE_DONE  PA_5   ///< HIGH = precarga finalizada
#define PIN_SDC_3V3         PC_7   ///< HIGH = precarga iniciada (SDC cerrado)

// Amperímetro DHAB S/118
#define PIN_AMP_30A     PA_1   ///< Canal alta resolución (±30A)
#define PIN_AMP_350A    PA_0   ///< Canal baja resolución (±350A)

// PWM ventiladores (2/3 hilos vía driver, baja frecuencia)
#define PIN_PWM         PB_4

// Pendiente (sin lib aún): CAN PA12/PA11

// ============================================================================
//  OBJETOS
// ============================================================================
static const BQConfig bqCfg = {
    .uartPort     = 0,        // No usado en STM32 (pines van al driver)
    .pinWake      = PIN_BQ_WAKE,
    .pinFault     = PIN_BQ_FAULT,
    .pinBmsOk     = PIN_BMS_OK,
    .pinRx        = PIN_BQ_RX,
    .pinTx        = PIN_BQ_TX,
    .pinTxEnable  = -1,       // OE_TXS lo gestiona el main (gated por VIO)
    .baudrate     = 125000
};
BQ79606 bms(bqCfg);

HallSensor hall(PIN_AMP_30A, PIN_AMP_350A);

SocEstimator soc;   // SOC: coulomb counting + OCV (ver SocEstimator.h)

FanController fan(PIN_PWM);   // ventiladores (ver FanController.h)

// NUM_MODULES derivado del driver: 2 boards por módulo. Se auto-adapta a
// TOTALBOARDS (20 banco / 24 pack completo).
static constexpr int NUM_MODULES = TOTALBOARDS / 2;

// ============================================================================
//  UMBRALES Y TIEMPOS
// ============================================================================
// ⚠ [TUNE-datasheet] Confirmar UV/OV/UT/OT con el datasheet de la celda
//   (gap-analysis P1). Valores del diseño codigo_bms_master.txt.
#define CELL_UV_V      2.8f     ///< Undervoltage (V)
#define CELL_OV_V      4.2f     ///< Overvoltage (V)
#define CELL_UT_C    -20.0f     ///< Undertemperature (°C)
#define CELL_OT_C     60.0f     ///< Overtemperature (°C) — EV5.8.4: ≤60 o datasheet

// Debounce por normativa FS EV5.8
#define FAULT_V_MS     500UL    ///< V debe persistir ≥500 ms
#define FAULT_T_MS    1000UL    ///< T debe persistir ≥1000 ms
#define FAULT_NTC_MS  1000UL    ///< NTC abierto (pérdida de medida, clase T)
#define FAULT_COMM_MS  500UL    ///< Comms BQ caídas sin recuperar

// Cadencias de muestreo (EV5.8: V cada 500 ms, T cada 1000 ms)
#define SAMPLE_V_MS    500UL
#define SAMPLE_T_MS   1000UL
#define PRINT_MS      2000UL

#define PRECHARGE_TIMEOUT_MS  5000UL

// Watchdog HW independiente: si el loop() se cuelga y no se refresca,
// el IWDG resetea el MCU → BMS_OK cae (fail-safe). 8 s: por encima
// del peor caso normal incl. reInit() (que bloquea — ver §9.4 del doc;
// bajar este valor exige hacer reInit no bloqueante).
#define WDG_TIMEOUT_US        8000000UL

// ============================================================================
//  ESTADO DE FALLOS — debounce NO-latching (auto-rearma; el latch es HW)
// ============================================================================
struct FaultTimer {
    bool          cond = false;   ///< condición presente AHORA
    unsigned long tStart = 0;     ///< inicio de la condición (ms)
    bool confirmed(unsigned long now, unsigned long windowMs) {
        if (!cond) { tStart = 0; return false; }
        if (tStart == 0) tStart = now;
        return (now - tStart) >= windowMs;
    }
};
static FaultTimer fV, fT, fNtc, fComm;
static bool bmsFault = false;     ///< fallo confirmado AHORA (no latcheado)

// Lecturas
static BQResult lastResV = BQResult::OK, lastResT = BQResult::OK;

// Precarga
static bool          prechargeStarted = false;
static bool          prechargeOk      = false;
static unsigned long tPrechargeStart  = 0;

// ============================================================================
//  TELEMETRÍA CAN (resumen IDs 10-14; ver docs/Mapa_CAN.txt)
//  El CAN_BUS se construye en setup() (su ctor hace HAL FDCAN init y
//  debe correr tras el SystemClock del core → NO como global static).
// ============================================================================
static CAN_BUS*      gCan             = nullptr;
static bool          gCanOk           = false; ///< FDCAN inicializó OK (gate TX)
static uint16_t      canNumCommFails  = 0;   ///< nº lecturas BQ con COMM_ERROR
static uint16_t      canNumCrcFails   = 0;   ///< nº lecturas BQ con CRC_ERROR
static uint16_t      canNumTriesReset = 0;   ///< nº reInit() por comms
static unsigned long faultEpisodeStart= 0;   ///< ms inicio episodio fallo (0=sin)
static uint16_t      canLastFailMs    = 0;   ///< duración (ms) episodio actual/último
static uint16_t      canMaxFailMs     = 0;   ///< máx duración de episodio vista

// ============================================================================
//  PROTOTIPOS
// ============================================================================
void updateVio();
void sampleAndEvaluate();
void updateBmsOk();
void updatePrecharge();
void updateCanTx();
void handleSerial();
void printStatus();

// ============================================================================
//  SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    // Si el último reset lo causó el watchdog, dejar traza (diagnóstico).
    if (IWatchdog.isReset(true))
        Serial.println(F("[WDG] *** Reset previo causado por el WATCHDOG ***"));

    // Estado seguro inicial: BMS_OK lo pone el driver (LOW hasta init OK).
    pinMode(PIN_MC_OK,   OUTPUT); digitalWrite(PIN_MC_OK,   HIGH); // micro vivo
    pinMode(PIN_PRE_FAIL,OUTPUT); digitalWrite(PIN_PRE_FAIL, LOW);
    pinMode(PIN_OE_TXS,  OUTPUT); digitalWrite(PIN_OE_TXS,  LOW);
    pinMode(PIN_VIO_3V3,        INPUT);
    pinMode(PIN_PRECHARGE_DONE, INPUT);
    pinMode(PIN_SDC_3V3,        INPUT);

    Serial.println(F("========================================"));
    Serial.println(F("  BMS MASTER — STM32G474RE (Formula Student)"));
    Serial.println(F("========================================"));
    Serial.printf("  Modulos=%d (TOTALBOARDS=%d)  UV=%.2f OV=%.2f UT=%.0f OT=%.0f\n",
                  NUM_MODULES, TOTALBOARDS, CELL_UV_V, CELL_OV_V, CELL_UT_C, CELL_OT_C);
    Serial.printf("  I_max desc=%.0fA carga=%.0fA\n",
                  HALL_I_MAX_DISCHARGE, fabsf(HALL_I_MAX_CHARGE));

    hall.begin();   // autocalibración (~1 s, vehículo en reposo)
    fan.begin();    // PWM ventiladores a 0 %

    Serial.println(F("Iniciando BQ79606..."));
    bool initOk = bms.begin();
    if (!initOk) {
        // No colgar: BMS_OK queda LOW (lo gestiona el driver). Reintento por 'i'.
        Serial.println(F("[ERROR] BQ init fallo. BMS_OK LOW. Pulsa 'i' para reintentar."));
        while (!initOk) {
            if (Serial.available()) {
                char c = Serial.read();
                while (Serial.available()) Serial.read();
                if (c == 'i' || c == 'I') {
                    Serial.println(F(">>> Reintentando init..."));
                    initOk = bms.reInit();
                    Serial.println(initOk ? F("[OK]") : F("[ERROR]"));
                }
            }
            delay(10);
        }
    }
    Serial.println(F("[OK] BQ79606 listo."));

    // SOC: init desde OCV (se asume coche EN REPOSO al arrancar).
    if (bms.readVoltages() == BQResult::OK) {
        soc.begin(bms.getMinVoltage());
        Serial.printf("[SOC] init = %u%% (OCV)\n", soc.soc());
    }

    // ── CAN: FDCAN1 PA11/PA12, 500 kbps, perfil BMS (nodeID=3). El ctor
    //    hace el init de FDCAN (HAL) → debe correr AQUI, no en global. ──
    static CAN_BUS canBus(HardwareType::Transciever, 500, 3);
    gCan = &canBus;
    gCanOk = (gCan->SetupState() == 0);
    if (!gCanOk) {
        Serial.println(F("[CAN] FDCAN init FALLO — TX CAN deshabilitada."));
    } else {
        gCan->setPacketTimer(10, 800);   // periodos de docs/Mapa_CAN.txt
        gCan->setPacketTimer(11, 799);
        gCan->setPacketTimer(12, 799);
        gCan->setPacketTimer(13, 798);
        gCan->setPacketTimer(14, 200);
        gCan->setPacketTimer(392, 553);   // SOC
        gCan->setPacketTimer(386, 557);   // detalle/módulo (paginado)
        gCan->setPacketTimer(387, 556);
        gCan->setPacketTimer(388, 556);
        gCan->setPacketTimer(389, 555);
        gCan->setPacketTimer(390, 554);
        gCan->setPacketTimer(391, 554);
        Serial.println(F("[CAN] FDCAN listo (500k, IDs 10-14)."));
    }

    Serial.println(F("Cmd: v t a s f c i r"));

    // Arrancar el watchdog AL FINAL (tras el init/calibración acotados).
    // Una vez iniciado NO se puede parar (es independiente por HW).
    IWatchdog.begin(WDG_TIMEOUT_US);
    Serial.printf("[WDG] IWDG armado (%lu ms)\n", WDG_TIMEOUT_US / 1000);
}

// ============================================================================
//  LOOP
// ============================================================================
void loop()
{
    updateVio();          // OE_TXS según VIO_3V3
    hall.update();        // amperímetro cada ciclo (máxima resolución)

    sampleAndEvaluate();  // V/T en cadencia + debounce de fallos
    soc.update(hall.getCurrent(), bms.getMinVoltage(),
               bms.isVoltageReadingReliable());   // coulomb + OCV
    // Fail-safe de refrigeración: T no fresca/fiable (lectura T fallida,
    // comms o NTC abierto) → ventiladores 100 % (no fiarse de Tmax rancia).
    bool fanFS = (lastResT != BQResult::OK) || fComm.cond || fNtc.cond;
    fan.update(bms.getMaxTemp(), hall.getCurrent(), fanFS);  // curva + FF
    updateBmsOk();         // BMS_OK no-latching (auto-rearma)
    updatePrecharge();
    updateCanTx();         // telemetría CAN IDs 10-14 (throttled por timer)
    handleSerial();

    static unsigned long tPrint = 0;
    if (millis() - tPrint >= PRINT_MS) { tPrint = millis(); printStatus(); }

    // Refrescar el watchdog SOLO si la iteración completa terminó: si el
    // loop se cuelga en cualquier punto, el IWDG no se refresca → reset
    // → BMS_OK LOW (fail-safe). No mover esto al principio del loop.
    IWatchdog.reload();
}

// ============================================================================
//  OE_TXS (level shifter) — solo activo si VIO_3V3 presente
// ============================================================================
void updateVio()
{
    digitalWrite(PIN_OE_TXS, digitalRead(PIN_VIO_3V3) ? HIGH : LOW);
}

// ============================================================================
//  MUESTREO + EVALUACIÓN DE FALLOS (debounce, NO-latching)
// ============================================================================
void sampleAndEvaluate()
{
    static unsigned long tV = 0, tT = 0;
    static unsigned long tCommLost = 0;
    static bool commLost = false;
    unsigned long now = millis();

    // ── Voltaje: cada 500 ms (EV5.8). isVoltageReadingReliable() es
    //    siempre true en este firmware (no balancea); se mantiene como
    //    guarda defensiva. ────────────────────────────────────────────────
    if ((now - tV) >= SAMPLE_V_MS && bms.isVoltageReadingReliable()) {
        tV = now;
        lastResV = bms.readVoltages();
        if      (lastResV == BQResult::COMM_ERROR) canNumCommFails++;
        else if (lastResV == BQResult::CRC_ERROR)  canNumCrcFails++;
        if (lastResV == BQResult::OK) {
            fV.cond = (bms.getMinVoltage() < CELL_UV_V) ||
                      (bms.getMaxVoltage() > CELL_OV_V);
        }
    }

    // ── Temperatura + NTC abierto: cada 1000 ms (siempre fiable) ────────────
    if ((now - tT) >= SAMPLE_T_MS) {
        tT = now;
        lastResT = bms.readTemperatures();
        if      (lastResT == BQResult::COMM_ERROR) canNumCommFails++;
        else if (lastResT == BQResult::CRC_ERROR)  canNumCrcFails++;
        if (lastResT == BQResult::OK) {
            fT.cond   = (bms.getMinTemp() < CELL_UT_C) ||
                        (bms.getMaxTemp() > CELL_OT_C);
            // Pérdida de medida térmica (EV5.8.13): NTC abierto/ inválido.
            fNtc.cond = bms.hasOpenNtc();
        }
    }

    // ── Pérdida de comunicación BQ ──────────────────────────────────────────
    bool readErr = (lastResV != BQResult::OK) || (lastResT != BQResult::OK);
    if (readErr) {
        if (!commLost) {
            commLost  = true;
            tCommLost = now;
            bms.reInit();              // intento de recuperación
            canNumTriesReset++;
        }
        fComm.cond = ((now - tCommLost) >= FAULT_COMM_MS);
    } else {
        commLost   = false;
        fComm.cond = false;
    }
}

// ============================================================================
//  BMS_OK — no-latching, auto-rearma (el latch que abre el SDC es HW)
// ============================================================================
void updateBmsOk()
{
    unsigned long now = millis();

    bool faultV    = fV.confirmed(now,  FAULT_V_MS);
    bool faultT    = fT.confirmed(now,  FAULT_T_MS);
    bool faultNtc  = fNtc.confirmed(now, FAULT_NTC_MS);
    bool faultComm = fComm.confirmed(now, FAULT_COMM_MS);
    bool faultHall = !hall.isOK();      // HallSensor ya debounced 500 ms

    bmsFault = faultV || faultT || faultNtc || faultComm || faultHall;

    // Telemetría: duración del episodio de fallo (IDs 13/14).
    if (bmsFault) {
        if (faultEpisodeStart == 0) faultEpisodeStart = now;
        unsigned long dur = now - faultEpisodeStart;
        canLastFailMs = (dur > 65535UL) ? 65535 : (uint16_t)dur;
        if (canLastFailMs > canMaxFailMs) canMaxFailMs = canLastFailMs;
    } else {
        faultEpisodeStart = 0;   // canLastFailMs se mantiene (último episodio)
    }

    // Auto-rearma: cuando todo se despeja, BMS_OK vuelve a HIGH. El latch HW
    // mantiene el SDC abierto hasta el reset manual humano (EV6.1.6).
    bms.setBmsOk(!bmsFault);
}

// ============================================================================
//  PRECARGA
// ============================================================================
void updatePrecharge()
{
    bool sdcActive     = digitalRead(PIN_SDC_3V3);
    bool prechargeDone = digitalRead(PIN_PRECHARGE_DONE);

    if (sdcActive && !prechargeStarted && !prechargeOk) {
        prechargeStarted = true;
        tPrechargeStart  = millis();
        Serial.println(F("[PRE] Precarga iniciada."));
    }
    if (prechargeStarted && !prechargeOk) {
        if (prechargeDone) {
            prechargeOk = true;
            digitalWrite(PIN_PRE_FAIL, LOW);
            Serial.println(F("[PRE] Precarga OK."));
        } else if ((millis() - tPrechargeStart) >= PRECHARGE_TIMEOUT_MS) {
            digitalWrite(PIN_PRE_FAIL, HIGH);
            Serial.println(F("[PRE] FALLO: precarga no completada en 5s."));
        }
    }
    if (!sdcActive) {
        prechargeStarted = false;
        prechargeOk      = false;
        digitalWrite(PIN_PRE_FAIL, LOW);
    }
}

// ============================================================================
//  Helpers de mapeo módulo → celdas/NTC del driver
//  Módulo m = 2 boards: par (2m) 6 celdas + 6 NTC; impar (2m+1) 5 celdas
//  + 3 NTC.  V1..V6 = par 0..5 ; V7..V11 = impar 0..4 ; T1..T6 = par
//  NTC 0..5 ; T7..T9 = impar NTC 0..2.
//  ⚠ El mapeo Vn↔celda física y la def. de VTotal deben confirmarse
//    contra el cableado real / lo que espera el dashboard (el diseño
//    antiguo tenía una inconsistencia en V10/V11/celda dummy).
// ============================================================================
static float modCellV(int m, int n)   // n = 1..11
{
    int ev = 2 * m, od = 2 * m + 1;
    return (n <= 6) ? bms.getVoltage(ev, n - 1) : bms.getVoltage(od, n - 7);
}
static float modCellT(int m, int k)   // k = 1..9
{
    int ev = 2 * m, od = 2 * m + 1;
    return (k <= 6) ? bms.getTemperature(ev, k - 1) : bms.getTemperature(od, k - 7);
}
static uint16_t mv16(float v) { return (v > 0.0f) ? (uint16_t)lroundf(v * 1000.0f) : 0; }
static uint8_t  t8(float t)   { return (uint8_t)(int8_t)lroundf(t); }

// ============================================================================
//  TELEMETRÍA CAN — resumen IDs 10-14 (docs/Mapa_CAN.txt)
//  setPacket() encola; send() emite solo los que su setPacketTimer venció.
//  Patrón validado en el BMS antiguo (setPacket+send cada loop, throttle
//  por timer). Bits del ID 10 = fallo CONFIRMADO actual (no-latching;
//  el latch del SDC es HW). Detalle por módulo (386-392) y SOC: TODO.
// ============================================================================
void updateCanTx()
{
    if (!gCan || !gCanOk) return;

    // Recuperación de bus-off (cableado / baud / sin otros nodos). Barato
    // si el bus está sano (solo chequea FDCAN_PSR_BO); solo actúa si BO.
    static unsigned long tCanChk = 0;
    if (millis() - tCanChk >= 1000) { tCanChk = millis(); gCan->rebootBusFromError(); }

    // ── ID 10 (0xA) — estado general (1 byte de flags) ──────────────────────
    bool condNow = fV.cond || fT.cond || fNtc.cond || fComm.cond || !hall.isOK();
    uint8_t st = 0;
    if (bmsFault)                                  st |= (1 << 0); // B0 StsFail
    // B1, B2: reservados (0)
    if (condNow)                                   st |= (1 << 3); // B3 failCondition
    if (fComm.confirmed(millis(), FAULT_COMM_MS))  st |= (1 << 4); // B4 COMM
    if (fV.confirmed(millis(), FAULT_V_MS))        st |= (1 << 5); // B5 VOLT
    if (!hall.isOK())                              st |= (1 << 6); // B6 AMP
    if (bms.isOK())                                st |= (1 << 7); // B7 AUTOADDR_OK
    gCan->setPacket((uint32_t)10, &st, 1);

    // ── ID 11 (0xB) — MaxT(ºC), MaxV(mV), MinV(mV), MinT(ºC) UINT16×4 ───────
    // T puede ser negativa: se envía el patrón int16 en el slot u16.
    uint16_t d11[4] = {
        (uint16_t)(int16_t)lroundf(bms.getMaxTemp()),
        (uint16_t)lroundf(bms.getMaxVoltage() * 1000.0f),
        (uint16_t)lroundf(bms.getMinVoltage() * 1000.0f),
        (uint16_t)(int16_t)lroundf(bms.getMinTemp())
    };
    gCan->setPacket((uint32_t)11, d11, 4);

    // ── ID 12 (0xC) — GEN_STATUS_VOLT, GEN_STATUS_TEMP UINT16×2 ────────────
    // ⚠ PROVISIONAL: la tabla oficial de códigos NO está definida.
    //   Asumido VOLT: 0=OK 1=UV 2=OV ; TEMP: 0=OK 1=UT 2=OT 3=NTC_open.
    //   Confirmar con el equipo y ajustar.
    uint16_t gsV = (bms.getMinVoltage() < CELL_UV_V) ? 1 :
                   (bms.getMaxVoltage() > CELL_OV_V) ? 2 : 0;
    uint16_t gsT = bms.hasOpenNtc()                  ? 3 :
                   (bms.getMinTemp()    < CELL_UT_C) ? 1 :
                   (bms.getMaxTemp()    > CELL_OT_C) ? 2 : 0;
    uint16_t d12[2] = { gsV, gsT };
    gCan->setPacket((uint32_t)12, d12, 2);

    // ── ID 13 (0xD) — maxTotalFailTime(ms), numTriesResetComm UINT16×2 ─────
    uint16_t d13[2] = { canMaxFailMs, canNumTriesReset };
    gCan->setPacket((uint32_t)13, d13, 2);

    // ── ID 14 (0xE) — lastFailTime(ms), numCommFails, numCrcFails,
    //                  numTriesReset  UINT16×4 ─────────────────────────────
    uint16_t d14[4] = { canLastFailMs, canNumCommFails,
                        canNumCrcFails, canNumTriesReset };
    gCan->setPacket((uint32_t)14, d14, 4);

    // ── ID 392 (0x188) — SOC (UINT8, %) ────────────────────────────────────
    uint8_t socv = soc.soc();
    gCan->setPacket((uint32_t)392, &socv, 1);

    // ── Detalle por módulo 386-391 (paginado: 1 módulo por ronda) ───────────
    // El receptor identifica el módulo por el 1er campo (IDmodule) del
    // payload. Status por módulo = encoding PROVISIONAL (igual que ID12).
    static uint8_t       cMod  = 0;
    static unsigned long tcMod = 0;
    if (millis() - tcMod >= 556) {
        tcMod = millis();
        cMod  = (cMod + 1) % (uint8_t)NUM_MODULES;
    }
    {
        const uint16_t m = cMod;
        uint16_t vtot = 0;
        for (int n = 1; n <= 11; n++) vtot += mv16(modCellV(m, n));

        uint16_t d386[4] = { m, mv16(modCellV(m,1)), mv16(modCellV(m,2)), mv16(modCellV(m,3)) };
        uint16_t d387[4] = { m, mv16(modCellV(m,4)), mv16(modCellV(m,5)), mv16(modCellV(m,6)) };
        uint16_t d388[4] = { m, mv16(modCellV(m,7)), mv16(modCellV(m,8)), mv16(modCellV(m,9)) };
        uint16_t d389[4] = { m, mv16(modCellV(m,10)), mv16(modCellV(m,11)), vtot };
        gCan->setPacket((uint32_t)386, d386, 4);
        gCan->setPacket((uint32_t)387, d387, 4);
        gCan->setPacket((uint32_t)388, d388, 4);
        gCan->setPacket((uint32_t)389, d389, 4);

        float tmx = -300.0f, tmn = 300.0f;
        for (int k = 1; k <= 9; k++) {
            float t = modCellT(m, k);
            if (t > tmx) tmx = t;
            if (t < tmn) tmn = t;
        }
        bool uv = false, ov = false;
        for (int n = 1; n <= 11; n++) {
            float v = modCellV(m, n);
            if (v > CELL_OV_V)               ov = true;
            else if (v > 0.0f && v < CELL_UV_V) uv = true;
        }
        uint8_t stV = ov ? 2 : (uv ? 1 : 0);
        uint8_t stT = (tmn < CELL_UT_C) ? 1 : (tmx > CELL_OT_C ? 2 : 0);

        uint8_t d390[8] = { (uint8_t)m, t8(modCellT(m,1)), t8(modCellT(m,2)),
                            t8(modCellT(m,3)), t8(modCellT(m,4)), t8(modCellT(m,5)),
                            t8(modCellT(m,6)), t8(modCellT(m,7)) };
        uint8_t d391[8] = { (uint8_t)m, t8(modCellT(m,8)), t8(modCellT(m,9)),
                            t8(tmx), t8(tmn), stV, stT, 0 };
        gCan->setPacket((uint32_t)390, d390, 8);
        gCan->setPacket((uint32_t)391, d391, 8);
    }

    gCan->send();   // emite solo los vencidos según setPacketTimer
}

// ============================================================================
//  COMANDOS SERIE
// ============================================================================
void handleSerial()
{
    if (!Serial.available()) return;
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();

    switch (cmd) {

    case 'v':
        if (bms.readVoltages() == BQResult::OK) {
            for (int b = 0; b < TOTALBOARDS; b++) {
                Serial.printf("B%d:", b);
                for (int c = 0; c < CELLS_FOR_BOARD(b); c++)
                    Serial.printf(" %.3f", bms.getVoltage(b, c));
                Serial.println();
            }
            Serial.printf("Vmin=%.3f Vmax=%.3f Delta=%.1fmV\n",
                          bms.getMinVoltage(), bms.getMaxVoltage(),
                          bms.getVoltageDelta());
        } else Serial.println(F("[ERROR] lectura V"));
        break;

    case 't':
        if (bms.readTemperatures() == BQResult::OK) {
            for (int b = 0; b < TOTALBOARDS; b++) {
                Serial.printf("B%d:", b);
                for (int c = 0; c < (int)NTCS_PER_BOARD[b % 2]; c++)
                    Serial.printf(" %.1f", bms.getTemperature(b, c));
                Serial.println();
            }
            Serial.printf("Tmin=%.1f Tmax=%.1f  NTCopen=%d\n",
                          bms.getMinTemp(), bms.getMaxTemp(),
                          bms.getOpenNtcCount());
        } else Serial.println(F("[ERROR] lectura T"));
        break;

    case 'a':
        hall.printStatus();
        break;

    case 's':
        printStatus();
        break;

    case 'f': {
        for (int b = 0; b < TOTALBOARDS; b++) {
            BQFaultStatus fs;
            if (bms.getFaultStatus(b, fs) != BQResult::OK) {
                Serial.printf("B%d: ERROR\n", b); continue;
            }
            if (fs.hasAnyFault())
                Serial.printf("B%d: SUM=0x%X UV=0x%X OV=0x%X UT=0x%X OT=0x%X\n",
                              b, fs.summary, fs.uvFault, fs.ovFault,
                              fs.utFault, fs.otFault);
        }
        Serial.println(F("(boards sin fallo omitidos)"));
        break;
    }

    case 'c':
        bms.clearAllFaults();
        Serial.println(F("Fallos BQ limpiados."));
        break;

    case 'i':
        Serial.println(bms.reInit() ? F("Re-init [OK]") : F("Re-init [ERROR]"));
        break;

    case 'r':
        Serial.println(F("Restart..."));
        delay(100);
        NVIC_SystemReset();
        break;

    default: break;
    }
}

// ============================================================================
//  STATUS
// ============================================================================
void printStatus()
{
    Serial.println(F("\n=== BMS STATUS ==="));
    Serial.printf("BMS_OK:   %s%s\n",
                  bmsFault ? "LOW (FALLO)" : "HIGH (OK)",
                  bmsFault ? "" : "  [latch SDC es HW]");
    Serial.printf("V: min=%.3f max=%.3f d=%.1fmV\n",
                  bms.getMinVoltage(), bms.getMaxVoltage(), bms.getVoltageDelta());
    Serial.printf("T: min=%.1f max=%.1f  NTCopen=%d\n",
                  bms.getMinTemp(), bms.getMaxTemp(), bms.getOpenNtcCount());
    Serial.printf("I: %.2f A (%s)  Hall=%s\n",
                  hall.getCurrent(), hall.isLowRange() ? "30A" : "350A",
                  hall.isOK() ? "OK" : "FALLO");
    Serial.printf("SOC: %u%% (%.1f)  [aprox: tabla OCV generica]\n",
                  soc.soc(), soc.socF());
    Serial.printf("FAN: %u%% (%s)\n", fan.duty(), fan.isOn() ? "ON" : "off");
    Serial.printf("Fallos: V=%d T=%d NTC=%d COMM=%d HALL=%d\n",
                  fV.confirmed(millis(), FAULT_V_MS),
                  fT.confirmed(millis(), FAULT_T_MS),
                  fNtc.confirmed(millis(), FAULT_NTC_MS),
                  fComm.confirmed(millis(), FAULT_COMM_MS),
                  !hall.isOK());
    Serial.printf("PRE_FAIL=%s  VIO=%s  SDC=%s\n",
                  digitalRead(PIN_PRE_FAIL) ? "HIGH" : "LOW",
                  digitalRead(PIN_VIO_3V3)  ? "HIGH" : "LOW",
                  digitalRead(PIN_SDC_3V3)  ? "HIGH" : "LOW");
    Serial.println(F("=================="));
}
