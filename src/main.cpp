/**
 * @file    main.cpp
 * @brief   BMS Master — Firmware de producción STM32G474RE (NUCLEO-G474RE)
 *
 * Reúne los componentes VALIDADOS EN HW (banco 20 ICs, 2026-05-19):
 *   · Driver BQ79606 (cadena daisy-chain UART)
 *   · HallSensor (amperímetro DHAB S/118, doble rango)
 *
 * ── ARQUITECTURA DE SEGURIDAD (Formula Student EV5.8 / EV6) ──────────────────
 *   · El latch que abre el SDC es HARDWARE no programable (EV6.1.6). Por
 *     tanto BMS_OK es una señal de SALUD: NO-latching, con debounce, y
 *     AUTO-REARMA cuando el fallo se despeja. El HW retiene el SDC abierto
 *     hasta el reset manual; el firmware nunca debe sostener el latch.
 *   · ⚠ POLARIDAD INVERTIDA (solo rama testing): BMS_OK OK = LOW,
 *     fallo = HIGH (en las demás ramas es al revés: OK = HIGH). Donde
 *     abajo se diga "BMS_OK LOW/cae" como estado de FALLO, léase HIGH;
 *     "BMS_OK HIGH/sube" como estado OK, léase LOW.
 *   · El indicador rojo "AMS" lo enciende el latch HW (EV5.8.12) → SIN
 *     código en firmware.
 *   · Debounce por normativa: V leída/evaluada cada 500 ms y el fallo debe
 *     persistir ≥500 ms; T cada 1000 ms y ≥1000 ms. Un error que se corrige
 *     antes de su ventana NO dispara fallo en BMS_OK.
 *   · Pérdida de medida (NTC abierto, comms BQ caídas) → fallo (EV5.8.13).
 *   · Sobre-I la gestiona HallSensor (debounce 500 ms propio).
 *
 * ⚠ TOTALBOARDS (en BQ79606.h) debe ser EXACTAMENTE el nº de ICs del HW.
 *   Validado en banco con 20. El pack completo son 24 (12 módulos × 2).
 *   Antes de desplegar en el pack real: TOTALBOARDS=24 y RE-VALIDAR.
 *   Este main deriva NUM_MODULES de TOTALBOARDS → se auto-adapta.
 *
 * ── COMANDOS SERIE ──────────────────────────────────────────────────────────
 *   v=voltajes  t=temps  g=CSV módulos (app registro)  a=amperimetro
 *   s=status  f=fallos  c=limpiar fallos BQ  i=re-init BQ  r=restart MCU
 *   d=volcar log FRAM  D=reset índice log  C=reset contadores fallo (ID 16)
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
// ✓ Topología confirmada: 12 módulos × (6+5)s × 11p → Ns=132, Np=11, 44 Ah pack
// (banco hoy: 10 mods → Ns=110, misma Np y misma capacidad). La capacidad
// del pack solo depende de Np, no del nº de módulos en serie.
// Debe ir ANTES del include de SocEstimator.h.
#define SOC_PACK_CAPACITY_AH   (4.0f * 11)
#include "SocEstimator.h"
#include "FanController.h"      // ventiladores: curva Tmax + feed-forward I
#include "FaultLogger.h"        // FRAM MB85RC256V (I²C1 PB8/PB9, 0x50)
#include "FaultTimer.h"         // debounce K-de-N (lógica pura, testeable en nativo)
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
///< HIGH si la precarga no termina a tiempo
#define PIN_OE_TXS          PB_10  ///< OE del level shifter (gated por VIO_3V3)
#define PIN_VIO_3V3         PB_0   ///< HIGH → activar OE_TXS  ///< HIGH = precarga finalizada
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
// ✓ Confirmado contra datasheet Samsung INR21700-40T (rev. 2026-05-20):
//   UV 2.80 V — margen sobre el 2.50 V mín. del datasheet (preserva vida útil
//             y deja reserva para el debounce de 500 ms).
//   OV 4.20 V — estricto al máx. del datasheet (decisión del equipo:
//             priorizar conservadurismo frente a falsos OV al final de carga).
//   UT -20 °C — mín. descarga del datasheet; en pista funciona también como
//             detector de NTC roto/desconectado (lectura espuria muy baja).
//   OT  60 °C — coincide con FS EV5.8.4 y máx. del datasheet. Fans saturan
//             al 100 % a 50 °C → 10 °C de margen real antes del trip.
#define CELL_UV_V      2.8f     ///< Undervoltage (V)
#define CELL_OV_V      4.2f     ///< Overvoltage (V)
#define CELL_UT_C    -20.0f     ///< Undertemperature (°C)
#define CELL_OT_C     60.0f     ///< Overtemperature (°C) — EV5.8.4: ≤60

// Debounce por normativa FS EV5.8
#define FAULT_V_MS    100000UL    ///< V debe persistir ≥500 ms
#define FAULT_T_MS    100000UL    ///< T debe persistir ≥1000 ms
#define FAULT_NTC_MS  10000000UL    ///< NTC abierto (pérdida de medida, clase T)
#define FAULT_COMM_MS  100000UL    ///< Comms BQ caídas sin recuperar

// ⚠⚠ BANCO — relaja las ventanas de fallo para que un glitch del BQ no abra el
// SDC mientras se prueban otros subsistemas. SOLO en el build de banco
// (env [env:bms_bench], -D BENCH_BMS). EN PRODUCCIÓN rigen 0,5/1 s (FS EV5.8,
// scruti 103/104). NO FLASHEAR ESTE BUILD AL COCHE.
#ifdef BENCH_BMS
  #undef  FAULT_V_MS
  #undef  FAULT_T_MS
  #undef  FAULT_NTC_MS
  #undef  FAULT_COMM_MS
  #define FAULT_V_MS    1000UL   ///< BANCO: 1 s (prod 500 ms)
  #define FAULT_T_MS    1500UL   ///< BANCO: 1,5 s (prod 1000 ms)
  #define FAULT_NTC_MS  1500UL   ///< BANCO: 1,5 s (prod 1000 ms)
  #define FAULT_COMM_MS 1500UL   ///< BANCO: 1,5 s (prod 500 ms)
  #warning "BENCH_BMS activo: ventanas de fallo relajadas (1-1.5s). NO FLASHEAR AL COCHE."
#endif

// Cadencias de muestreo: 2× respecto al mínimo FS para tener ≥2 muestras
// dentro de cada ventana de debounce → mejor filtrado de ruido transitorio.
// Las ventanas FAULT_V_MS / FAULT_T_MS NO se tocan (las marca FS EV5.8).
#define SAMPLE_V_MS    250UL
#define SAMPLE_T_MS    500UL
#define PRINT_MS      2000UL

// Watchdog HW independiente: si el loop() se cuelga y no se refresca,
// el IWDG resetea el MCU → BMS_OK pasa a fallo (HIGH en esta rama). 8 s: por encima
// del peor caso normal incl. reInit() (que bloquea — ver §9.4 del doc;
// bajar este valor exige hacer reInit no bloqueante).
#define WDG_TIMEOUT_US        8000000UL

// ============================================================================
//  ESTADO DE FALLOS — debounce NO-latching (auto-rearma; el latch es HW)
//  La clase FaultTimer vive en lib/FaultTimer/FaultTimer.h (testeable en
//  nativo: test/test_faulttimer/, `pio test -e native`).
// ============================================================================
static FaultTimer fV, fT, fNtc, fComm, fInit;
static bool bmsFault = false;     ///< fallo confirmado AHORA (no latcheado)

// Estado de inicialización del BQ (#3, opción B: init/reInit no bloqueantes
// a nivel main). El loop arranca aunque begin() falle; sampleAndEvaluate()
// reintenta reInit() con rate-limit (no en cada flanco).
static bool          bmsInitOk    = false;   ///< true tras begin()/reInit() OK
static unsigned long tLastReinit  = 0;       ///< ms del último intento de reInit
#define BMS_REINIT_RETRY_MS  2000UL          ///< cadencia mín. entre reintentos

// Lecturas
static BQResult lastResV = BQResult::OK, lastResT = BQResult::OK;



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

// Contadores por causa (ID 16): nº de veces que cada fallo confirmado ha
// disparado desde el último reset. Se incrementan en el FLANCO DE SUBIDA
// (así captan episodios cortos que no verías en vivo). uint8 saturado a 255.
static uint8_t       cntFltV    = 0;   ///< episodios de fallo de tensión
static uint8_t       cntFltT    = 0;   ///< episodios de fallo de temperatura
static uint8_t       cntFltNtc  = 0;   ///< episodios de NTC abierto
static uint8_t       cntFltComm = 0;   ///< episodios de comms BQ caídas
static uint8_t       cntFltHall = 0;   ///< episodios de fallo del Hall
static uint8_t       cntFltInit = 0;   ///< episodios de init BQ fallido

// ── Debug / diagnóstico (ID 15 BMS_DEBUG) ──────────────────────────────────
// firstFaultTrigger : primer fallo que entró en este episodio (enum B4 ID 15)
//   0=none 1=V 2=T 3=NTC 4=Comm 5=Hall 6=!Init
// resetCauseSnapshot: RCC->CSR capturado al boot (bitfield B7 ID 15)
static uint8_t       firstFaultTrigger  = 0;
static uint8_t       resetCauseSnapshot = 0;

// ── Persistencia de fallos en FRAM (MB85RC256V via I²C1 PB8/PB9) ────────────
// Se loguea: BOOT (con reset cause), transiciones BMS_OK fall/rise,
// intentos de reInit, fallo de precarga. Lectura con comando 'd' (Serial).
static FaultLogger   logger;

// ============================================================================
//  PROTOTIPOS
// ============================================================================
void updateVio();
void sampleAndEvaluate();
void updateBmsOk();
void updateCanTx();
void handleSerial();
void printStatus();
static void buildDebugFlags(uint8_t out[4]);
static FaultRecord buildFaultRecord(uint8_t eventType, uint8_t firstFlt);

// ============================================================================
//  SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    // Snapshot del último reset (RCC->CSR) ANTES de cualquier clear → consumido
    // por ID 15 (BMS_DEBUG, B7). Mapeo de bits propio (no el del CSR raw) para
    // que el receptor no dependa de la familia STM32. En G4 POR y BOR
    // comparten BORRSTF; OBL = reset por option-byte loader.
    {
        uint32_t csr = RCC->CSR;
        uint8_t  rc  = 0;
        if (csr & RCC_CSR_LPWRRSTF) rc |= (1 << 0);  // LPWR
        if (csr & RCC_CSR_WWDGRSTF) rc |= (1 << 1);  // WWDG (window)
        if (csr & RCC_CSR_IWDGRSTF) rc |= (1 << 2);  // IWDG (independent) ←*
        if (csr & RCC_CSR_SFTRSTF)  rc |= (1 << 3);  // software reset
        if (csr & RCC_CSR_BORRSTF)  rc |= (1 << 4);  // POR/BOR (G4: BOR)
        if (csr & RCC_CSR_PINRSTF)  rc |= (1 << 5);  // NRST pin
        if (csr & RCC_CSR_OBLRSTF)  rc |= (1 << 7);  // option-byte loader
        resetCauseSnapshot = rc;
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }

    // Si el último reset lo causó el watchdog, dejar traza (diagnóstico).
    if (resetCauseSnapshot & (1 << 2))
        Serial.println(F("[WDG] *** Reset previo causado por el WATCHDOG ***"));

    // Estado seguro inicial: BMS_OK lo pone el driver (LOW hasta init OK).
    pinMode(PIN_OE_TXS,  OUTPUT); digitalWrite(PIN_OE_TXS,  LOW);
    pinMode(PIN_VIO_3V3,        INPUT);
    pinMode(PIN_SDC_3V3,        INPUT);

    Serial.println(F("========================================"));
    Serial.println(F("  BMS MASTER — STM32G474RE (Formula Student)"));
    Serial.println(F("========================================"));
#ifdef BENCH_BMS
    Serial.println(F("  *** BUILD DE BANCO (BENCH_BMS) ***"));
    Serial.println(F("  Ventanas de fallo RELAJADAS (1-1.5s)."));
    Serial.println(F("  >>> NO FLASHEAR AL COCHE <<<"));
    Serial.println(F("========================================"));
#endif
    Serial.printf("  Modulos=%d (TOTALBOARDS=%d)  UV=%.2f OV=%.2f UT=%.0f OT=%.0f\n",
                  NUM_MODULES, TOTALBOARDS, CELL_UV_V, CELL_OV_V, CELL_UT_C, CELL_OT_C);
    Serial.printf("  I_max desc=%.0fA carga=%.0fA\n",
                  HALL_I_MAX_DISCHARGE, fabsf(HALL_I_MAX_CHARGE));

    hall.begin();   // autocalibración (~1 s, vehículo en reposo)
    fan.begin();    // PWM ventiladores a 0 %

    // Init NO BLOQUEANTE (#3, opción B): si begin() falla, NO colgamos
    // esperando 'i'. El loop arranca igualmente y sampleAndEvaluate()
    // reintenta reInit() cada BMS_REINIT_RETRY_MS. BMS_OK forzado a fallo
    // (HIGH) mientras !bmsInitOk (ver updateBmsOk: bmsFault |= !bmsInitOk).
    Serial.println(F("Iniciando BQ79606..."));
    bmsInitOk = bms.begin();
    // Estado inicial del pin: fallo hasta que el primer loop() lo decida
    digitalWrite(PIN_BMS_OK, HIGH);  // HIGH = fallo en esta rama
    if (bmsInitOk) {
        Serial.println(F("[OK] BQ79606 listo."));
    } else {
        tLastReinit = millis();
        Serial.println(F("[WARN] BQ init FALLO. BMS_OK en fallo (HIGH); reintento en loop cada 2s."));
    }

    // SOC: init desde OCV (se asume coche EN REPOSO al arrancar).
    if (bms.readVoltages() == BQResult::OK) {
        soc.begin(bms.getMinVoltage(), millis());
        Serial.printf("[SOC] init = %u%% (OCV)\n", soc.soc());
    }

    // ── CAN: FDCAN1 PA11/PA12, 125 kbps (bus del coche), perfil BMS (nodeID=3).
    //    El ctor hace el init de FDCAN (HAL) → debe correr AQUI, no en global. ──
    static CAN_BUS canBus(HardwareType::Transciever, 125, 3);
    gCan = &canBus;
    gCanOk = (gCan->SetupState() == 0);
    if (!gCanOk) {
        Serial.println(F("[CAN] FDCAN init FALLO — TX CAN deshabilitada."));
    } else {
        gCan->configurePacketTimersByPriority();   // BMS_DEBUG (todos los fallos por bits)
        gCan->setPacketTimer(16, 500);             // ID 16 contadores de fallo: 500 ms
        // 13/14/15 NO están en configurePacketTimersByPriority() (la lista salta de
        // 12 a 33); sin timer, send() los emitiría cada loop (kHz) y saturaría el bus
        // (ademas son IDs bajos = alta prioridad CAN → inanición de VCU/PDM).
        gCan->setPacketTimer(13, 500);             // ID 13 diagnóstico
        gCan->setPacketTimer(14, 500);             // ID 14 tiempos/contadores de comms
        gCan->setPacketTimer(15, 500);             // ID 15 BMS_DEBUG
        Serial.println(F("[CAN] FDCAN listo (125k, IDs 10-16, 386-392)."));
    }

    // ── FRAM logger (MB85RC256V): si está, loguear evento BOOT con
    //    el resetCauseSnapshot capturado arriba. No bloquea si la FRAM
    //    no responde — la telemetría sigue funcionando por CAN/Serial.
    if (logger.begin()) {
        logger.log(buildFaultRecord(FaultLogger::EVT_BOOT, 0));
    }

    Serial.println(F("Cmd: v t a s f c i r  d=dump log  D=clear log  C=clear cnt"));

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
    // voltsReliable: solo fiar la V (para el re-snap OCV) si el BQ está
    // inicializado Y la última lectura de tensión fue OK. Si no, getMinVoltage()
    // devuelve 0/rancio y en reposo arrastraría el SOC hacia un valor falso.
    bool vReliable = bmsInitOk && (lastResV == BQResult::OK);
    bool iReliable = hall.isOK();   // no integrar corriente railada si el Hall falla
    soc.update(hall.getCurrent(), bms.getMinVoltage(), vReliable, iReliable, millis());   // coulomb + OCV
    // Fail-safe de refrigeración: T no fresca/fiable (lectura T fallida,
    // comms o NTC abierto) → ventiladores 100 % (no fiarse de Tmax rancia).
    bool fanFS = (lastResT != BQResult::OK) || fComm.cond || fNtc.cond;
    fan.update(bms.getMaxTemp(), hall.getCurrent(), fanFS);  // curva + FF
    updateBmsOk();         // BMS_OK no-latching (auto-rearma)
    updateCanTx();         // telemetría CAN IDs 10-14 (throttled por timer)
    handleSerial();

    static unsigned long tPrint = 0;
    if (millis() - tPrint >= PRINT_MS) { tPrint = millis(); printStatus(); }

    // Refrescar el watchdog SOLO si la iteración completa terminó: si el
    // loop se cuelga en cualquier punto, el IWDG no se refresca → reset
    // → BMS_OK a fallo (HIGH en esta rama). No mover esto al principio del loop.
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
    unsigned long now = millis();

    // ───────────────────────────────────────────────
    // 1) INIT FALLIDO (driver no inicializado)
    //    → NO leer V/T
    //    → reintentar con rate-limit
    //    → aplicar debounce de INIT
    // ───────────────────────────────────────────────
    if (!bmsInitOk) {

        // Debounce de INIT (fallo mientras !bmsInitOk)
        fInit.sample(true, now);

        // También consideramos fallo de COMM mientras no hay init
        fComm.sample(true, now);

        // Reintento rate-limited
        if ((now - tLastReinit) >= BMS_REINIT_RETRY_MS) {
            tLastReinit = now;
            canNumTriesReset++;

            if (bms.reInit()) {
                bmsInitOk = true;

                // 🔥 LÍNEA CRÍTICA: limpiar debounce de INIT al recuperarse
                fInit.sample(false, now);

                Serial.println(F("[OK] BQ recuperado."));
            }
        }
        return;
    }

    // Si estamos aquí, INIT está OK → limpiar debounce de INIT
    fInit.sample(false, now);

    // ───────────────────────────────────────────────
    // 2) LECTURA DE VOLTAJE (cada SAMPLE_V_MS)
    // ───────────────────────────────────────────────
    if ((now - tV) >= SAMPLE_V_MS) {
        tV = now;
        lastResV = bms.readVoltages();

        if      (lastResV == BQResult::COMM_ERROR) canNumCommFails++;
        else if (lastResV == BQResult::CRC_ERROR)  canNumCrcFails++;

        if (lastResV == BQResult::OK) {
            bool badV = (bms.getMinVoltage() < CELL_UV_V) ||
                        (bms.getMaxVoltage() > CELL_OV_V);
            fV.sample(badV, now);
        }
    }

    // ───────────────────────────────────────────────
    // 3) LECTURA DE TEMPERATURA (cada SAMPLE_T_MS)
    // ───────────────────────────────────────────────
    if ((now - tT) >= SAMPLE_T_MS) {
        tT = now;
        lastResT = bms.readTemperatures();

        if      (lastResT == BQResult::COMM_ERROR) canNumCommFails++;
        else if (lastResT == BQResult::CRC_ERROR)  canNumCrcFails++;

        if (lastResT == BQResult::OK) {
            bool badT = (bms.getMinTemp() < CELL_UT_C) ||
                        (bms.getMaxTemp() > CELL_OT_C);
            fT.sample(badT, now);

            // NTC abierto
            fNtc.sample(bms.hasOpenNtc(), now);
        }
    }

    // ───────────────────────────────────────────────
    // 4) DEBOUNCE DE COMM (fallo si V o T fallan)
    // ───────────────────────────────────────────────
    bool readErr = (lastResV != BQResult::OK) || (lastResT != BQResult::OK);
    fComm.sample(readErr, now);

    if (readErr) {
        if ((now - tLastReinit) >= BMS_REINIT_RETRY_MS) {
            tLastReinit = now;
            canNumTriesReset++;
            bms.reInit();
        }
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
    static FaultTimer fInit;
    fInit.sample(!bmsInitOk, now);
    bool faultInit = fInit.confirmed(now, FAULT_COMM_MS);


    // Contadores por causa (ID 16): incrementar en el FLANCO de subida de
    // cada fallo confirmado (0→1). Capta episodios cortos aunque ya se hayan
    // despejado al mirar el bus. Saturan a 255 para no dar la vuelta.
    static bool pV = false, pT = false, pNtc = false,
                pComm = false, pHall = false, pInit = false;
    if (faultV    && !pV   && cntFltV    < 255) cntFltV++;
    if (faultT    && !pT   && cntFltT    < 255) cntFltT++;
    if (faultNtc  && !pNtc && cntFltNtc  < 255) cntFltNtc++;
    if (faultComm && !pComm&& cntFltComm < 255) cntFltComm++;
    if (faultHall && !pHall&& cntFltHall < 255) cntFltHall++;
    if (faultInit && !pInit&& cntFltInit < 255) cntFltInit++;
    pV = faultV; pT = faultT; pNtc = faultNtc;
    pComm = faultComm; pHall = faultHall; pInit = faultInit;

    // !bmsInitOk fuerza fault para que BMS_OK quede en fallo (HIGH) sin
    // ventana de debounce al arrancar con init fallido (#3, opción B).
    bmsFault = faultV || faultT || faultNtc || faultComm || faultHall || faultInit;

    // Telemetría: duración del episodio (IDs 13/14) + primer trigger (ID 15 B4).
    // Y persistencia FRAM: log de FALL al inicio del episodio, RISE al final.
    if (bmsFault) {
        if (faultEpisodeStart == 0) {
            faultEpisodeStart = now;
            // Prioridad arbitraria: si varios entran a la vez, gana el de
            // mayor severidad de seguridad (V > T > NTC > Comm > Hall > Init).
            if      (faultV)     firstFaultTrigger = 1;
            else if (faultT)     firstFaultTrigger = 2;
            else if (faultNtc)   firstFaultTrigger = 3;
            else if (faultComm)  firstFaultTrigger = 4;
            else if (faultHall)  firstFaultTrigger = 5;
            else if (!bmsInitOk) firstFaultTrigger = 6;
            else                 firstFaultTrigger = 0;
            // Persistir en FRAM la transición BMS_OK ↓ (causa raíz + snapshot)
            logger.log(buildFaultRecord(FaultLogger::EVT_BMS_OK_FALL,
                                        firstFaultTrigger));
        }
        unsigned long dur = now - faultEpisodeStart;
        canLastFailMs = (dur > 65535UL) ? 65535 : (uint16_t)dur;
        if (canLastFailMs > canMaxFailMs) canMaxFailMs = canLastFailMs;
    } else {
        if (faultEpisodeStart != 0) {
            // Transición BMS_OK ↑ (despeje): log con el firstFault del
            // episodio que acaba (útil para correlacionar con el FALL).
            logger.log(buildFaultRecord(FaultLogger::EVT_BMS_OK_RISE,
                                        firstFaultTrigger));
        }
        faultEpisodeStart = 0;   // canLastFailMs se mantiene (último episodio)
        firstFaultTrigger = 0;
    }

    // Auto-rearma: cuando todo se despeja, BMS_OK vuelve a OK (LOW en esta
    // rama). El latch HW mantiene el SDC abierto hasta el reset manual humano (EV6.1.6).
    bms.setBmsOk(!bmsFault);
}

// ============================================================================
//  PRECARGA
// ============================================================================


// ============================================================================
//  Helpers de mapeo módulo → celdas/NTC del driver
//  Módulo m = 2 boards: par (2m) 6 celdas + 6 NTC; impar (2m+1) 5 celdas
//  + 3 NTC.  V1..V6 = par 0..5 ; V7..V11 = impar 0..4 ; T1..T6 = par
//  NTC 0..5 ; T7..T9 = impar NTC 0..2.
//  ✓ Mapeo Vn↔paralelo CONFIRMADO contra cableado (rev. 2026-05-20):
//    par antes que impar en el daisy-chain, canal 0 del BQ = paralelo
//    más bajo del rango que mide, canal 5 del impar sin usar (dummy
//    arriba). El antiguo dummy V10/V11 ya no aplica.
//  VTotal (campo 4 de ID 389) = suma de los 11 paralelos en mV →
//    debe coincidir con la tensión medida entre terminales del módulo.
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
//  Helpers debug (compartidos por CAN ID 15 y por el FaultLogger en FRAM)
// ============================================================================
// buildDebugFlags rellena los 4 bytes B0..B3 del CAN ID 15 (ver
// docs/CAN_Solo2DL.md §9). Uso desde ambos sitios garantiza coherencia
// entre lo que ves en el sniffer y lo que queda en FRAM.
static void buildDebugFlags(uint8_t out[4])
{
    out[0] = out[1] = out[2] = out[3] = 0;
    const unsigned long now = millis();

    // B0 — fallos CONFIRMADOS (drivers de bmsFault)
    if (fV.confirmed(now,  FAULT_V_MS))    out[0] |= (1 << 0);
    if (fT.confirmed(now,  FAULT_T_MS))    out[0] |= (1 << 1);
    if (fNtc.confirmed(now,FAULT_NTC_MS))  out[0] |= (1 << 2);
    if (fComm.confirmed(now,FAULT_COMM_MS))out[0] |= (1 << 3);
    if (!hall.isOK())                      out[0] |= (1 << 4);
    if (!bmsInitOk)                        out[0] |= (1 << 5);
    if (bmsFault)                          out[0] |= (1 << 7);

    // B1 — sub-fallos del Hall
    if (hall.isDisconnected())             out[1] |= (1 << 0);
    if (hall.isStuck())                    out[1] |= (1 << 1);
    if (hall.isNoisy())                    out[1] |= (1 << 2);
    if (hall.isOverCurrent())              out[1] |= (1 << 3);
    if (hall.isAdcSaturated())             out[1] |= (1 << 4);

    // B2 — snapshot V/T sin debounce
    if (bms.getMinVoltage() < CELL_UV_V)   out[2] |= (1 << 0);
    if (bms.getMaxVoltage() > CELL_OV_V)   out[2] |= (1 << 1);
    if (bms.getMinTemp()    < CELL_UT_C)   out[2] |= (1 << 2);
    if (bms.getMaxTemp()    > CELL_OT_C)   out[2] |= (1 << 3);
    if (bms.hasOpenNtc())                  out[2] |= (1 << 4);
    if (lastResV == BQResult::COMM_ERROR)  out[2] |= (1 << 5);
    if (lastResV == BQResult::CRC_ERROR)   out[2] |= (1 << 6);
    if (lastResT != BQResult::OK)          out[2] |= (1 << 7);

    // B3 — state
    if (bms.isOK())                        out[3] |= (1 << 0);
    if (bmsInitOk)                         out[3] |= (1 << 1);
    if (gCanOk)                            out[3] |= (1 << 2);
    if (digitalRead(PIN_SDC_3V3))          out[3] |= (1 << 6);
    if (digitalRead(PIN_VIO_3V3))          out[3] |= (1 << 7);
}

static FaultRecord buildFaultRecord(uint8_t eventType, uint8_t firstFlt)
{
    uint8_t flags[4];
    buildDebugFlags(flags);
    FaultRecord r;
    r.eventType  = eventType;
    r.firstFault = firstFlt;
    r.flagsConf  = flags[0];
    r.flagsHall  = flags[1];
    r.flagsSnap  = flags[2];
    r.flagsState = flags[3];
    r.resetCause = resetCauseSnapshot;
    r.minV_mV    = (uint16_t)lroundf(bms.getMinVoltage() * 1000.0f);
    r.maxV_mV    = (uint16_t)lroundf(bms.getMaxVoltage() * 1000.0f);
    r.maxT_C     = (int8_t)lroundf(bms.getMaxTemp());
    return r;
}

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
    if (!bmsFault)                                 st |= (1 << 1); // B1 BMSok (=!bmsFault)
    if (digitalRead(PIN_SDC_3V3))                  st |= (1 << 2); // B2 SDC presente
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

    // ── ID 12 (0xC) — GEN_STATUS_VOLT, GEN_STATUS_TEMP, V_DELTA UINT16×3 ────
    // Bitmap POR MÓDULO (alineado con el Excel CAN): cada bit i = módulo i.
    //   VOLT bit i = 1 si algún paralelo del módulo i fuera de [UV,OV].
    //   TEMP bit i = 1 si algún NTC del módulo i fuera de [UT,OT].
    // Los NUM_MODULES bits menos significativos son válidos (resto = 0).
    // Slot 3 (V_DELTA): delta de tensión de celda (máx−mín) en mV → indicador de
    // desbalanceo del pack. ⚠ DLC ahora 6 B (antes 4): ACTUALIZAR el Excel/receptor.
    uint16_t gsV = 0, gsT = 0;
    for (int m = 0; m < NUM_MODULES; m++) {
        bool vFail = false;
        for (int n = 1; n <= 11; n++) {
            float v = modCellV(m, n);
            if (v > CELL_OV_V || (v > 0.0f && v < CELL_UV_V)) { vFail = true; break; }
        }
        bool tFail = false;
        for (int k = 1; k <= 9; k++) {
            float t = modCellT(m, k);
            if (t > CELL_OT_C || t < CELL_UT_C) { tFail = true; break; }
        }
        if (vFail) gsV |= (1u << m);
        if (tFail) gsT |= (1u << m);
    }
    // getVoltageDelta() ya devuelve (vMax−vMin) en mV. Clamp a u16 por si una celda
    // abierta (0 V) dispara el delta; el valor real nunca pasa de unos pocos miles.
    float vDeltaMv = bms.getVoltageDelta();
    uint16_t vDelta = (vDeltaMv <= 0.0f) ? 0
                    : (vDeltaMv >= 65535.0f) ? 65535
                    : (uint16_t)lroundf(vDeltaMv);
    uint16_t d12[3] = { gsV, gsT, vDelta };
    gCan->setPacket((uint32_t)12, d12, 3);

    // ── ID 13 (0xD) — maxTotalFailTime(ms), numTriesResetComm UINT16×2 ─────
    uint16_t d13[2] = { canMaxFailMs, canNumTriesReset };
    gCan->setPacket((uint32_t)13, d13, 2);

    // ── ID 14 (0xE) — lastFailTime(ms), numCommFails, numCrcFails,
    //                  numTriesReset  UINT16×4 ─────────────────────────────
    uint16_t d14[4] = { canLastFailMs, canNumCommFails,
                        canNumCrcFails, canNumTriesReset };
    gCan->setPacket((uint32_t)14, d14, 4);

    // ── ID 15 (0xF) — BMS_DEBUG: TODOS los fallos por bits (8 bytes) ───────
    // Layout completo en ARQUITECTURA.md §7 (tabla BMS_DEBUG).
    //   B0 fallos confirmados (= drivers de bmsFault)
    //   B1 sub-fallos del Hall (causa raíz del bit Hall de B0)
    //   B2 snapshot V/T sin debounce (lo que pasa AHORA)
    //   B3 state (init/CAN/precharge/pines)
    //   B4 enum primer fallo del episodio actual
    //   B5-6 duración del episodio actual (u16 ms, big-endian)
    //   B7 causa del último reset (snapshot RCC->CSR del boot)
    uint8_t d15[8] = {0};

    // B0..B3 — flags via helper compartido con el FaultLogger.
    uint8_t flags[4];
    buildDebugFlags(flags);
    d15[0] = flags[0];
    d15[1] = flags[1];
    d15[2] = flags[2];
    d15[3] = flags[3];

    // B4 — Primer fallo del episodio actual (enum, ver updateBmsOk).
    d15[4] = firstFaultTrigger;

    // B5-6 — Duración del episodio actual (u16 ms, big-endian, sat 65535).
    {
        uint32_t dur = (faultEpisodeStart != 0)
                       ? (millis() - faultEpisodeStart) : 0UL;
        uint16_t durU16 = (dur > 65535UL) ? 65535 : (uint16_t)dur;
        d15[5] = (uint8_t)((durU16 >> 8) & 0xFF);
        d15[6] = (uint8_t)(durU16 & 0xFF);
    }

    // B7 — Causa del último reset (snapshot RCC->CSR del boot).
    d15[7] = resetCauseSnapshot;

    gCan->setPacket((uint32_t)15, d15, 8);

    // ── ID 16 (0x10) — contadores de fallo por causa (6×UINT8) ─────────────
    // nº de episodios de cada fallo desde el último reset (flanco de subida).
    // Post-mortem rápido: si no viste el fallo en vivo, mira qué contador subió.
    uint8_t d16[6] = { cntFltV, cntFltT, cntFltNtc,
                       cntFltComm, cntFltHall, cntFltInit };
    gCan->setPacket((uint32_t)16, d16, 6);

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
//  VOLCADO CSV POR MÓDULO (para APP_Registro_Modulos → Google Sheet)
// ============================================================================
// Emite por Serial el formato EXACTO que espera make_dataset.py:
//   Modulo;V1;…;V11;T1;…;T9   (separador ';', punto decimal, una fila por
//   módulo M01…M{NUM_MODULES}). El bloque va entre marcadores para que un
//   capturador serie en el PC lo detecte sin ambigüedad y lo postee al Apps
//   Script. Hace lecturas FRESCAS de V y T antes de volcar.
static void dumpModulesCsv()
{
    bool vOk = (bms.readVoltages()     == BQResult::OK);
    bool tOk = (bms.readTemperatures() == BQResult::OK);
    if (!vOk || !tOk)
        Serial.printf("[CSV] aviso: fallo lectura %s%s (valores pueden ser viejos)\n",
                      vOk ? "" : "V ", tOk ? "" : "T");

    Serial.println(F("<<<CSV_BEGIN>>>"));
    Serial.print(F("Modulo"));
    for (int n = 1; n <= 11; n++) Serial.printf(";V%d", n);
    for (int k = 1; k <= 9;  k++) Serial.printf(";T%d", k);
    Serial.println();
    for (int m = 0; m < NUM_MODULES; m++) {
        Serial.printf("M%02d", m + 1);
        for (int n = 1; n <= 11; n++) Serial.printf(";%.3f", modCellV(m, n));
        for (int k = 1; k <= 9;  k++) Serial.printf(";%.1f", modCellT(m, k));
        Serial.println();
    }
    Serial.println(F("<<<CSV_END>>>"));
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

    case 'g':
        // Volcado CSV por módulo (para APP_Registro_Modulos / Google Sheet).
        dumpModulesCsv();
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
        if (bms.reInit()) {
            bmsInitOk   = true;
            tLastReinit = millis();   // reset cadencia auto
            Serial.println(F("Re-init [OK]"));
        } else {
            Serial.println(F("Re-init [ERROR]"));
        }
        break;

    case 'r':
        Serial.println(F("Restart..."));
        delay(100);
        NVIC_SystemReset();
        break;

    case 'd':
        // Volcado del log persistente (FRAM). Pasamos un callback que
        // refresca el IWDG durante el dump (puede tardar varios segundos
        // si el log está lleno: ~18 s a 115200 baud para 2047 records).
        logger.dumpToSerial([]() { IWatchdog.reload(); });
        break;

    case 'D':
        // Reset del log (no borra datos físicos — solo el índice).
        // Confirmación implícita: D mayúscula es intencional.
        logger.clearLog();
        break;

    case 'C':
        // Reset de los contadores de fallo por causa (CAN ID 16).
        cntFltV = cntFltT = cntFltNtc = cntFltComm = cntFltHall = cntFltInit = 0;
        Serial.println(F("Contadores de fallo (ID 16) reseteados a 0."));
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
                  bmsFault ? "HIGH (FALLO)" : "LOW (OK)",
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
    Serial.printf("Cnt(ID16): V=%u T=%u NTC=%u COMM=%u HALL=%u INIT=%u  [C=reset]\n",
                  cntFltV, cntFltT, cntFltNtc, cntFltComm, cntFltHall, cntFltInit);
    
}
