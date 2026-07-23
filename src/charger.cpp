/**
 * @file    charger.cpp
 * @brief   Firmware de CARGA — OBC TC HK-L 6.6kW (entorno [env:charger]).
 *
 *  Programa mínimo e INDEPENDIENTE del firmware de carrera (src/main.cpp).
 *  Reutiliza el driver BQ79606 (V/T de celda) y la librería MART_CAN.
 *  NO usa Hall / SOC / ventiladores / telemetría CAN del coche.
 *
 *  ── Protocolo OBC (J1939, IDs EXTENDIDOS, ver datasheet TC HK-L §7) ──────────
 *    BMS → Cargador  0x1806E5F4  cada 1 s:
 *      B0-1 Vmax (0.1 V/bit, big-endian)   B2-3 Imax (0.1 A/bit, big-endian)
 *      B4 control (0=cargar 1=parar)       B5-7 reservado
 *    Cargador → BMS  0x18FF50E5  cada 1 s:
 *      B0-1 Vout (0.1 V)  B2-3 Iout (0.1 A, bit15=signo)  B4 status  B5-7 resv
 *    Si el cargador NO recibe el frame del BMS en 5 s → corta la salida.
 *    (datasheet usa BYTE1..8 1-indexado; aquí byte index 0..7).
 *
 *  ── ESTRATEGIA: corriente DC FIJA ───────────────────────────────────────────
 *    El cargador hace el codo CV solo al llegar a Vmax. Nosotros NO regulamos
 *    la corriente para el límite AC (los plomos): por eso CHG_FIXED_CURRENT_A
 *    debe dimensionarse para no pasarse del límite AC NI con el pack lleno
 *    (el consumo AC sube con la tensión del pack — ver conversación).
 *
 *  ⚠⚠ TODOS los #define marcados CONFIRMAR son PROVISIONALES.
 *     NO flashear a un pack real sin validarlos en banco.
 */

#include <Arduino.h>
#include <IWatchdog.h>
#include "BQ79606.h"
#include "MART_CAN.h"
#include "FaultTimer.h"         // debounce K-de-N (mismo que main.cpp)
#include "HallSensor.h"         // amperímetro DHAB S/118 (mide corriente de carga)

// ============================================================================
//  PINES BQ79606 — idénticos a src/main.cpp (mismo hardware).
//  Estos van dentro de BQConfig y el DRIVER los convierte con BQ_DPIN() →
//  aquí SÍ es correcto el formato PA_x (PinName).
// ============================================================================
#define PIN_BQ_WAKE   PB_7
#define PIN_BQ_FAULT  PB_2
#define PIN_BQ_RX     PC_5
#define PIN_BQ_TX     PC_4
#define PIN_BMS_OK    PB_5

// ============================================================================
//  SDC / TSON / precarga (gestionados por el charger, como en main.cpp)
//  ⚠ FORMATO SIN guion bajo (PA6, no PA_6): estos pines se usan con
//    digitalRead/digitalWrite/pinMode DIRECTOS (sin BQ_DPIN). Con PA_6
//    (PinName) apuntarían a OTRA pata. Pines CONFIRMADOS contra el esquemático
//    de la PCB nueva (boton=PB9 y SDC_TSON=PA6 además con pin_walker).
// ============================================================================
#define PIN_TSON_FAIL       PB8    ///< TSON_FAIL_STM (in). HIGH = fallo TSON
#define PIN_TSON_BTN        PB9    ///< TSON_STM (in). Boton de arranque. CONFIRMADO PB9.
#define PIN_SDC_TSON        PA6    ///< SDC_TSON_STM (out). Latch del TSON. CONFIRMADO PA6.
#define PIN_PRECHARGE_DONE  PA7    ///< PRECHARGE_DONE_STM (in). HIGH = precarga OK
#define PIN_PRECHARGE_FAIL  PB6    ///< PRECHARGE_FAIL_STM (out). HIGH enclavado si timeout 5 s
#define PIN_SDC_3V3         PC7    ///< SDC_3V3_STM (in). HIGH = SDC presente
#define PIN_IMD_OK          PA8    ///< IMD_OK_STM (in). Solo print
#define PIN_HV_ACCU_VIL     PB4    ///< HV_ACCU_VIL_STM (in). Condicion de armado del TSON

// Amperímetro DHAB S/118 — idéntico a main.cpp (validado en HW). Formato PA_x:
// lo consume analogRead dentro de HallSensor, que SÍ funciona con PinName.
#define PIN_AMP_30A     PA_1   ///< Canal alta resolución (±30A)
#define PIN_AMP_350A    PA_0   ///< Canal baja resolución (±350A)

// ============================================================================
//  CONFIG DE CARGA  — ⚠ CONFIRMAR TODO antes de usar
// ============================================================================
#define CHG_CAN_BAUD         500        ///< kbps. Confirmado (igual que el FW antiguo: CAN_BUS(...,500,...)).
#define CHG_NODE_ID          3          ///< perfil de filtro MART_CAN (acepta todo)

// Vmax que se manda al cargador. El FW ANTIGUO (ESP32) cargaba a 456 V
// (controlCharge(456,...), con tope de seguridad en 555 V). Usamos ese valor
// PROBADO en vez de calcularlo por nº de serie (no quedó claro: los arrays
// eran [12][11]=132 puntos, pero indicaste 5S/módulo). ⚠ CONFIRMAR con
// multímetro la tensión del pack al 100 %.
#define CHG_TERM_VOLT_V      456.0f     ///< ⚠ Vmax fin de carga (valor del FW antiguo). FIJO: nunca por serial.
#define CHG_START_CURRENT_A  3.0f       ///< corriente DC al arrancar (la que ya sabes que funciona)
#define CHG_MAX_CURRENT_A    4.0f       ///< ⚠ TOPE duro: el serial no puede pedir más (atado al límite AC/plomos)

#define CELL_VMAX_HARD_V     4.25f      ///< corte DURO por celda (seguridad independiente de Vmax)
#define CELL_TMAX_CHG_C      45.0f      ///< ⚠ T máx para cargar (el FW antiguo usaba 40)
#define CELL_TMIN_CHG_C      0.0f       ///< ⚠ T mín para cargar (Li-ion NO carga en frío)

#define MSG1_PERIOD_MS       1000UL     ///< cadencia de envío Message 1 (cargador corta a los 5 s sin él)
#define RX_TIMEOUT_MS        5000UL     ///< si no llega Message 2 en este tiempo → cargador mudo

#define WDG_TIMEOUT_US       8000000UL  ///< IWDG 8 s (igual que el BMS)

// Debounce de comms del BQ: un error de lectura suelto (ruido) NO cuenta como
// fallo; debe persistir esta ventana. Mismo criterio que main.cpp (2 s).
#define FAULT_COMM_MS        2000UL

// Tras armar SDC_TSON, PRECHARGE_DONE debe llegar antes de esto o
// PRECHARGE_FAIL se enclava HIGH (solo se quita con reset de alimentación/MCU).
#define PRECHARGE_TIMEOUT_MS 5000UL

// IDs J1939 extendidos
#define ID_BMS_TO_CHG        0x1806E5F4UL   ///< Message 1 (BMS → cargador)
#define ID_CHG_TO_BMS        0x18FF50E5UL   ///< Message 2 (cargador → BMS)

// Bits del byte STATUS (B4 del Message 2)
#define ST_HW_FAIL    (1 << 0)
#define ST_OVERTEMP   (1 << 1)
#define ST_INPUT_V    (1 << 2)
#define ST_BAT_CONN   (1 << 3)   ///< batería no conectada / invertida
#define ST_COMM_TO    (1 << 4)   ///< timeout de comunicación

// ============================================================================
//  OBJETOS
// ============================================================================
static const BQConfig bqCfg = {
    .uartPort    = 0,
    .pinWake     = PIN_BQ_WAKE,
    .pinFault    = PIN_BQ_FAULT,
    .pinBmsOk    = PIN_BMS_OK,
    .pinRx       = PIN_BQ_RX,
    .pinTx       = PIN_BQ_TX,
    .pinTxEnable = -1,
    .baudrate    = 125000
};
static BQ79606  bms(bqCfg);

// Amperímetro: mide la corriente de carga; sus fallos (desconexión, stuck,
// ruido, sobre-I, ADC saturado) cortan la carga vía chargeAllowed().
static HallSensor hall(PIN_AMP_30A, PIN_AMP_350A);

static CAN_BUS* gCan   = nullptr;
static bool     gCanOk = false;
static bool     bmsInitOk = false;

// Debounce de comms (mismo FaultTimer que main.cpp).
static FaultTimer fComm;

// SDC / TSON / precarga (misma máquina que main.cpp::updateTson).
static bool          bmsSafe          = false;  ///< última evaluación de seguridad (= safe del loop)
static bool          sdcTson          = false;  ///< estado del latch TSON (= nivel de PIN_SDC_TSON)
static bool          tsonBtnPrev      = false;  ///< nivel previo del botón (flanco de subida)
static bool          prechargeRunning = false;  ///< temporizador de precarga en marcha
static bool          prechargeFail    = false;  ///< latch HIGH si timeout 5 s (se quita con reset)
static unsigned long tPrechargeStart  = 0;

// Estado de carga
static bool          chargeRequested = false;  ///< orden start/stop por serial (ARRANCA SIN cargar)
static float         chgCurrentA     = CHG_START_CURRENT_A;  ///< corriente DC comandada (ajustable, capada)
static bool          charging        = false;  ///< true mientras mandamos control=0
static unsigned long tLastChgRx      = 0;       ///< ms del último Message 2 recibido
static uint8_t       chgStatus       = 0;       ///< último byte STATUS del cargador
static float         chgOutV         = 0.0f;    ///< V de salida reportada
static float         chgOutI         = 0.0f;    ///< I de salida reportada

// ============================================================================
//  PROTOTIPOS
// ============================================================================
bool   readPack();
bool   chargeAllowed(bool readOk);
void   updateTson();
void   sendMessage1(bool allow);
void   pollMessage2();
void   printChgStatus();
void   printVoltages();
void   printTemps();
void   handleSerial();

// ============================================================================
//  SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(50);   // que readBytesUntil no bloquee el loop esperando
    delay(50);
    Serial.println(F("\n=== BMS CHARGER FW (TC HK-L 6.6kW) ==="));
    Serial.println(F("⚠ Config PROVISIONAL — verificar #define antes de cargar."));

    bmsInitOk = bms.begin();
    Serial.println(bmsInitOk ? F("[OK] BQ79606 listo.")
                             : F("[WARN] BQ init FALLO."));

    // Hall: autocalibración de offset (~1 s, con corriente ~0 → aún sin cargar).
    hall.begin();
    Serial.println(hall.isOK() ? F("[OK] Hall listo.")
                               : F("[WARN] Hall calibración fuera de rango."));

    // Cadena TSON/precarga en estado seguro: SDC_TSON abierto, sin fallo de
    // precarga. Entradas con PULL-DOWN (en la PCB nueva el botón lo necesita;
    // confirmado con pin_walker). Salidas y entradas usan formato PAx.
    pinMode(PIN_SDC_TSON,       OUTPUT); digitalWrite(PIN_SDC_TSON,       LOW);
    pinMode(PIN_PRECHARGE_FAIL, OUTPUT); digitalWrite(PIN_PRECHARGE_FAIL, LOW);
    pinMode(PIN_TSON_FAIL,      INPUT_PULLDOWN);
    pinMode(PIN_TSON_BTN,       INPUT_PULLDOWN);
    pinMode(PIN_PRECHARGE_DONE, INPUT_PULLDOWN);
    pinMode(PIN_SDC_3V3,        INPUT_PULLDOWN);
    pinMode(PIN_IMD_OK,         INPUT_PULLDOWN);
    pinMode(PIN_HV_ACCU_VIL,    INPUT_PULLDOWN);
    tsonBtnPrev = digitalRead(PIN_TSON_BTN);   // no interpretar botón ya pulsado como flanco

    static CAN_BUS canBus(HardwareType::Transciever, CHG_CAN_BAUD, CHG_NODE_ID);
    gCan   = &canBus;
    gCanOk = (gCan->SetupState() == 0);
    Serial.println(gCanOk ? F("[CAN] FDCAN listo.")
                          : F("[CAN] FDCAN init FALLO."));

    Serial.printf("Vmax: %.1f V (fijo)  I_inicio: %.1f A (tope %.1f A)\n",
                  CHG_TERM_VOLT_V, CHG_START_CURRENT_A, CHG_MAX_CURRENT_A);
    Serial.println(F("ARRANCA SIN CARGAR. Comandos: g=start x=stop c,<I>=corriente v=voltajes t=temps d=datos r=restart"));

    IWatchdog.begin(WDG_TIMEOUT_US);
    tLastChgRx = millis();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop()
{
    // Comandos serie (start/stop, corriente, datos).
    handleSerial();

    // RX continua (el cargador emite Message 2 cada 1 s).
    pollMessage2();

    // Amperímetro cada vuelta (máxima resolución + debounce interno de fallos).
    hall.update();

    // Máquina TSON + precarga: cada vuelta (flanco del botón, timing precarga).
    // Usa bmsSafe, que se refresca en el bloque de 1 s de abajo.
    updateTson();

    // TX Message 1 cada 1 s — el cargador corta si no lo recibe en 5 s.
    static unsigned long tMsg1 = 0;
    if (millis() - tMsg1 >= MSG1_PERIOD_MS) {
        tMsg1 = millis();

        bool readOk = readPack();
        bool safe   = chargeAllowed(readOk);
        bmsSafe     = safe;   // lo consume updateTson() como condición de armado

        // Un fallo CANCELA la orden de carga: hay que re-armar con 'g'.
        // (Igual que el FW antiguo, que ponía cmdCharge=0 ante fallo.)
        if (!safe) chargeRequested = false;

        bool allow = chargeRequested && safe;
        sendMessage1(allow);
        charging = allow;

        // BMS_OK refleja la SEGURIDAD del pack (no si cargamos o no).
        // Polaridad del driver: OK=HIGH, fallo=LOW (fail-safe).
        bms.setBmsOk(safe);

        printChgStatus();
    }

    IWatchdog.reload();
}

// ============================================================================
//  Lectura del pack (V/T de celda vía BQ79606)
// ============================================================================
bool readPack()
{
    unsigned long now = millis();

    if (!bmsInitOk) {
        // Sin init NO hay datos válidos → no se puede cargar (no se debouncea).
        bmsInitOk = bms.reInit();
        if (!bmsInitOk) return false;
    }
    bool okV = (bms.readVoltages()     == BQResult::OK);
    bool okT = (bms.readTemperatures() == BQResult::OK);

    // Debounce de comms (mismo criterio que main.cpp): un error de lectura
    // suelto (ruido) NO cuenta como fallo; solo si persiste FAULT_COMM_MS.
    // Ante un glitch se sigue con la última medida buena durante la ventana.
    fComm.sample(!(okV && okT), now);
    return !fComm.confirmed(now, FAULT_COMM_MS);
}

// ============================================================================
//  ¿Se permite cargar?  (corte por celda/temperatura/lectura)
// ============================================================================
bool chargeAllowed(bool readOk)
{
    if (!readOk) {
        Serial.println(F("[SAFE] lectura BQ fallida → parar carga."));
        return false;
    }
    if (bms.getMaxVoltage() >= CELL_VMAX_HARD_V) {
        Serial.printf("[SAFE] celda %.3f V >= %.2f V (OV) → parar.\n",
                      bms.getMaxVoltage(), CELL_VMAX_HARD_V);
        return false;
    }
    if (bms.getMaxTemp() >= CELL_TMAX_CHG_C) {
        Serial.printf("[SAFE] T %.1f C >= %.1f → parar.\n",
                      bms.getMaxTemp(), CELL_TMAX_CHG_C);
        return false;
    }
    if (bms.getMinTemp() <= CELL_TMIN_CHG_C) {
        Serial.printf("[SAFE] T %.1f C <= %.1f (frío) → parar.\n",
                      bms.getMinTemp(), CELL_TMIN_CHG_C);
        return false;
    }
    // Amperímetro: fallo confirmado (desconexión, stuck, ruido, sobre-I, ADC
    // saturado) → no fiarse de la medida de corriente de carga → parar.
    // isOK() ya viene con el debounce propio del HallSensor.
    if (!hall.isOK()) {
        Serial.printf("[SAFE] Hall FALLO (%s%s%s%s%s) → parar.\n",
                      hall.isDisconnected() ? "desc "   : "",
                      hall.isStuck()        ? "stuck "  : "",
                      hall.isNoisy()        ? "noisy "  : "",
                      hall.isOverCurrent()  ? "sobreI " : "",
                      hall.isAdcSaturated() ? "adcSat"  : "");
        return false;
    }
    return true;
}

// ============================================================================
//  TSON — máquina del Tractive System ON + precarga (copia de main.cpp).
//  Único cambio: usa bmsSafe (seguridad del pack) donde main usa !bmsFault, y
//  no persiste en FRAM (el charger no lleva FaultLogger).
//    SDC_TSON (out, latch): ARMA con el flanco del botón solo si bmsSafe Y
//      HV_ACCU=LOW Y SDC_3V3=HIGH Y TSON_FAIL=LOW; se mantiene mientras
//      SDC_3V3 y !TSON_FAIL. PRECHARGE_FAIL (latch duro): si PRECHARGE_DONE no
//      llega en 5 s tras armar → HIGH enclavado (solo reset de alimentación).
// ============================================================================
void updateTson()
{
    bool sdc3v3   = digitalRead(PIN_SDC_3V3);
    bool tsonFail = digitalRead(PIN_TSON_FAIL);
    bool hvAccu   = digitalRead(PIN_HV_ACCU_VIL);
    bool tsonBtn  = digitalRead(PIN_TSON_BTN);

    // ── Latch SDC_TSON ──
    if (sdcTson && (!sdc3v3 || tsonFail)) {     // pierde condición de mantenimiento
        sdcTson = false;
        Serial.println(F("[TSON] desarmado (SDC_3V3 bajo o TSON_FAIL)."));
    }
    bool btnRising = tsonBtn && !tsonBtnPrev;   // flanco de subida del botón
    if (!sdcTson && btnRising && bmsSafe && !hvAccu && sdc3v3 && !tsonFail) {
        sdcTson = true;
        Serial.println(F("[TSON] armado."));
    } else if (!sdcTson && btnRising) {
        Serial.printf("[TSON] boton IGNORADO: SAFE=%d HV_ACCU=%d SDC_3V3=%d TSON_FAIL=%d\n",
                      bmsSafe, hvAccu, sdc3v3, tsonFail);
    }
    tsonBtnPrev = tsonBtn;
    digitalWrite(PIN_SDC_TSON, sdcTson ? HIGH : LOW);

    // ── Precarga: temporizador de 5 s desde el flanco SDC_TSON ↑ ──
    static bool sdcTsonPrev = false;
    if (sdcTson && !sdcTsonPrev) {               // flanco de armado
        if (digitalRead(PIN_PRECHARGE_DONE)) {
            prechargeFail = true;
            Serial.println(F("[PRE] FALLO: PRECHARGE_DONE ya HIGH al armar (entrada atascada)."));
        } else {
            prechargeRunning = true;
            tPrechargeStart  = millis();
            Serial.println(F("[PRE] Precarga iniciada (5 s)."));
        }
    }
    if (!sdcTson) prechargeRunning = false;      // se cancela al desarmar (si no falló)
    sdcTsonPrev = sdcTson;

    if (prechargeRunning && !prechargeFail) {
        if (digitalRead(PIN_PRECHARGE_DONE)) {
            prechargeRunning = false;            // precarga completada a tiempo
            Serial.println(F("[PRE] Precarga OK."));
        } else if ((millis() - tPrechargeStart) >= PRECHARGE_TIMEOUT_MS) {
            prechargeFail = true;                // LATCH duro → reset de alimentación
            Serial.println(F("[PRE] FALLO: precarga no completada en 5 s (enclavado)."));
        }
    }
    digitalWrite(PIN_PRECHARGE_FAIL, prechargeFail ? HIGH : LOW);
}

// ============================================================================
//  Message 1: BMS → Cargador (0x1806E5F4)
//  B0-1 Vmax (0.1 V/bit, BE) · B2-3 Imax (0.1 A/bit, BE) · B4 control · B5-7 0
// ============================================================================
void sendMessage1(bool allow)
{
    if (!gCanOk) return;

    // Al parar, el FW antiguo manda V=0 e I=0 además del byte de control.
    uint16_t vSet = allow ? (uint16_t)lroundf(CHG_TERM_VOLT_V * 10.0f) : 0;
    uint16_t iSet = allow ? (uint16_t)lroundf(chgCurrentA     * 10.0f) : 0;

    uint8_t d[8] = {0};
    d[0] = (uint8_t)(vSet >> 8);    // Vmax high byte
    d[1] = (uint8_t)(vSet & 0xFF);  // Vmax low byte
    d[2] = (uint8_t)(iSet >> 8);    // Imax high byte
    d[3] = (uint8_t)(iSet & 0xFF);  // Imax low byte
    d[4] = allow ? 0x00 : 0x01;     // control: 0=cargar 1=parar
    // d[5..7] reservado (0)

    // Bytes crudos (sin conversión): controlamos el layout completo.
    gCan->setPacket(ID_BMS_TO_CHG, d, 8, false);
    gCan->send();
}

// ============================================================================
//  Message 2: Cargador → BMS (0x18FF50E5)
// ============================================================================
void pollMessage2()
{
    if (!gCanOk) return;
    gCan->receive();

    uint8_t r[8];
    if (gCan->getPacket(ID_CHG_TO_BMS, r, 8, false)) {
        uint16_t vRaw =  ((uint16_t)r[0] << 8) | r[1];
        uint16_t iRaw = (((uint16_t)r[2] << 8) | r[3]) & 0x7FFF;  // bit15 = signo
        chgOutV   = vRaw * 0.1f;
        chgOutI   = iRaw * 0.1f;
        chgStatus = r[4];
        tLastChgRx = millis();
    }
}

// ============================================================================
//  STATUS por serie
// ============================================================================
void printChgStatus()
{
    bool rxAlive = (millis() - tLastChgRx) < RX_TIMEOUT_MS;
    Serial.println(F("\n--- CHARGER ---"));
    Serial.printf("Pack: Vmin=%.3f Vmax=%.3f  Tmin=%.1f Tmax=%.1f\n",
                  bms.getMinVoltage(), bms.getMaxVoltage(),
                  bms.getMinTemp(), bms.getMaxTemp());
    Serial.printf("Solicitado: %s  I_cmd=%.1f A  →  Cargando: %s (control=%d)\n",
                  chargeRequested ? "SI" : "NO", chgCurrentA,
                  charging ? "SI" : "NO", charging ? 0 : 1);
    Serial.printf("TSON: SDC_TSON=%d PRE_FAIL=%d | SDC_3V3=%d TSON_FAIL=%d HV_ACCU=%d IMD_OK=%d\n",
                  sdcTson, prechargeFail,
                  digitalRead(PIN_SDC_3V3), digitalRead(PIN_TSON_FAIL),
                  digitalRead(PIN_HV_ACCU_VIL), digitalRead(PIN_IMD_OK));
    Serial.printf("Hall: %.2f A (%s)  %s\n",
                  hall.getCurrent(), hall.isLowRange() ? "30A" : "350A",
                  hall.isOK() ? "OK" : "FALLO");
    if (rxAlive) {
        Serial.printf("OBC: Vout=%.1f V  Iout=%.1f A  st=0x%02X%s%s%s%s%s\n",
                      chgOutV, chgOutI, chgStatus,
                      (chgStatus & ST_HW_FAIL)  ? " HWFAIL"   : "",
                      (chgStatus & ST_OVERTEMP) ? " OVERTEMP" : "",
                      (chgStatus & ST_INPUT_V)  ? " INPUT_V"  : "",
                      (chgStatus & ST_BAT_CONN) ? " BAT_CONN" : "",
                      (chgStatus & ST_COMM_TO)  ? " COMM_TO"  : "");
    } else {
        Serial.println(F("OBC: sin respuesta (>5 s) — ¿bus/baudrate/cargador apagado?"));
    }
}

// ============================================================================
//  Volcado de la tabla completa de V/T (comandos 'v' / 't')
//  Hacen una lectura fresca del BQ y vuelcan board por board.
// ============================================================================
void printVoltages()
{
    if (bms.readVoltages() != BQResult::OK) {
        Serial.println(F("[V] lectura BQ fallida."));
        return;
    }
    Serial.println(F("\n--- VOLTAJES por celda (V) ---"));
    for (uint8_t b = 0; b < TOTALBOARDS; b++) {
        Serial.printf("Board %2u:", b);
        for (uint8_t c = 0; c < CELLS_FOR_BOARD(b); c++)
            Serial.printf(" %.3f", bms.getVoltage(b, c));
        Serial.println();
    }
    Serial.printf("Vmin=%.3f  Vmax=%.3f  dV=%.0f mV\n",
                  bms.getMinVoltage(), bms.getMaxVoltage(), bms.getVoltageDelta());
}

void printTemps()
{
    if (bms.readTemperatures() != BQResult::OK) {
        Serial.println(F("[T] lectura BQ fallida."));
        return;
    }
    Serial.println(F("\n--- TEMPERATURAS por NTC (C) ---"));
    for (uint8_t b = 0; b < TOTALBOARDS; b++) {
        Serial.printf("Board %2u:", b);
        for (uint8_t k = 0; k < NTCS_PER_BOARD[b % 2]; k++)
            Serial.printf(" %.1f", bms.getTemperature(b, k));
        Serial.println();
    }
    Serial.printf("Tmin=%.1f  Tmax=%.1f\n", bms.getMinTemp(), bms.getMaxTemp());
}

// ============================================================================
//  COMANDOS SERIE
//    g       → solicitar START de carga
//    x       → STOP
//    c,<I>   → fijar corriente DC (A), capada a CHG_MAX_CURRENT_A
//    v       → tabla de voltajes por celda
//    t       → tabla de temperaturas por NTC
//    d       → volcar estado
//    r       → reiniciar MCU
// ============================================================================
void handleSerial()
{
    if (!Serial.available()) return;

    char buf[24];
    size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
    buf[n] = '\0';
    if (n && buf[n - 1] == '\r') buf[--n] = '\0';
    if (n == 0) return;

    // c,<corriente>  → fijar corriente (con tope duro)
    if (buf[0] == 'c' && buf[1] == ',') {
        float v = atof(buf + 2);
        if (v < 0.0f) v = 0.0f;
        if (v > CHG_MAX_CURRENT_A) {
            Serial.printf("[I] %.1f A > tope %.1f A → capado.\n", v, CHG_MAX_CURRENT_A);
            v = CHG_MAX_CURRENT_A;
        }
        chgCurrentA = v;
        Serial.printf("[I] corriente comandada = %.1f A\n", chgCurrentA);
        return;
    }

    if (n != 1) return;   // resto: solo comandos de 1 carácter (ignora ruido)

    switch (buf[0]) {
    case 'g':
        chargeRequested = true;
        Serial.printf("[CHG] START solicitado (%.1f A). La carga arranca si la seguridad da OK.\n",
                      chgCurrentA);
        break;
    case 'x':
        chargeRequested = false;
        Serial.println(F("[CHG] STOP."));
        break;
    case 'v':
        printVoltages();
        break;
    case 't':
        printTemps();
        break;
    case 'd':
        printChgStatus();
        break;
    case 'r':
        Serial.println(F("Restart..."));
        delay(100);
        NVIC_SystemReset();
        break;
    default:
        break;
    }
}
