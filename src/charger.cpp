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
#include "FaultTimer.h"

// ============================================================================
//  PINES BQ79606 — idénticos a src/main.cpp (mismo hardware)
// ============================================================================
#define PIN_BQ_WAKE   PB_7
#define PIN_BQ_FAULT  PB_2
#define PIN_BQ_RX     PC_5
#define PIN_BQ_TX     PC_4
#define PIN_BMS_OK    PB_5

// Amperímetro DHAB S/118
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
#define CHG_TERM_VOLT_V      440.0f     ///< ⚠ Vmax fin de carga (valor del FW antiguo). FIJO: nunca por serial.
#define CHG_START_CURRENT_A  1.0f       ///< corriente DC al arrancar (la que ya sabes que funciona)
#define CHG_MAX_CURRENT_A    1.2f       ///< ⚠ TOPE duro: el serial no puede pedir más (atado al límite AC/plomos)

#define CELL_VMAX_HARD_V     4.25f      ///< corte DURO por celda (seguridad independiente de Vmax)
#define CELL_TMAX_CHG_C      45.0f      ///< ⚠ T máx para cargar 
#define CELL_TMIN_CHG_C      20.0f       ///< ⚠ T mín para cargar (Li-ion NO carga en frío)

#define MSG1_PERIOD_MS       1000UL     ///< cadencia de envío Message 1 (cargador corta a los 5 s sin él)
#define RX_TIMEOUT_MS        5000UL     ///< si no llega Message 2 en este tiempo → cargador mudo

#define WDG_TIMEOUT_US       8000000UL  ///< IWDG 8 s (igual que el BMS)

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


static CAN_BUS* gCan   = nullptr;
static bool     gCanOk = false;
static bool     bmsInitOk = false;
static FaultTimer fComm;
static FaultTimer fInit;


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

    // TX Message 1 cada 1 s — el cargador corta si no lo recibe en 5 s.
    static unsigned long tMsg1 = 0;
    if (millis() - tMsg1 >= MSG1_PERIOD_MS) {
        tMsg1 = millis();

        bool readOk = readPack();
        bool safe   = chargeAllowed(readOk);
        Serial.printf("[DBG] readOk=%d safe=%d bmsInit=%d\n",
              readOk, safe, bmsInitOk);
        bms.setBmsOk(safe);
Serial.printf("[PIN] PA4 real=%d (safe=%d)\n", digitalRead(PIN_BMS_OK), safe);
        // Un fallo CANCELA la orden de carga: hay que re-armar con 'g'.
        // (Il.begin();gual que el FW antiguo, que ponía cmdCharge=0 ante fallo.)
        if (!safe) chargeRequested = false;

        bool allow = chargeRequested && safe;
        sendMessage1(allow);
        charging = allow;

        // BMS_OK refleja la SEGURIDAD del pack (no si cargamos o no).
        // ⚠ Polaridad heredada del driver (rama testing: OK=LOW). Verificar
        //   que coincide con lo que espera el hardware de habilitación de carga.
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

    // Debounce de INIT
    fInit.sample(!bmsInitOk, now);

    if (!bmsInitOk) {
        bmsInitOk = bms.reInit();
        fInit.sample(!bmsInitOk, now);
        if (!bmsInitOk) return false;
    }

    bool okV = (bms.readVoltages()     == BQResult::OK);
    bool okT = (bms.readTemperatures() == BQResult::OK);

    bool readOk = okV && okT;

    // Debounce de COMM
    fComm.sample(!readOk, now);

    return !fComm.confirmed(now, 1000);   // 500 ms como en el BMS
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
    Serial.printf("[SAFE] pack OK: Vmax=%.3f V  Tmin=%.1f C  Tmax=%.1f C\n",
                  bms.getMaxVoltage(), bms.getMinTemp(), bms.getMaxTemp());
    return true;
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
