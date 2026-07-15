/**
 * @file    pcb_bench.cpp
 * @brief   BANCO PCB — validacion de la cadena TSON / precarga / SDC
 *
 * Firmware MINIMO para el bring-up de la PCB nueva (STM32G474RE). NO es
 * firmware de seguridad: aqui BMS_OK lo decides TU por Serial, no el estado
 * del pack. Sirve para verificar el cableado y los aisladores de la placa
 * SIN bateria ni cadena BQ conectada.
 *
 * ── QUE LLEVA ───────────────────────────────────────────────────────────────
 *   · BMS_OK: arranca en HIGH y se cambia con comando serie (1/0/b).
 *   · Maquina de estados del TSON: IDENTICA a main.cpp::updateTson().
 *   · Latch de PRECHARGE_FAIL: IDENTICO a main.cpp (timeout 5 s, enclavado).
 *   · Volcado periodico de TODOS los pines usados (2 s) + eventos por flanco.
 *
 * ── QUE NO LLEVA (a proposito) ──────────────────────────────────────────────
 *   BQ79606, HallSensor, CAN, FRAM, SOC, ventiladores y watchdog. En una PCB
 *   sin cadena BQ, main.cpp deja BMS_OK LOW para siempre (bmsFault |=
 *   !bmsInitOk) y el TSON NUNCA armaria -> no se podria probar la placa.
 *   Ese es el motivo de que este banco exista.
 *
 * ⚠ NO MEZCLAR EN main.cpp: forzar BMS_OK=HIGH salta toda la vigilancia del
 *   pack (EV5.8). Este env es solo de banco, con el HV DESCONECTADO. Para
 *   probar la placa de verdad, flashear el env nucleo_g474re.
 *
 * ── COMANDOS SERIE ──────────────────────────────────────────────────────────
 *   1 = BMS_OK HIGH   0 = BMS_OK LOW   b = toggle BMS_OK
 *   s = status (pines)   r = reset del MCU (unica via de limpiar PRE_FAIL)
 *
 * Lanzar con:  pio run -e pcb_bench -t upload
 * Monitor:     pio device monitor -e pcb_bench
 */

#include <Arduino.h>

// ============================================================================
//  PINES — STM32G474RE (PCB BMS Master, rev. nueva). Espejo de main.cpp:65-86.
//  Si la PCB nueva remapea algo, cambiarlo AQUI y en main.cpp (o sacarlo a
//  lib/COMMON/common.h para no tenerlo en dos sitios).
// ============================================================================
#define PIN_BMS_OK          PB_5   ///< BMS_OK_STM (out) -> SDC. OK=HIGH. Aqui manual.

// SDC / TSON / precarga
#define PIN_TSON_FAIL       PB_8   ///< TSON_FAIL_STM (in). HIGH = fallo TSON
#define PIN_TSON_BTN        PB_9   ///< TSON_STM (in). Pulsador de arranque del TSON
#define PIN_SDC_TSON        PA_6   ///< SDC_TSON_STM (out). Latch del TSON
#define PIN_PRECHARGE_DONE  PA_7   ///< PRECHARGE_DONE_STM (in). HIGH = precarga OK
#define PIN_PRECHARGE_FAIL  PB_6   ///< PRECHARGE_FAIL_STM (out). HIGH enclavado si timeout 5 s
#define PIN_SDC_3V3         PC_7   ///< SDC_3V3_STM (in). HIGH = SDC presente
#define PIN_IMD_OK          PA_8   ///< IMD_OK_STM (in). Solo telemetria (aqui, solo print)
#define PIN_HV_ACCU_VIL     PB_4   ///< HV_ACCU_VIL_STM (in). Condicion de armado del TSON

// Igual que main.cpp: PRECHARGE_DONE debe llegar antes de esto o PRECHARGE_FAIL
// se enclava HIGH (solo se quita con reset de alimentacion / MCU).
#define PRECHARGE_TIMEOUT_MS  5000UL

#define PRINT_MS              2000UL

// ============================================================================
//  ESTADO
// ============================================================================
// bmsOk sustituye a !bmsFault de main.cpp: aqui NO lo decide el pack, lo
// decides tu por Serial. Arranca en HIGH para poder armar el TSON de entrada.
static bool          bmsOk            = true;

static bool          sdcTson          = false;  ///< estado del latch TSON (= nivel de PIN_SDC_TSON)
static bool          tsonBtnPrev      = false;  ///< nivel previo del boton (flanco de subida)
static bool          prechargeRunning = false;  ///< temporizador de precarga en marcha
static bool          prechargeFail    = false;  ///< latch HIGH si timeout 5 s (se quita con reset)
static unsigned long tPrechargeStart  = 0;

void updateTson();
void handleSerial();
void printStatus();
static void setBmsOk(bool ok);

// ============================================================================
//  SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    // Salidas en estado seguro ANTES de nada: SDC_TSON abierto, sin fallo de
    // precarga. BMS_OK se pone HIGH abajo (es lo que diferencia este banco).
    pinMode(PIN_SDC_TSON,       OUTPUT); digitalWrite(PIN_SDC_TSON,       LOW);
    pinMode(PIN_PRECHARGE_FAIL, OUTPUT); digitalWrite(PIN_PRECHARGE_FAIL, LOW);

    // Entradas SIN pull: vienen drivadas por el aislador ISO7742 (push-pull,
    // inactivo=LOW) y el boton pasa por un comparador -> flanco limpio. NO anadir
    // INPUT_PULLUP/DOWN (seria redundante y pelearia con el driver).
    pinMode(PIN_TSON_FAIL,      INPUT);
    pinMode(PIN_TSON_BTN,       INPUT);
    pinMode(PIN_PRECHARGE_DONE, INPUT);
    pinMode(PIN_SDC_3V3,        INPUT);
    pinMode(PIN_IMD_OK,         INPUT);
    pinMode(PIN_HV_ACCU_VIL,    INPUT);

    // BMS_OK arranca HIGH (banco). En main.cpp lo pone el driver del BQ en LOW
    // hasta que el init de la cadena va OK — aqui no hay cadena que esperar.
    pinMode(PIN_BMS_OK, OUTPUT);
    digitalWrite(PIN_BMS_OK, HIGH);
    bmsOk = true;

    // Estado real del boton al arrancar -> un boton pegado en HIGH al boot NO
    // se interpreta como flanco de subida (no auto-arma el TSON).
    tsonBtnPrev = digitalRead(PIN_TSON_BTN);

    Serial.println(F("========================================"));
    Serial.println(F("  BANCO PCB — TSON / PRECARGA / SDC"));
    Serial.println(F("  (BMS_OK MANUAL — no es firmware de seguridad)"));
    Serial.println(F("========================================"));
    Serial.println(F("Cmd: 1=BMS_OK HIGH  0=BMS_OK LOW  b=toggle  s=status  r=reset"));
    Serial.printf("BMS_OK arranca en HIGH. Precarga: timeout %lu ms.\n",
                  PRECHARGE_TIMEOUT_MS);
    printStatus();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop()
{
    updateTson();
    handleSerial();

    static unsigned long tPrint = 0;
    if (millis() - tPrint >= PRINT_MS) { tPrint = millis(); printStatus(); }
}

// ============================================================================
//  BMS_OK — manual (banco). En main.cpp esto es bms.setBmsOk(!bmsFault).
// ============================================================================
static void setBmsOk(bool ok)
{
    if (ok == bmsOk) return;              // escribir solo en los cambios
    bmsOk = ok;
    digitalWrite(PIN_BMS_OK, ok ? HIGH : LOW);   // OK=HIGH, fallo=LOW
    Serial.printf("[BMS_OK] -> %s\n", ok ? "HIGH (OK)" : "LOW (FALLO)");
}

// ============================================================================
//  TSON — maquina del Tractive System ON + precarga
//  COPIA FIEL de main.cpp::updateTson(). Unico cambio: la condicion de armado
//  usa !bmsOk (manual) donde main.cpp usa bmsFault (derivado del pack).
// ============================================================================
//  SDC_TSON (salida, latch):
//    · ARMA (LOW->HIGH) con el FLANCO DE SUBIDA del boton TSON, solo si en ese
//      instante: BMS_OK=HIGH Y HV_ACCU_VIL=LOW Y SDC_3V3=HIGH Y TSON_FAIL=LOW.
//    · SE MANTIENE mientras SDC_3V3=HIGH y TSON_FAIL=LOW.
//    · Si SDC_3V3 cae o TSON_FAIL sube -> desarma (hay que re-pulsar el boton).
//  PRECHARGE_FAIL (salida, latch DURO):
//    · Al armar SDC_TSON arranca un temporizador de 5 s.
//    · Si PRECHARGE_DONE no llega en 5 s -> HIGH ENCLAVADO (solo se quita con
//      reset de alimentacion / MCU; re-armar NO lo limpia).
// ============================================================================
void updateTson()
{
    bool sdc3v3   = digitalRead(PIN_SDC_3V3);
    bool tsonFail = digitalRead(PIN_TSON_FAIL);
    bool hvAccu   = digitalRead(PIN_HV_ACCU_VIL);
    bool tsonBtn  = digitalRead(PIN_TSON_BTN);

    // ── Latch SDC_TSON ──
    if (sdcTson && (!sdc3v3 || tsonFail)) {     // pierde condicion de mantenimiento
        sdcTson = false;
        Serial.println(F("[TSON] desarmado (SDC_3V3 bajo o TSON_FAIL)."));
    }
    bool btnRising = tsonBtn && !tsonBtnPrev;   // flanco de subida del boton
    if (!sdcTson && btnRising && bmsOk && !hvAccu && sdc3v3 && !tsonFail) {
        sdcTson = true;
        Serial.println(F("[TSON] armado."));
    } else if (!sdcTson && btnRising) {
        // Banco: si el boton llega pero NO arma, decir por que. En main.cpp esto
        // no existe (alli el motivo se deduce del CAN/status); aqui es lo que
        // hace util la placa en la mesa.
        Serial.printf("[TSON] boton IGNORADO: BMS_OK=%d HV_ACCU=%d SDC_3V3=%d TSON_FAIL=%d"
                      "  (arma con BMS_OK=1 HV_ACCU=0 SDC_3V3=1 TSON_FAIL=0)\n",
                      bmsOk, hvAccu, sdc3v3, tsonFail);
    }
    tsonBtnPrev = tsonBtn;
    digitalWrite(PIN_SDC_TSON, sdcTson ? HIGH : LOW);

    // ── Precarga: temporizador de 5 s desde el flanco SDC_TSON ↑ ──
    static bool sdcTsonPrev = false;
    if (sdcTson && !sdcTsonPrev) {               // flanco de armado
        // PRECHARGE_DONE NO puede estar HIGH antes de cerrar el TSON: si lo
        // esta, la entrada esta atascada (aislador/soldadura) -> no fiarse y
        // enclavar fallo, en vez de dar la precarga por hecha al instante.
        if (digitalRead(PIN_PRECHARGE_DONE)) {
            prechargeFail = true;
            Serial.println(F("[PRE] FALLO: PRECHARGE_DONE ya HIGH al armar (entrada atascada)."));
        } else {
            prechargeRunning = true;
            tPrechargeStart  = millis();
            Serial.println(F("[PRE] Precarga iniciada (5 s)."));
        }
    }
    if (!sdcTson) prechargeRunning = false;      // se cancela al desarmar (si no fallo)
    sdcTsonPrev = sdcTson;

    if (prechargeRunning && !prechargeFail) {
        if (digitalRead(PIN_PRECHARGE_DONE)) {
            prechargeRunning = false;            // precarga completada a tiempo
            Serial.println(F("[PRE] Precarga OK."));
        } else if ((millis() - tPrechargeStart) >= PRECHARGE_TIMEOUT_MS) {
            prechargeFail = true;                // LATCH duro -> reset de alimentacion
            Serial.println(F("[PRE] FALLO: precarga no completada en 5 s (enclavado)."));
            Serial.println(F("[PRE] Solo se limpia con reset ('r') o quitando alimentacion."));
        }
    }
    digitalWrite(PIN_PRECHARGE_FAIL, prechargeFail ? HIGH : LOW);
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

    case '1': setBmsOk(true);   break;
    case '0': setBmsOk(false);  break;
    case 'b': setBmsOk(!bmsOk); break;

    case 's': printStatus(); break;

    case 'r':
        // Unica via honesta de limpiar el latch de PRECHARGE_FAIL: equivale al
        // reset de alimentacion que exige main.cpp. NO se anade un comando que
        // lo baje "a mano" — eso probaria una logica que no es la de verdad.
        Serial.println(F("Restart..."));
        delay(100);
        NVIC_SystemReset();
        break;

    default: break;
    }
}

// ============================================================================
//  STATUS — vuelca TODOS los pines usados, con su nombre y sentido
// ============================================================================
void printStatus()
{
    Serial.println(F("\n=== BANCO PCB — PINES ==="));
    Serial.printf("BMS_OK_STM       PB5  (out) = %d  %s\n",
                  bmsOk, bmsOk ? "HIGH (OK)" : "LOW (FALLO)");
    Serial.printf("SDC_TSON_STM     PA6  (out) = %d  %s\n",
                  sdcTson, sdcTson ? "TSON ARMADO" : "abierto");
    Serial.printf("PRECHARGE_FAIL   PB6  (out) = %d  %s\n",
                  prechargeFail, prechargeFail ? "ENCLAVADO (reset para limpiar)" : "sin fallo");
    Serial.printf("TSON_STM (boton) PB9  (in)  = %d\n", digitalRead(PIN_TSON_BTN));
    Serial.printf("TSON_FAIL_STM    PB8  (in)  = %d\n", digitalRead(PIN_TSON_FAIL));
    Serial.printf("SDC_3V3_STM      PC7  (in)  = %d\n", digitalRead(PIN_SDC_3V3));
    Serial.printf("HV_ACCU_VIL_STM  PB4  (in)  = %d\n", digitalRead(PIN_HV_ACCU_VIL));
    Serial.printf("PRECHARGE_DONE   PA7  (in)  = %d\n", digitalRead(PIN_PRECHARGE_DONE));
    Serial.printf("IMD_OK_STM       PA8  (in)  = %d\n", digitalRead(PIN_IMD_OK));
    if (prechargeRunning)
        Serial.printf("[PRE] precarga en curso: %lu ms / %lu\n",
                      millis() - tPrechargeStart, PRECHARGE_TIMEOUT_MS);
    Serial.println(F("========================="));
}