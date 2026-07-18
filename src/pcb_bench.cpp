/**
 * @file    pcb_bench.cpp
 * @brief   BANCO PCB — validacion de la cadena TSON / precarga / SDC
 *
 * Firmware MINIMO para el bring-up de la PCB nueva (STM32G474RE). NO es
 * firmware de seguridad: aqui BMS_OK lo decides TU por Serial, no el estado
 * del pack. Sirve para verificar el cableado y los aisladores de la placa
 * SIN bateria ni cadena BQ conectada.
 *
 * ── PINES (PCB nueva — CONFIRMADOS con pin_walker / pin_out_walker) ───────────
 *   BMS_OK = PA4 (no PB5 como la placa vieja), boton TSON = PB9,
 *   SDC_TSON = PA6. El resto (TSON_FAIL, PRECHARGE_*, SDC_3V3, IMD_OK,
 *   HV_ACCU) siguen SIN confirmar contra el esquematico de la placa nueva.
 *
 * ── COMANDOS SERIE ──────────────────────────────────────────────────────────
 *   1 = BMS_OK HIGH   0 = BMS_OK LOW   b = toggle BMS_OK
 *   F = forzar SDC_TSON (prueba la SALIDA sola, sin logica ni boton)
 *   u/p/n = diag entradas: pull-up / pull-down / sin pull
 *   s = status (pines)   r = reset del MCU (unica via de limpiar PRE_FAIL)
 *
 * ⚠ NO MEZCLAR EN main.cpp: forzar BMS_OK salta la vigilancia del pack (EV5.8).
 *   Banco con HV DESCONECTADO. Para la placa de verdad, flashear nucleo_g474re.
 *
 * Lanzar con:  pio run -e pcb_bench -t upload
 * Monitor:     pio device monitor -e pcb_bench
 */

#include <Arduino.h>

// ============================================================================
//  PINES — STM32G474RE (PCB nueva). ⚠ pinout DISTINTO al de main.cpp.
// ============================================================================
// ⚠ SIN guion bajo (PA6, no PA_6): son nº de pin Arduino. Con PA_6 (PinName)
//   digitalWrite/pinMode apuntan a OTRA pata (lo interpreta como indice).
#define PIN_BMS_OK          PA_4    ///< BMS_OK_STM (out). CONFIRMADO PA4 (no PB5).

// SDC / TSON / precarga
#define PIN_TSON_FAIL       PB8    ///< TSON_FAIL_STM (in). HIGH = fallo TSON
#define PIN_TSON_BTN        PB9    ///< TSON_STM (in). Boton de arranque. CONFIRMADO PB9.
#define PIN_SDC_TSON        PA6    ///< SDC_TSON_STM (out). Latch del TSON. CONFIRMADO PA6.
#define PIN_PRECHARGE_DONE  PA7    ///< PRECHARGE_DONE_STM (in). HIGH = precarga OK
#define PIN_PRECHARGE_FAIL  PB6    ///< PRECHARGE_FAIL_STM (out). HIGH enclavado si timeout 5 s
#define PIN_SDC_3V3         PC7    ///< SDC_3V3_STM (in). HIGH = SDC presente
#define PIN_IMD_OK          PA8    ///< IMD_OK_STM (in). Solo print
#define PIN_HV_ACCU_VIL     PB4    ///< HV_ACCU_VIL_STM (in). Condicion de armado del TSON

#define PRECHARGE_TIMEOUT_MS  500000UL
#define PRINT_MS              2000UL

// ============================================================================
//  ESTADO
// ============================================================================
static bool          bmsOk            = true;   ///< OK manual (sustituye a !bmsFault)
static bool          forceTson        = false;  ///< 'F': fuerza SDC_TSON HIGH (test de salida)

static bool          sdcTson          = false;  ///< estado del latch TSON (= nivel de PIN_SDC_TSON)
static bool          tsonBtnPrev      = false;  ///< nivel previo del boton (flanco de subida)
static bool          prechargeRunning = false;  ///< temporizador de precarga en marcha
static bool          prechargeFail    = false;  ///< latch HIGH si timeout 5 s (se quita con reset)
static unsigned long tPrechargeStart  = 0;

// Las 6 entradas, para reconfigurar su pull de golpe (comandos u/p/n).
static const int INPUT_PINS[] = {
    PIN_TSON_FAIL, PIN_TSON_BTN, PIN_PRECHARGE_DONE,
    PIN_SDC_3V3, PIN_IMD_OK, PIN_HV_ACCU_VIL
};
static const int NUM_INPUTS = sizeof(INPUT_PINS) / sizeof(INPUT_PINS[0]);

void updateTson();
void handleSerial();
void printStatus();
static void setBmsOk(bool ok);
static void setInputPull(int mode, const char* nombre);

// ============================================================================
//  SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    // Salidas en estado seguro: SDC_TSON abierto (LOW), sin fallo de precarga.
    pinMode(PIN_SDC_TSON,       OUTPUT); digitalWrite(PIN_SDC_TSON,       LOW);
    pinMode(PIN_PRECHARGE_FAIL, OUTPUT); digitalWrite(PIN_PRECHARGE_FAIL, LOW);

    // Entradas con PULL-DOWN: en la PCB nueva el boton (y probablemente las
    // demas) no vienen drivadas push-pull en el banco -> flotarian sin pull.
    // Confirmado con pin_walker: PB9 solo se detecta con pull-down. u/p/n en
    // caliente para diagnosticar cada una.
    for (int i = 0; i < NUM_INPUTS; i++) pinMode(INPUT_PINS[i], INPUT_PULLDOWN);

    // BMS_OK arranca HIGH (banco). Manual, no lo decide ningun pack.
    pinMode(PIN_BMS_OK, OUTPUT);
    digitalWrite(PIN_BMS_OK, HIGH);
    bmsOk = true;

    // Nivel real del boton al arrancar -> no interpretar un boton ya pulsado
    // como flanco de subida.
    tsonBtnPrev = digitalRead(PIN_TSON_BTN);

    Serial.println(F("========================================"));
    Serial.println(F("  BANCO PCB — TSON / PRECARGA / SDC (PCB nueva)"));
    Serial.println(F("  (BMS_OK MANUAL — no es firmware de seguridad)"));
    Serial.println(F("========================================"));
    Serial.println(F("Cmd: 1=BMS_OK HIGH  0=BMS_OK LOW  b=toggle  F=forzar SDC_TSON"));
    Serial.println(F("     u=pull-up  p=pull-down  n=sin pull  s=status  r=reset"));
    printStatus();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop()
{
    handleSerial();

    // 'F' fuerza la SALIDA a HIGH para probarla sola (mide PA6). Si no, corre
    // la maquina TSON normal, que ya escribe SDC_TSON segun su estado.
    if (forceTson) digitalWrite(PIN_SDC_TSON, HIGH);
    else           updateTson();

    static unsigned long tPrint = 0;
    if (millis() - tPrint >= PRINT_MS) { tPrint = millis(); printStatus(); }
}

// ============================================================================
//  BMS_OK — manual (banco). En main.cpp esto es bms.setBmsOk(!bmsFault).
// ============================================================================
static void setBmsOk(bool ok)
{
    bmsOk = ok;
    digitalWrite(PIN_BMS_OK, ok ? HIGH : LOW);   // OK=HIGH, fallo=LOW
    Serial.printf("[BMS_OK] -> %s\n", ok ? "HIGH (OK)" : "LOW (FALLO)");
}

// ============================================================================
//  DIAGNOSTICO DE ENTRADAS — reconfigura el pull de las 6 entradas de golpe.
//  pull-down (p): si el pin lee 0 -> nadie lo drive (senal no llega).
//  pull-up   (u): si el pin lee 1 -> nadie lo drive.
//  si NO cambia con el pull -> algo lo drive fuerte (llega OK).
// ============================================================================
static void setInputPull(int mode, const char* nombre)
{
    for (int i = 0; i < NUM_INPUTS; i++) pinMode(INPUT_PINS[i], mode);
    delay(2);
    tsonBtnPrev = digitalRead(PIN_TSON_BTN);   // re-sync: no soltar flanco falso
    Serial.printf("[DIAG] entradas -> %s.\n", nombre);
    printStatus();
}

// ============================================================================
//  TSON — maquina del Tractive System ON + precarga
//  COPIA FIEL de main.cpp::updateTson(). Unico cambio: usa bmsOk (manual)
//  donde main.cpp usa !bmsFault.
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
        // Banco: si el boton llega pero NO arma, decir por que.
        Serial.printf("[TSON] boton IGNORADO: BMS_OK=%d HV_ACCU=%d SDC_3V3=%d TSON_FAIL=%d"
                      "  (arma con BMS_OK=1 HV_ACCU=0 SDC_3V3=1 TSON_FAIL=0)\n",
                      bmsOk, hvAccu, sdc3v3, tsonFail);
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
    if (!sdcTson) prechargeRunning = false;      // se cancela al desarmar (si no fallo)
    sdcTsonPrev = sdcTson;

    if (prechargeRunning && !prechargeFail) {
        if (digitalRead(PIN_PRECHARGE_DONE)) {
            prechargeRunning = false;            // precarga completada a tiempo
            Serial.println(F("[PRE] Precarga OK."));
        } else if ((millis() - tPrechargeStart) >= PRECHARGE_TIMEOUT_MS) {
            prechargeFail = true;                // LATCH duro -> reset de alimentacion
            Serial.println(F("[PRE] FALLO: precarga no completada en 5 s (enclavado)."));
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

    case 'F':   // fuerza la SALIDA SDC_TSON HIGH (test de pin, sin logica)
        forceTson = !forceTson;
        if (!forceTson) { sdcTson = false; digitalWrite(PIN_SDC_TSON, LOW); }
        Serial.printf("[FORCE] SDC_TSON forzado %s\n", forceTson ? "HIGH" : "OFF (vuelve la logica)");
        break;

    case 's': printStatus(); break;

    case 'u': setInputPull(INPUT_PULLUP,   "PULL-UP");   break;
    case 'p': setInputPull(INPUT_PULLDOWN, "PULL-DOWN"); break;
    case 'n': setInputPull(INPUT,          "SIN PULL");  break;

    case 'r':
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
    Serial.printf("BMS_OK_STM       PA4  (out) = %d  %s\n",
                  bmsOk, bmsOk ? "HIGH (OK)" : "LOW (FALLO)");
    Serial.printf("SDC_TSON_STM     PA6  (out) = %d  %s%s\n",
                  digitalRead(PIN_SDC_TSON), sdcTson ? "TSON ARMADO" : "abierto",
                  forceTson ? "  [FORZADO]" : "");
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