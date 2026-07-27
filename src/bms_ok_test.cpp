/**
 *  bms_ok_test.cpp — firmware minimo: BMS_OK fijo + control TSON completo.
 *
 *  Sirve para verificar en banco los caminos BMS_OK_STM → SDC, SDC_TSON_STM →
 *  latch del TSON y la precarga, sin arrancar el BQ ni el CAN.
 *   · BMS_OK  : HIGH fijo mientras el micro este vivo.
 *   · SDC_TSON / PRECHARGE_FAIL: updateTson(), copia de la de main.cpp salvo
 *     que aqui se IGNORAN bmsFault (no hay BQ), TSON_FAIL y HV_ACCU_VIL, y no
 *     se escribe el FaultLogger. El resto (latches, precarga, timeout) es
 *     identico: la unica condicion que queda es SDC_3V3.
 *
 *  ⚠ OJO CON EL FORMATO DE LOS PINES (ver board_pins.h):
 *   · PIN_BMS_OK es PinName (PB_5, CON guion) porque en produccion lo consume
 *     el driver BQ via BQ_DPIN(). pinMode/digitalWrite NO aceptan PinName: lo
 *     interpretan como INDICE (PB_5 = 0x15 = 21 → D21 = PB_7 = BQ_WAKE). Hay
 *     que convertirlo con pinNametoDigitalPin(), igual que hace BQ_DPIN.
 *   · Los pines de SDC/TSON son nº Arduino (SIN guion) → uso DIRECTO.
 *
 *  Lanzar con:  pio run -e bms_ok_test -t upload
 *  Monitor:     pio device monitor -e bms_ok_test
 */
#include <Arduino.h>
#include "board_pins.h"

#define PRECHARGE_TIMEOUT_MS  5000UL   ///< igual que main.cpp

static uint32_t bmsOkPin;                    ///< nº de pin Arduino de PIN_BMS_OK (PB5 → D4)
static bool     sdcTson          = false;    ///< estado del latch TSON (= nivel de PIN_SDC_TSON)
static bool     tsonBtnPrev      = false;    ///< nivel previo del boton (flanco de subida)
static bool     prechargeRunning = false;    ///< precarga en curso (temporizador armado)
static bool     prechargeFail    = false;    ///< latch HIGH si timeout 5 s (solo lo quita un reset)
static uint32_t tPrechargeStart  = 0;        ///< millis() del flanco de armado

void updateTson();

void setup()
{
    Serial.begin(115200);

    bmsOkPin = pinNametoDigitalPin(PIN_BMS_OK);
    pinMode(bmsOkPin, OUTPUT);
    digitalWrite(bmsOkPin, HIGH);

    // Salidas en estado seguro: SDC_TSON abierto, PRECHARGE_FAIL sin fallo.
    pinMode(PIN_SDC_TSON,       OUTPUT); digitalWrite(PIN_SDC_TSON,       LOW);
    pinMode(PIN_PRECHARGE_FAIL, OUTPUT); digitalWrite(PIN_PRECHARGE_FAIL, LOW);

    // Entradas con PULL-DOWN, igual que main.cpp: en esta PCB el boton solo se
    // detecta con pull-down; sin el la pata flota.
    // TSON_FAIL y HV_ACCU_VIL no se leen (ignorados a proposito), por eso no
    // se configuran aqui.
    pinMode(PIN_TSON_BTN,       INPUT_PULLDOWN);
    pinMode(PIN_PRECHARGE_DONE, INPUT_PULLDOWN);
    pinMode(PIN_SDC_3V3,        INPUT_PULLDOWN);

    // Estado real del boton al arrancar → un boton pegado en HIGH al boot NO
    // cuenta como flanco de subida (no auto-arma el TSON).
    tsonBtnPrev = digitalRead(PIN_TSON_BTN);

    Serial.println(F("[BOOT] bms_ok_test — BMS_OK=HIGH, TSON completo (sin BQ)."));
}

void loop()
{
    digitalWrite(bmsOkPin, HIGH);
    updateTson();
    delay(20);   // antirrebote basico del pulsador
}

// ============================================================================
//  TSON — maquina del Tractive System ON + precarga (copia de main.cpp)
// ============================================================================
//  SDC_TSON (salida, latch):
//    · ARMA (LOW→HIGH) con el FLANCO DE SUBIDA del boton TSON, solo si en ese
//      instante SDC_3V3=HIGH.
//      main.cpp exige ademas BMS_OK=HIGH, HV_ACCU_VIL=LOW y TSON_FAIL=LOW:
//      aqui las tres se IGNORAN a proposito (banco, sin BQ).
//    · SE MANTIENE mientras SDC_3V3=HIGH.
//    · Si SDC_3V3 cae → desarma (hay que re-pulsar el boton).
//  PRECHARGE_FAIL (salida, latch DURO):
//    · Al armar SDC_TSON arranca un temporizador de 5 s.
//    · Si PRECHARGE_DONE no llega en 5 s → HIGH ENCLAVADO (solo se quita con
//      reset de alimentacion / MCU; re-armar NO lo limpia).
// ============================================================================
void updateTson()
{
    bool sdc3v3  = digitalRead(PIN_SDC_3V3);
    bool tsonBtn = digitalRead(PIN_TSON_BTN);

    // ── Latch SDC_TSON ──
    if (sdcTson && !sdc3v3) {                   // pierde condicion de mantenimiento
        sdcTson = false;
        Serial.println(F("[TSON] desarmado (SDC_3V3 bajo)."));
    }
    bool btnRising = tsonBtn && !tsonBtnPrev;   // flanco de subida del boton
    if (!sdcTson && btnRising && sdc3v3) {
        sdcTson = true;
        Serial.println(F("[TSON] armado."));
    } else if (btnRising && !sdcTson) {
        // No esta en main.cpp: en banco, saber POR QUE se ignora la pulsacion.
        Serial.println(F("[TSON] pulsacion IGNORADA (SDC_3V3 bajo)."));
    }
    tsonBtnPrev = tsonBtn;
    digitalWrite(PIN_SDC_TSON, sdcTson ? HIGH : LOW);

    // ── Precarga: temporizador de 5 s desde el flanco SDC_TSON ↑ ──
    static bool sdcTsonPrev = false;
    if (sdcTson && !sdcTsonPrev) {               // flanco de armado
        // PRECHARGE_DONE NO puede estar HIGH antes de cerrar el TSON: si lo
        // esta, la entrada esta atascada (aislador/soldadura) → no fiarse y
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
            prechargeFail = true;                // LATCH duro → reset de alimentacion
            Serial.println(F("[PRE] FALLO: precarga no completada en 5 s (enclavado)."));
        }
    }
    digitalWrite(PIN_PRECHARGE_FAIL, prechargeFail ? HIGH : LOW);
}
