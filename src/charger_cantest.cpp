/**
 * @file    charger_cantest.cpp
 * @brief   TEST de COMUNICACIÓN CAN con el cargador OBC TC HK-L 6.6kW.
 *
 *  Programa MÍNIMO para verificar que el cargador habla por CAN ANTES de
 *  meter la lógica de carga. NO usa el BQ79606 ni manda ninguna orden de
 *  cargar: el frame que envía lleva V=0, I=0 y control=PARAR (keep-alive).
 *  Es 100% seguro tenerlo conectado al pack.
 *
 *  Qué comprueba:
 *    · RX  → recibimos el Message 2 del cargador (0x18FF50E5) = el OBC habla,
 *            está alimentado (AC) y al baud correcto.
 *    · TX  → al mandarle el Message 1, si el bit COMM_TO (timeout) del status
 *            queda en 0, es que el cargador NOS recibe = enlace bidireccional.
 *
 *  Lanzar:   pio run -e charger_cantest -t upload
 *  Monitor:  pio device monitor -e charger_cantest   (115200 baud)
 *  Comandos serie:  t=on/off TX   d=volcar ahora   r=reset
 */

#include <Arduino.h>
#include "MART_CAN.h"

// ── Config CAN (idéntica a charger.cpp) ─────────────────────────────────────
#define CHG_CAN_BAUD     500          ///< kbps — baud del cargador
#define CHG_NODE_ID      3            ///< perfil de filtro MART_CAN (acepta todo)

#define ID_BMS_TO_CHG    0x1806E5F4UL ///< Message 1 (BMS → cargador), extendido
#define ID_CHG_TO_BMS    0x18FF50E5UL ///< Message 2 (cargador → BMS), extendido

#define TX_PERIOD_MS     1000UL       ///< cadencia del keep-alive
#define RX_TIMEOUT_MS    5000UL       ///< sin Message 2 en este tiempo → "mudo"

// Bits del byte STATUS (B4 del Message 2)
#define ST_HW_FAIL    (1 << 0)
#define ST_OVERTEMP   (1 << 1)
#define ST_INPUT_V    (1 << 2)
#define ST_BAT_CONN   (1 << 3)   ///< batería no conectada / invertida
#define ST_COMM_TO    (1 << 4)   ///< timeout de comunicación (el OBC NO nos recibe)

// ── Estado ──────────────────────────────────────────────────────────────────
static CAN_BUS*      gCan       = nullptr;
static bool          gCanOk     = false;
static bool          txEnabled  = true;
static unsigned long rxCount    = 0;
static unsigned long tLastRx    = 0;
static uint8_t       lastRaw[8] = {0};
static float         chgOutV    = 0.0f;
static float         chgOutI    = 0.0f;
static uint8_t       chgStatus  = 0;

// ── Prototipos ──────────────────────────────────────────────────────────────
void sendKeepAlive();
void pollCharger();
void printStatus();
void handleSerial();

// ============================================================================
void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(50);
    delay(50);
    Serial.println(F("\n=== TEST CAN CARGADOR (TC HK-L 6.6kW) ==="));
    Serial.printf("Baud: %d kbps | TX 0x%08lX | RX 0x%08lX\n",
                  CHG_CAN_BAUD, ID_BMS_TO_CHG, ID_CHG_TO_BMS);
    Serial.println(F("Keep-alive SEGURO (V=0 I=0 PARAR). No ordena cargar."));
    Serial.println(F("Comandos: t=TX on/off  d=datos  r=reset\n"));

    static CAN_BUS canBus(HardwareType::Transciever, CHG_CAN_BAUD, CHG_NODE_ID);
    gCan   = &canBus;
    gCanOk = (gCan->SetupState() == 0);
    Serial.println(gCanOk ? F("[CAN] FDCAN listo.")
                          : F("[CAN] FDCAN init FALLO — revisa el transceiver."));

    tLastRx = millis();
}

// ============================================================================
void loop()
{
    handleSerial();
    pollCharger();   // RX continua (el OBC emite cada ~1 s)

    static unsigned long tTx = 0;
    if (millis() - tTx >= TX_PERIOD_MS) {
        tTx = millis();
        if (txEnabled) sendKeepAlive();
        printStatus();
    }
}

// ── Message 1: keep-alive SEGURO (no carga) ─────────────────────────────────
void sendKeepAlive()
{
    if (!gCanOk) return;
    uint8_t d[8] = {0};
    // V=0 (B0-1), I=0 (B2-3), control=1 PARAR (B4), resto 0.
    d[4] = 0x01;
    gCan->setPacket(ID_BMS_TO_CHG, d, 8, false);   // bytes crudos
    gCan->send();
}

// ── Message 2: estado del cargador ──────────────────────────────────────────
void pollCharger()
{
    if (!gCanOk) return;
    gCan->receive();

    uint8_t r[8];
    if (gCan->getPacket(ID_CHG_TO_BMS, r, 8, false)) {
        uint16_t vRaw =  ((uint16_t)r[0] << 8) | r[1];
        uint16_t iRaw = (((uint16_t)r[2] << 8) | r[3]) & 0x7FFF;
        chgOutV   = vRaw * 0.1f;
        chgOutI   = iRaw * 0.1f;
        chgStatus = r[4];
        memcpy(lastRaw, r, 8);
        rxCount++;
        tLastRx = millis();
    }
}

// ── Volcado por serie ───────────────────────────────────────────────────────
void printStatus()
{
    bool alive = (millis() - tLastRx) < RX_TIMEOUT_MS;
    Serial.println(F("\n--- CAN TEST ---"));
    Serial.printf("TX: %s (keep-alive PARAR @1Hz)\n", txEnabled ? "ON" : "OFF");

    if (alive) {
        Serial.printf("RX: %lu frames | ultimo hace %lu ms  ->  COMUNICA OK\n",
                      rxCount, millis() - tLastRx);
        Serial.printf("    Vout=%.1f V  Iout=%.1f A  st=0x%02X%s%s%s%s%s\n",
                      chgOutV, chgOutI, chgStatus,
                      (chgStatus & ST_HW_FAIL)  ? " HWFAIL"   : "",
                      (chgStatus & ST_OVERTEMP) ? " OVERTEMP" : "",
                      (chgStatus & ST_INPUT_V)  ? " INPUT_V"  : "",
                      (chgStatus & ST_BAT_CONN) ? " BAT_CONN" : "",
                      (chgStatus & ST_COMM_TO)  ? " COMM_TO"  : "");
        Serial.printf("    raw: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                      lastRaw[0], lastRaw[1], lastRaw[2], lastRaw[3],
                      lastRaw[4], lastRaw[5], lastRaw[6], lastRaw[7]);
        // Interpretación del enlace de subida (¿nos recibe el cargador?)
        if (txEnabled) {
            Serial.println((chgStatus & ST_COMM_TO)
                ? F("    TX: COMM_TO=1 -> el cargador NO nos recibe (revisa CANH/CANL).")
                : F("    TX: COMM_TO=0 -> el cargador NOS recibe (enlace bidireccional OK)."));
        }
    } else {
        Serial.println(F("RX: SIN frames -> NO comunica."));
        Serial.println(F("    Revisa: baud=500k, CANH/CANL (no cruzados), 120 ohm,"));
        Serial.println(F("            alimentacion AC del OBC, GND comun."));
    }
}

// ── Comandos serie ──────────────────────────────────────────────────────────
void handleSerial()
{
    if (!Serial.available()) return;
    char c = Serial.read();
    switch (c) {
    case 't':
        txEnabled = !txEnabled;
        Serial.printf("[TX] %s\n", txEnabled ? "ON" : "OFF (solo escucha)");
        break;
    case 'd':
        printStatus();
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
