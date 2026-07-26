/**
 * @file    can_test.cpp
 * @brief   TEST CAN MÍNIMO — solo ENVÍA tramas por CAN, para depurar el bus.
 *
 * Usa la MISMA librería MART_CAN y el mismo FDCAN1 (RX=PA11 / TX=PA12) que el
 * firmware real. Si estas tramas se ven en un analizador CAN o en el osciloscopio
 * sobre CANH/CANL → el camino CAN (MCU + transceptor + init + baudrate) está
 * bien y el problema está en otra parte. Si NO se ve nada → hardware
 * (transceptor, alimentación, pines, terminación).
 *
 * Envía una trama ESTÁNDAR (ID = CAN_TEST_ID) cada SEND_MS con un contador que
 * incrementa en el byte 0 (fácil de ver). Recupera bus-off solo (por si no hay
 * otro nodo que haga ACK). Imprime por serie cuántas ha mandado y el estado.
 *
 * ⚠ Ajusta CAN_BAUD_KBPS al bus que pruebes: 125 = coche, 500 = cargador (OBC).
 *
 * Lanzar:  pio run -e can_test -t upload
 * Monitor: pio device monitor -e can_test
 */

#include <Arduino.h>
#include "MART_CAN.h"

// ── CONFIG ──────────────────────────────────────────────────────────────────
#define CAN_BAUD_KBPS   500        ///< 125 = bus del coche · 500 = cargador (OBC)
#define CAN_NODE_ID     3          ///< perfil de filtro MART_CAN (3 = acepta todo)
#define CAN_TEST_ID     0x123UL    ///< ID estándar de prueba (11 bits)
#define SEND_MS         200UL      ///< cada cuánto se manda una trama

// 1 = LOOPBACK externo: transmite en el pin TX y se auto-ACKa → NUNCA entra en
//     bus-off aunque estés solo (sin otro nodo ni terminación). Para ver las
//     tramas en el OSCILOSCOPIO/analizador y aislar MCU+transceptor del bus.
// 0 = NORMAL: bus real. Si no hay otro nodo que haga ACK (o falta la
//     terminación de 120 Ω), entra en bus-off — eso NO es un bug del código.
#define CAN_LOOPBACK    1
// ────────────────────────────────────────────────────────────────────────────

static CAN_BUS*  gCan    = nullptr;
static bool      gCanOk  = false;
static uint32_t  txCount = 0;

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println(F("\n=== TEST CAN (solo envio) ==="));
    Serial.printf("Baud=%d kbps  ID=0x%lX  cada %lu ms  (FDCAN1 PA11=RX PA12=TX)\n",
                  CAN_BAUD_KBPS, CAN_TEST_ID, SEND_MS);

    // El ctor hace el init del FDCAN (HAL) → debe correr en setup, no como global.
    static CAN_BUS canBus(HardwareType::Transciever, CAN_BAUD_KBPS, CAN_NODE_ID,
                          5, 4, 30, 30, CAN_LOOPBACK);
    gCan   = &canBus;
    Serial.println(CAN_LOOPBACK ? F("[CAN] Modo LOOPBACK (osciloscopio, sin bus-off).")
                                : F("[CAN] Modo NORMAL (bus real)."));
    gCanOk = (gCan->SetupState() == 0);
    Serial.println(gCanOk ? F("[CAN] FDCAN listo. Enviando...")
                          : F("[CAN] FDCAN init FALLO."));
}

void loop()
{
    if (!gCanOk) {
        Serial.println(F("[CAN] init FALLO — pulsa reset para reintentar."));
        delay(1000);
        return;
    }

    // ── Enviar una trama cada SEND_MS con contador en el byte 0 ──
    static unsigned long tSend = 0;
    if (millis() - tSend >= SEND_MS) {
        tSend = millis();
        uint8_t d[8] = { (uint8_t)txCount, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 };
        gCan->setPacket(CAN_TEST_ID, d, 8, false);   // false = bytes crudos (sin conversión)
        gCan->send(CAN_TEST_ID);                      // envía YA esa trama
        txCount++;
    }

    // ── Estado + recuperación de bus-off cada 1 s ──
    // Si no hay otro nodo que haga ACK, el FDCAN va a bus-off; esto lo recupera
    // y así se sigue viendo actividad en el bus. En el osciloscopio verás las
    // tramas igualmente (los bits salen aunque nadie haga ACK).
    static unsigned long tChk = 0;
    if (millis() - tChk >= 1000) {
        tChk = millis();
        bool busOk = gCan->rebootBusFromError();
        Serial.printf("TX=%lu  bus=%s\n", txCount,
                      busOk ? "OK" : "BUS-OFF (sin ACK? recuperando)");
    }
}