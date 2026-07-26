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

// ⚠ TEMPORAL: 0 = el fallo del Hall NO corta la carga (solo avisa). Mientras el
// divisor del canal 350A esté sin arreglar en HW (offset ~2.5V fuera de ventana).
// Volver a 1 cuando el DHAB de 350A dé ~1.62V a 0 A.
#define CHG_HALL_BLOCKS  1

// ============================================================================
//  PINES — mapa central de la PCB (lib/COMMON/board_pins.h). ÚNICA fuente de
//  verdad, compartida con main.cpp. Ver ahí la convención PA_x vs PAx.
// ============================================================================
#include "board_pins.h"

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
// Corte DURO por celda BAJA. Mismo umbral que main.cpp (CELL_UV_V) y que el
// registro UV_THRESH del BQ (0x53 = 2.8 V). Cubre dos escenarios:
//   · celda realmente descargada → no se carga a 3 A un pack por debajo de UV;
//   · cable de sense abierto / conector suelto → el canal lee ~0 V o NEGATIVO,
//     así que la medida de esa celda es falsa y no se puede garantizar su OV.
// Sin este chequeo, un Vmin de -0.889 V dejaba BMS_OK en HIGH indefinidamente.
#define CELL_VMIN_HARD_V     2.8f       ///< ⚠ corte DURO por celda baja / sense abierto
#define CELL_TMAX_CHG_C      45.0f      ///< ⚠ T máx para cargar (el FW antiguo usaba 40)
#define CELL_TMIN_CHG_C      0.0f       ///< ⚠ T mín para cargar (Li-ion NO carga en frío)

#define MSG1_PERIOD_MS       1000UL     ///< cadencia de envío Message 1 (cargador corta a los 5 s sin él)
#define RX_TIMEOUT_MS        5000UL     ///< si no llega Message 2 en este tiempo → cargador mudo

// ============================================================================
//  PRESUPUESTO DE LATENCIA DE BMS_OK (FS EV5.8) — dimensionado con MEDIDAS
//
//  El límite es el tiempo TOTAL desde que la condición aparece hasta que
//  BMS_OK cae: ≤500 ms para V/I, ≤1000 ms para T. Ese total son TRES términos,
//  no solo la ventana de debounce (ese fue el error original: poner la ventana
//  igual al límite ya agota el presupuesto entero):
//
//    captura   el ADC de celda es single-shot y se dispara una vez por lectura
//              → si la condición aparece justo después de una conversión,
//              espera un periodo entero. Peor caso = cadencia de muestreo.
//    confirma  FaultTimer: ≥2 muestras malas consecutivas Y la ventana
//              cumplida → max(P, ceil(W/P)·P). Con W = P sale exactamente P.
//    proceso   de la conversión al digitalWrite de BMS_OK.
//
//  Medido en banco (20 boards, 125 kbaud, comando 'd'):
//    readVoltages     53.5 ms   → proceso V ≈ 53.5 - 10 (conversión) ≈  45 ms
//    readTemperatures 71.1 ms   → proceso T ≈ 71.1 - 15 (conversión) ≈  56 ms
//
//  Cuentas resultantes (peor caso):
//    V   200 (captura) + 200 (confirma) +  45 (proceso) = 445 ms  ≤ 500 ✓
//    T   400 (captura) + 400 (confirma) +  56 (proceso) = 856 ms  ≤1000 ✓
//    I   250 (ventana Hall) + 2 × ~90 (hueco de loop)   = 430 ms  ≤ 500 ✓
//
//  ⚠ Dos condiciones ESTRUCTURALES sostienen estas cuentas (ver loop()):
//     1. V y T NUNCA se leen en la misma vuelta. Si coincidieran, la
//        evaluación de V arrastraría los 71 ms de la lectura de T y el
//        término "proceso" de V pasaría de 45 a ~116 ms → 516 ms, fuera.
//     2. printChgStatus() no comparte vuelta con una lectura. Juntos daban
//        un hueco de loop de 214 ms medidos, que es lo que retrasa al Hall.
// ============================================================================
#define SAMPLE_V_MS          200UL      ///< cadencia de lectura+evaluación de V
#define SAMPLE_T_MS          400UL      ///< cadencia de lectura+evaluación de T/NTC
#define HALL_WINDOW_MS       250UL      ///< ventana del HallSensor (setFaultWindowMs)

#define WDG_TIMEOUT_US       8000000UL  ///< IWDG 8 s (igual que el BMS)

// ── VENTANA DE GRACIA POR FALLO DE COMMS DEL BQ ─────────────────────────────
// Un fallo de comunicación (lectura fallida o BQ sin direccionar) NO baja
// BMS_OK al instante: debe persistir esta ventana ENTERA. Dentro de ella se
// sigue reintentando la lectura y se conserva la última medida buena, con
// BMS_OK en HIGH. Solo si al expirar el fallo SIGUE presente se declara fallo
// y el pin cae a LOW. Ajustable en caliente por serial con 'f,<ms>'.
#define FAULT_COMM_MS        5000UL     ///< valor por defecto al arrancar (ms)
#define FAULT_COMM_MIN_MS     250UL     ///< suelo: < SAMPLE_V_MS no da ni 2 muestras
#define FAULT_COMM_MAX_MS   30000UL     ///< tope duro del comando 'f'

// Ante fallo de comms CONFIRMADO, intentar reconectar el BQ con reInit().
// Rate-limit porque reInit() BLOQUEA varios segundos (wake + auto-address,
// hasta 5 intentos): no se llama en cada muestra.
// ⚠ Aquí ponía `PF_2`, que NO es un tiempo sino un token PinName de STM32
//   (= 82): el rate-limit era de 82 ms y se relanzaba el auto-address
//   prácticamente en cada muestra fallida. Mismo valor que main.cpp.
#define CHG_REINIT_RETRY_MS  2000UL

// Debounce de V/T/NTC: el fallo debe persistir la ventana Y ≥2 muestras
// consecutivas (k por defecto del FaultTimer). Cada ventana = su cadencia de
// muestreo → exactamente 2 lecturas malas seguidas para confirmar: un valor
// espurio aislado (con CRC válido) NO corta la carga, pero el presupuesto de
// EV5.8 se respeta. El ruido de TRANSPORTE (COMM/CRC) ya lo cubre aparte el
// retry del driver (BQ_READ_ATTEMPTS=3), así que esto solo filtra el valor malo.
// ⚠ Eran 500/1000/1000 = el límite completo de la norma, sin dejar nada para
//   captura ni proceso → V disparaba a ~876 ms y T a ~1312 ms medidos.
#define FAULT_V_MS           SAMPLE_V_MS  ///< 200 ms → confirma en 2 muestras
#define FAULT_T_MS           SAMPLE_T_MS  ///< 400 ms → confirma en 2 muestras
#define FAULT_NTC_MS         SAMPLE_T_MS  ///< NTC abierto: igual que T

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
// ruido, sobre-I, ADC saturado) cortan la carga vía hallSafe().
static HallSensor hall(PIN_AMP_30A, PIN_AMP_350A);

static CAN_BUS* gCan   = nullptr;
static bool     gCanOk = false;
static bool     bmsInitOk = false;

// Debounce (mismo FaultTimer que main.cpp): comms + V/T/NTC.
static FaultTimer fComm, fV, fT, fNtc;

// Ventana de gracia de comms VIVA (arranca en FAULT_COMM_MS, editable con
// 'f,<ms>'). Es la única fuente de verdad: todo confirmed() de fComm la usa.
static unsigned long commWindowMs = FAULT_COMM_MS;

// ms del último intento de reInit() (rate-limit compartido por los dos caminos
// que reconectan: BQ sin inicializar y fallo de lectura confirmado).
static unsigned long tLastReinit = 0;

// ¿Hubo alguna lectura V+T buena desde el reset? La ventana de gracia solo
// tiene sentido si existe una "última medida buena" que conservar; sin ella
// (arranque con la cadena muda) BMS_OK debe quedarse LOW desde el principio.
static bool everRead = false;

// Veredicto del pack (comms + V/T/NTC). Lo recalcula evalPack() al final de
// cada slot de lectura; el loop lo combina con hallSafe() cada vuelta.
// Arranca en false: no se afirma "OK" antes de haber medido nada.
static bool packSafe = false;

// Último resultado de cada lectura. Se guardan por separado porque V y T se
// leen en slots distintos y el debounce de comms necesita mirar los dos.
static BQResult lastResV = BQResult::COMM_ERROR;
static BQResult lastResT = BQResult::COMM_ERROR;

// ============================================================================
//  INSTRUMENTACIÓN DE TIEMPOS — validación en banco de EV5.8
//
//  Presupuesto: desde que la condición APARECE hasta que BMS_OK cae, ≤500 ms
//  para V/I y ≤1000 ms para T. Ese total tiene TRES términos:
//
//    captura   cuantización del muestreo. El ADC de celda es single-shot y se
//              dispara una vez por lectura: si la condición aparece justo
//              después de una conversión, espera un periodo entero. NO se
//              puede medir desde el firmware (no sabemos cuándo apareció);
//              su peor caso ES el periodo de muestreo, que sí se mide aquí.
//    confirma  1ª muestra mala → fallo confirmado por el FaultTimer.
//    proceso   confirmación → digitalWrite de BMS_OK.
//
//  tmTripLatMs mide (confirma + proceso). El peor caso total es
//  tmTripLatMs + periodo de muestreo máximo.
//
//  Todo son contadores pasivos: nada imprime dentro del bloque de muestreo
//  (un printf ahí falsearía justo lo que se quiere medir). Se vuelcan en el
//  status de 1 s y se ponen a cero con el comando 'z'.
// ============================================================================
static uint32_t tmReadVUs = 0, tmReadVMaxUs = 0;   ///< duración de readVoltages()
static uint32_t tmReadTUs = 0, tmReadTMaxUs = 0;   ///< duración de readTemperatures()
static uint32_t tmBlockUs = 0, tmBlockMaxUs = 0;   ///< slot de V: lectura + evaluación
static uint32_t tmPeriodMs = 0, tmPeriodMaxMs = 0; ///< periodo real entre muestreos
static uint32_t tmLoopGapMaxMs = 0;                ///< mayor hueco entre vueltas de loop() (= stall que retrasa el Hall)
static uint32_t tmTripLatMs = 0;                   ///< último disparo: confirma+proceso
static const char* tmTripCause = "-";
static unsigned long tmHallFailMs = 0;             ///< millis() en que hall.isOK() pasó a false

// Rellenados por evalPack()/hallSafe() al confirmar un fallo; los consume el loop al
// bajar BMS_OK para cerrar la medida de latencia.
static const char*   tmTripPend  = nullptr;
static unsigned long tmTripStart = 0;

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
bool   tryReinit(unsigned long now);
void   evalVLimits(unsigned long now);
void   evalTLimits(unsigned long now);
void   sampleV();
void   sampleT();
void   evalPack();
bool   hallSafe();
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

    // BMS_OK a LOW hasta que haya una medida real. begin() lo deja en HIGH al
    // salir bien, pero eso afirma "pack seguro" sin haber leído V ni T todavía.
    bms.setBmsOk(false);

    // Hall: autocalibración de offset (~1 s, con corriente ~0 → aún sin cargar).
    hall.begin();
    // Ventana de persistencia del Hall. El default de la librería (500 ms) ES
    // el presupuesto entero de EV5.8 para corriente → nunca podría cumplirse
    // una vez sumados los huecos de loop. Ver el bloque de presupuesto arriba.
    hall.setFaultWindowMs(HALL_WINDOW_MS);
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
    Serial.printf("Corte por celda: UV=%.2f V  OV=%.2f V | T carga: %.0f..%.0f C\n",
                  CELL_VMIN_HARD_V, CELL_VMAX_HARD_V,
                  CELL_TMIN_CHG_C, CELL_TMAX_CHG_C);
    Serial.printf("Ventana de gracia comms BQ: %lu ms (BMS_OK NO cae antes)\n",
                  commWindowMs);
    Serial.println(F("ARRANCA SIN CARGAR. Comandos: g=start x=stop c,<I>=corriente f,<ms>=ventana comms v=voltajes t=temps d=datos z=reset timing r=restart"));

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

    // Recuperación de bus-off: si el OBC no está en el bus al arrancar (o hay un
    // glitch), las tramas sin ACK llevan el FDCAN a bus-off y se queda MUERTO
    // (ni TX ni RX) hasta reset. rebootBusFromError() lo saca de ahí. Barato si
    // el bus está sano (solo mira GetProtocolStatus().BusOff). Igual que main.cpp.
    static unsigned long tCanChk = 0;
    if (gCanOk && (millis() - tCanChk >= 1000)) {
        tCanChk = millis();
        gCan->rebootBusFromError();
    }

    // Mayor hueco entre vueltas de loop(). Es el retraso máximo que puede
    // sufrir el timer interno del Hall (hall.update() solo corre aquí): lo
    // domina la lectura del BQ, que BLOQUEA. Término "proceso" del presupuesto de I.
    static unsigned long tLoopPrev = 0;
    unsigned long tLoopNow = millis();
    if (tLoopPrev && (tLoopNow - tLoopPrev) > tmLoopGapMaxMs)
        tmLoopGapMaxMs = tLoopNow - tLoopPrev;
    tLoopPrev = tLoopNow;

    // Amperímetro cada vuelta (máxima resolución + debounce interno de fallos).
    hall.update();

    // Flanco de caída de hall.isOK(): marca cuándo el HallSensor CONFIRMÓ su
    // fallo (ya con su ventana HALL_WINDOW_MS interna). Lo que va de aquí a que
    // baje BMS_OK es latencia de CONSUMO; con hallSafe() en la vía rápida
    // debería salir ~0 — este contador es justo la comprobación de que es así.
    static bool hallOkPrev = true;
    bool hallOkNow = hall.isOK();
    if (hallOkPrev && !hallOkNow) tmHallFailMs = tLoopNow;
    hallOkPrev = hallOkNow;

    // Máquina TSON + precarga: cada vuelta (flanco del botón, timing precarga).
    updateTson();

    // ── Muestreo del pack: A LO SUMO UNA lectura por vuelta ─────────────────
    // El `else if` es de SEGURIDAD, no un ahorro: si V y T vencieran a la vez
    // y se leyeran seguidas, la evaluación de V arrastraría los ~71 ms de la
    // lectura de T y su término "proceso" pasaría de 45 a ~116 ms → 516 ms,
    // fuera del presupuesto de 500 ms. Cuando coinciden, V gana la vuelta y T
    // entra en la siguiente (µs después, ya con BMS_OK actualizado por V).
    // Causa del disparo: se re-arma cada vuelta para que la medida de latencia
    // no atribuya un fallo viejo a una caída nueva. La rellenan evalPack() (V/T/
    // NTC/comms) o hallSafe(), ambos aguas abajo de esta línea en esta vuelta.
    tmTripPend = nullptr;

    bool didRead = false;
    static unsigned long tV = 0, tT = 0;
    if (millis() - tV >= SAMPLE_V_MS) {
        unsigned long tPrev = tV;
        tV = millis();
        if (tPrev) {                       // periodo REAL de V (= término "captura")
            tmPeriodMs = tV - tPrev;
            if (tmPeriodMs > tmPeriodMaxMs) tmPeriodMaxMs = tmPeriodMs;
        }
        uint32_t tBlk = micros();
        sampleV();
        tmBlockUs = micros() - tBlk;
        if (tmBlockUs > tmBlockMaxUs) tmBlockMaxUs = tmBlockUs;
        didRead = true;
    } else if (millis() - tT >= SAMPLE_T_MS) {
        tT = millis();
        sampleT();
        didRead = true;
    }

    // ── BMS_OK en la VÍA RÁPIDA (cada vuelta) ───────────────────────────────
    // packSafe lo refrescan sampleV/sampleT; el Hall se consulta aquí mismo.
    // Antes el Hall solo se miraba dentro del bloque de muestreo, así que un
    // fallo suyo esperaba hasta un periodo entero de más para llegar al pin.
    // setBmsOk() solo escribe en los cambios → llamarlo cada vuelta es barato.
    bmsSafe = packSafe && hallSafe();
    if (!bmsSafe) chargeRequested = false;   // un fallo cancela la orden de carga
    bms.setBmsOk(bmsSafe);                   // OK=HIGH, fallo=LOW (fail-safe)

    // Flanco de caída de BMS_OK → cerrar la medida de latencia.
    static bool bmsSafePrev = true;
    if (bmsSafePrev && !bmsSafe && tmTripPend && tmTripStart) {
        tmTripCause = tmTripPend;
        tmTripLatMs = millis() - tmTripStart;
        Serial.printf("[TIMING] disparo %s: 1a muestra mala → BMS_OK LOW = %lu ms"
                      "  (+ hasta %lu ms de captura → peor caso %lu ms)\n",
                      tmTripCause, tmTripLatMs, tmPeriodMaxMs,
                      tmTripLatMs + tmPeriodMaxMs);
    }
    bmsSafePrev = bmsSafe;

    // ── TX Message 1 cada 1 s — el cargador corta si no lo recibe en 5 s. ──
    // El envío va SIEMPRE en su instante (el OBC no espera), pero el volcado
    // por serie (~70 ms a 115200) se aplaza a una vuelta sin lectura: juntos
    // daban un hueco de loop de 214 ms medidos, y ese hueco es justo lo que
    // retrasa el timer del Hall (dos veces, ver el presupuesto de arriba).
    static unsigned long tMsg1 = 0;
    static bool printPending = false;
    if (millis() - tMsg1 >= MSG1_PERIOD_MS) {
        tMsg1 = millis();

        bool allow = chargeRequested && bmsSafe;
        sendMessage1(allow);
        charging = allow;

        printPending = true;
    }
    if (printPending && !didRead) {
        printPending = false;
        printChgStatus();
    }

    IWatchdog.reload();
}

// ============================================================================
//  MUESTREO DEL PACK — V y T en slots SEPARADOS
//
//  V y T se leen en vueltas distintas del loop (ver el `else if` de loop()):
//  así la evaluación de V no arrastra los ~71 ms de la lectura de T y el
//  término "proceso" de su presupuesto se queda en ~45 ms. Cada slot termina
//  llamando a evalPack(), que recalcula el veredicto completo.
// ============================================================================

// Muestrea los límites de celda con los datos YA leídos (no lee nada).
void evalVLimits(unsigned long now)
{
    // OV y UV en el MISMO FaultTimer: cualquiera de los dos fuera de ventana es
    // "V mala". ⚠ El UV es tan crítico como el OV — sin él una celda a ~0 V (o
    // con el sense abierto, que lee negativo) no bajaba BMS_OK NUNCA.
    bool badVmax = (bms.getMaxVoltage() >= CELL_VMAX_HARD_V);
    bool badVmin = (bms.getMinVoltage() <= CELL_VMIN_HARD_V);
    fV.sample(badVmax || badVmin, now);
}

void evalTLimits(unsigned long now)
{
    bool badT = (bms.getMaxTemp() >= CELL_TMAX_CHG_C) ||
                (bms.getMinTemp() <= CELL_TMIN_CHG_C);
    fT.sample(badT, now);
    // NTC abierto/inválido: pérdida de medida térmica. El driver EXCLUYE los NTC
    // abiertos de get{Min,Max}Temp (centinela -1000) → hay que mirarlo aparte.
    fNtc.sample(bms.hasOpenNtc(), now);
}

// ── Slot de VOLTAJE (cada SAMPLE_V_MS) ──────────────────────────────────────
void sampleV()
{
    if (!bmsInitOk) { lastResV = BQResult::COMM_ERROR; evalPack(); return; }

    // Cronometrado ANTES de cualquier printf: los mensajes de diagnóstico a
    // 115200 baud cuestan ms y falsearían la medida.
    uint32_t t0 = micros();
    lastResV = bms.readVoltages();
    tmReadVUs = micros() - t0;
    if (tmReadVUs > tmReadVMaxUs) tmReadVMaxUs = tmReadVUs;

    if (lastResV != BQResult::OK)
        // Diagnóstico: TIPO (COMM=no responde / CRC=corrupto) y board que casca.
        // Board alto y persistente → integridad de señal de la cadena.
        Serial.printf("[BQ] V FALLO %s en board %d\n",
                      lastResV == BQResult::CRC_ERROR ? "CRC" : "COMM",
                      bms.getLastReadFailBoard());
    else
        evalVLimits(millis());

    evalPack();
}

// ── Slot de TEMPERATURA + NTC (cada SAMPLE_T_MS) ────────────────────────────
void sampleT()
{
    if (!bmsInitOk) { lastResT = BQResult::COMM_ERROR; evalPack(); return; }

    uint32_t t0 = micros();
    lastResT = bms.readTemperatures();
    tmReadTUs = micros() - t0;
    if (tmReadTUs > tmReadTMaxUs) tmReadTMaxUs = tmReadTUs;

    if (lastResT != BQResult::OK)
        Serial.printf("[BQ] T FALLO %s en board %d\n",
                      lastResT == BQResult::CRC_ERROR ? "CRC" : "COMM",
                      bms.getLastReadFailBoard());
    else
        evalTLimits(millis());

    evalPack();
}

// ============================================================================
//  Reconexión del BQ — rate-limited (reInit() BLOQUEA varios segundos: wake +
//  auto-address, hasta 5 intentos).
//
//  reInit() se llama con keepBmsOk por defecto (true) → NO toca BMS_OK. El
//  nivel del pin lo decide SOLO el debounce de evalPack().
//
//  Se refresca el watchdog a ambos lados de la llamada: el loop no llega a su
//  IWatchdog.reload() mientras reInit() bloquea, y un reset del IWDG durante
//  la reconexión reiniciaría el MCU → BMS_OK a LOW de inmediato, que es justo
//  lo que la ventana de gracia trata de evitar.
// ============================================================================
bool tryReinit(unsigned long now)
{
    if ((now - tLastReinit) < CHG_REINIT_RETRY_MS) return false;
    tLastReinit = now;

    Serial.println(F("[BQ] comms caídas (confirmado) → reInit() (reconectando)..."));
    IWatchdog.reload();
    bmsInitOk = bms.reInit();
    IWatchdog.reload();
    Serial.println(bmsInitOk ? F("[OK] BQ reconectado; releyendo el pack...")
                             : F("[ERROR] reInit falló; se reintentará."));
    return bmsInitOk;
}

// ============================================================================
//  VEREDICTO DEL PACK — comms (con ventana de gracia) + V/T/NTC
//
//  Se llama al final de CADA slot de lectura y deja el resultado en packSafe,
//  que el loop combina con el Hall para gobernar BMS_OK.
//
//  ⚠ NUNCA declara fallo de comms a la primera. Solo pone packSafe=false si al
//    agotarse commWindowMs el problema SIGUE ahí *después* de intentar
//    recuperarlo. Dentro de la ventana se conserva la última medida buena.
// ============================================================================
void evalPack()
{
    unsigned long now = millis();

    // El debounce de comms mira los DOS últimos resultados, aunque V y T se
    // lean en slots distintos. Si solo mirase la lectura del slot actual, un
    // fallo permanente en T alternaría malo/bueno con los slots de V y fComm
    // no confirmaría NUNCA. (Mismo criterio que main.cpp con lastResV/lastResT.)
    bool readOk = (lastResV == BQResult::OK) && (lastResT == BQResult::OK);
    if (readOk) everRead = true;          // ya hay medida buena que conservar
    fComm.sample(!readOk, now);

    if (!readOk) {
        // ── ¿Queda ventana de gracia? ───────────────────────────────────────
        // Dentro de la ventana NO se toca la cadena: se seguirá pidiendo datos
        // en el próximo slot y BMS_OK sigue HIGH con la última medida buena,
        // así un glitch de ruido se recupera solo. No se evalúan V/T/NTC: el
        // driver puede devolver caché parcial o centinelas. Excepción de
        // arranque en frío: sin ninguna lectura buena desde el reset
        // (!everRead) no hay medida que conservar → sin gracia, LOW fail-safe.
        if (everRead && !fComm.confirmed(now, commWindowMs)) { packSafe = true; return; }

        // ── Ventana agotada: ÚLTIMO CARTUCHO antes de declarar fallo ────────
        // Reconectar la cadena (auto-address) y RELEER aquí mismo. El requisito
        // es bajar BMS_OK solo "si al expirar el problema SIGUE presente": un
        // auto-address correcto seguido de una lectura buena demuestra que ya
        // no sigue, así que no se declara fallo y el pin no llega a caer.
        tmTripPend = "COMM";  tmTripStart = fComm.tStart;

        if (!tryReinit(now)) { packSafe = false; return; }   // rate-limited o falló

        now = millis();                    // reInit() bloquea varios segundos
        lastResV = bms.readVoltages();
        lastResT = bms.readTemperatures();
        if (lastResV != BQResult::OK || lastResT != BQResult::OK) {
            fComm.sample(true, now);       // reconectó pero sigue sin leer
            packSafe = false;
            return;
        }
        everRead = true;
        fComm.sample(false, now);          // un OK rompe la serie: fallo resuelto
        evalVLimits(now);                  // datos frescos → refrescar debounces
        evalTLimits(now);
        Serial.println(F("[BQ] comms restablecidas tras reInit (BMS_OK no llegó a caer)."));
    }

    // ── Límites de celda con datos frescos y válidos ────────────────────────
    // Cada FaultTimer confirma con ≥2 muestras malas consecutivas Y su ventana
    // (= su cadencia de muestreo). Un valor espurio aislado no corta la carga.
    if (fV.confirmed(now, FAULT_V_MS)) {
        tmTripPend = "V";  tmTripStart = fV.tStart;
        Serial.printf("[SAFE] V fuera de rango (min=%.3f max=%.3f, limites %.2f..%.2f V,"
                      " confirmado) → parar.\n",
                      bms.getMinVoltage(), bms.getMaxVoltage(),
                      CELL_VMIN_HARD_V, CELL_VMAX_HARD_V);
        packSafe = false;
        return;
    }
    if (fT.confirmed(now, FAULT_T_MS)) {
        tmTripPend = "T";  tmTripStart = fT.tStart;
        Serial.printf("[SAFE] T fuera de rango (min=%.1f max=%.1f, confirmado) → parar.\n",
                      bms.getMinTemp(), bms.getMaxTemp());
        packSafe = false;
        return;
    }
    if (fNtc.confirmed(now, FAULT_NTC_MS)) {
        tmTripPend = "NTC";  tmTripStart = fNtc.tStart;
        Serial.printf("[SAFE] %u NTC abierto(s) (confirmado) → parar.\n",
                      bms.getOpenNtcCount());
        packSafe = false;
        return;
    }
    packSafe = true;
}

// ============================================================================
//  Amperímetro — se consulta en la VÍA RÁPIDA (cada vuelta del loop).
//
//  isOK() ya viene con el debounce propio del HallSensor (ventana fijada en
//  setup() con setFaultWindowMs). Consultarlo aquí y no dentro de un slot de
//  muestreo elimina el término de consumo del presupuesto de corriente: antes
//  un fallo del Hall esperaba hasta un periodo entero para llegar al pin.
// ============================================================================
bool hallSafe()
{
    bool ok = hall.isOK();

#if CHG_HALL_BLOCKS
    // Aviso solo en el FLANCO: esto corre cada vuelta, un printf por vuelta
    // ahogaría el serial y dispararía el hueco de loop que se quiere minimizar.
    static bool prevOk = true;
    if (!ok && prevOk) {
        tmTripPend = "HALL";  tmTripStart = tmHallFailMs;
        Serial.printf("[SAFE] Hall FALLO (%s%s%s%s%s) → parar.\n",
                      hall.isDisconnected() ? "desc "   : "",
                      hall.isStuck()        ? "stuck "  : "",
                      hall.isNoisy()        ? "noisy "  : "",
                      hall.isOverCurrent()  ? "sobreI " : "",
                      hall.isAdcSaturated() ? "adcSat"  : "");
    } else if (ok && !prevOk) {
        Serial.println(F("[SAFE] Hall recuperado → BMS_OK puede volver a HIGH."));
    }
    prevOk = ok;
    return ok;
#else
    // TEMP: no corta (divisor 350A sin arreglar). Aviso cada 5 s, no cada vuelta.
    if (!ok) {
        static unsigned long tHallWarn = 0;
        if (millis() - tHallWarn >= 5000) {
            tHallWarn = millis();
            Serial.println(F("[HALL] fallo IGNORADO (temporal, CHG_HALL_BLOCKS=0)"));
        }
    }
    return true;
#endif
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
    Serial.printf("BMS_OK: %s (PB5, activo-alto)\n",
                  bmsSafe ? "HIGH (OK)" : "LOW (FALLO)");
    // Estado del debounce de comms: si hay un fallo en curso, cuánto lleva y
    // cuánto le queda para tumbar BMS_OK.
    if (fComm.cond || fComm.badRun) {
        unsigned long held = millis() - fComm.tStart;
        Serial.printf("COMM: FALLO en curso %lu/%lu ms (n=%u) %s\n",
                      held, commWindowMs, fComm.badRun,
                      fComm.confirmed(millis(), commWindowMs) ? "→ CONFIRMADO"
                                                             : "→ en gracia");
    } else {
        Serial.printf("COMM: OK (ventana de gracia %lu ms)\n", commWindowMs);
    }
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
    // ── Tiempos medidos (EV5.8). 'z' pone los máximos a cero. ──
    Serial.printf("TIMING: readV=%.1f/%.1f  readT=%.1f/%.1f  slotV=%.1f/%.1f ms (ult/max)\n",
                  tmReadVUs / 1000.0f, tmReadVMaxUs / 1000.0f,
                  tmReadTUs / 1000.0f, tmReadTMaxUs / 1000.0f,
                  tmBlockUs / 1000.0f, tmBlockMaxUs / 1000.0f);
    Serial.printf("        periodo V=%lu/%lu ms (ult/max)  gap loop max=%lu ms\n",
                  tmPeriodMs, tmPeriodMaxMs, tmLoopGapMaxMs);
    if (tmTripLatMs)
        Serial.printf("        ultimo disparo %s: confirma+proceso=%lu ms → peor caso %lu ms\n",
                      tmTripCause, tmTripLatMs, tmTripLatMs + tmPeriodMaxMs);
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
//    f,<ms>  → ventana de gracia ante fallo de comms del BQ (antes de BMS_OK LOW)
//    v       → tabla de voltajes por celda
//    t       → tabla de temperaturas por NTC
//    d       → volcar estado (incluye los tiempos medidos)
//    z       → poner a cero los máximos de TIMING
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

    // f,<ms>  → ventana de gracia ante fallo de comms del BQ. Es el tiempo que
    // debe persistir el fallo ANTES de bajar BMS_OK. Acotada a
    // [FAULT_COMM_MIN_MS, FAULT_COMM_MAX_MS]: por debajo del periodo de
    // muestreo no caben 2 muestras y el debounce dejaría de filtrar.
    if (buf[0] == 'f' && buf[1] == ',') {
        long ms = atol(buf + 2);
        if (ms < (long)FAULT_COMM_MIN_MS) ms = FAULT_COMM_MIN_MS;
        if (ms > (long)FAULT_COMM_MAX_MS) ms = FAULT_COMM_MAX_MS;
        commWindowMs = (unsigned long)ms;
        Serial.printf("[COMM] ventana de gracia = %lu ms (rango %lu..%lu)\n",
                      commWindowMs, FAULT_COMM_MIN_MS, FAULT_COMM_MAX_MS);
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
    case 'z':
        tmReadVMaxUs = tmReadTMaxUs = tmBlockMaxUs = 0;
        tmPeriodMaxMs = tmLoopGapMaxMs = tmTripLatMs = 0;
        tmTripCause = "-";
        Serial.println(F("[TIMING] maximos a cero."));
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
