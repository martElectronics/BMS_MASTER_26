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
#define CELL_TMAX_CHG_C      45.0f      ///< ⚠ T máx para cargar (el FW antiguo usaba 40)
#define CELL_TMIN_CHG_C      0.0f       ///< ⚠ T mín para cargar (Li-ion NO carga en frío)

#define MSG1_PERIOD_MS       1000UL     ///< cadencia de envío Message 1 (cargador corta a los 5 s sin él)
#define SAMPLE_MS            250UL      ///< cadencia de lectura+seguridad del pack (V/T/NTC). Desacoplada del MSG1 → OV/OT/NTC se detectan en <0,5 s, no en ~1 s.
#define RX_TIMEOUT_MS        5000UL     ///< si no llega Message 2 en este tiempo → cargador mudo

#define WDG_TIMEOUT_US       8000000UL  ///< IWDG 8 s (igual que el BMS)

// ── VENTANA DE GRACIA POR FALLO DE COMMS DEL BQ ─────────────────────────────
// Un fallo de comunicación (lectura fallida o BQ sin direccionar) NO baja
// BMS_OK al instante: debe persistir esta ventana ENTERA. Dentro de ella se
// sigue reintentando la lectura y se conserva la última medida buena, con
// BMS_OK en HIGH. Solo si al expirar el fallo SIGUE presente se declara fallo
// y el pin cae a LOW. Ajustable en caliente por serial con 'f,<ms>'.
#define FAULT_COMM_MS        5000UL     ///< valor por defecto al arrancar (ms)
#define FAULT_COMM_MIN_MS     250UL     ///< suelo: < SAMPLE_MS no da ni 2 muestras
#define FAULT_COMM_MAX_MS   30000UL     ///< tope duro del comando 'f'

// Ante fallo de comms CONFIRMADO, intentar reconectar el BQ con reInit().
// Rate-limit porque reInit() BLOQUEA varios segundos (wake + auto-address,
// hasta 5 intentos): no se llama en cada muestra.
// ⚠ Aquí ponía `PF_2`, que NO es un tiempo sino un token PinName de STM32
//   (= 82): el rate-limit era de 82 ms y se relanzaba el auto-address
//   prácticamente en cada muestra fallida. Mismo valor que main.cpp.
#define CHG_REINIT_RETRY_MS  2000UL

// Debounce de V/T/NTC (mismo criterio que main.cpp): el fallo debe persistir la
// ventana Y ≥2 muestras consecutivas (k por defecto del FaultTimer). Muestreamos
// a SAMPLE_MS (250 ms) → ≥2 lecturas por ventana → un glitch de ruido con CRC
// válido NO corta la carga; solo un fallo real y persistente. El retry del driver
// cubre el ruido de transporte (COMM/CRC); esto cubre el valor espurio.
#define FAULT_V_MS           500UL      ///< OV de celda debe persistir ≥500 ms
#define FAULT_T_MS           1000UL     ///< OT/UT debe persistir ≥1000 ms
#define FAULT_NTC_MS         1000UL     ///< NTC abierto debe persistir ≥1000 ms

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
bool   readVT();
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
    Serial.printf("Ventana de gracia comms BQ: %lu ms (BMS_OK NO cae antes)\n",
                  commWindowMs);
    Serial.println(F("ARRANCA SIN CARGAR. Comandos: g=start x=stop c,<I>=corriente f,<ms>=ventana comms v=voltajes t=temps d=datos r=restart"));

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

    // Amperímetro cada vuelta (máxima resolución + debounce interno de fallos).
    hall.update();

    // Máquina TSON + precarga: cada vuelta (flanco del botón, timing precarga).
    // Usa bmsSafe, que se refresca en el bloque de 1 s de abajo.
    updateTson();

    // ── Seguridad del pack: leer + evaluar RÁPIDO (SAMPLE_MS), desacoplado del
    //    envío de 1 s. Así un OV/OT/NTC se detecta en <0,5 s, no en ~1 s. ──
    static unsigned long tSample = 0;
    if (millis() - tSample >= SAMPLE_MS) {
        tSample = millis();

        bool readOk = readPack();
        bmsSafe = chargeAllowed(readOk);   // lo consume updateTson() y el MSG1

        // Un fallo CANCELA la orden de carga: hay que re-armar con 'g'.
        if (!bmsSafe) chargeRequested = false;

        // BMS_OK refleja la SEGURIDAD del pack (no si cargamos o no).
        // Polaridad del driver: OK=HIGH, fallo=LOW (fail-safe).
        bms.setBmsOk(bmsSafe);
    }

    // ── TX Message 1 cada 1 s — el cargador corta si no lo recibe en 5 s. ──
    static unsigned long tMsg1 = 0;
    if (millis() - tMsg1 >= MSG1_PERIOD_MS) {
        tMsg1 = millis();

        bool allow = chargeRequested && bmsSafe;
        sendMessage1(allow);
        charging = allow;

        printChgStatus();
    }

    IWatchdog.reload();
}

// ============================================================================
//  Reconexión del BQ — rate-limited (reInit() BLOQUEA varios segundos: wake +
//  auto-address, hasta 5 intentos).
//
//  reInit() se llama con keepBmsOk por defecto (true) → NO toca BMS_OK. El
//  nivel del pin lo decide SOLO el debounce de abajo.
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
//  Lectura cruda V+T de la cadena. Solo lee y reporta el TIPO de fallo
//  (COMM=no responde / CRC=corrupto) y en qué board casca — board alto y
//  persistente → problema de integridad de señal de la cadena.
//  No toca debounce ni BMS_OK: de eso se encarga readPack().
// ============================================================================
bool readVT()
{
    BQResult rV = bms.readVoltages();
    BQResult rT = bms.readTemperatures();
    bool okV = (rV == BQResult::OK);
    bool okT = (rT == BQResult::OK);

    if (!okV) Serial.printf("[BQ] V FALLO %s en board %d\n",
                            rV == BQResult::CRC_ERROR ? "CRC" : "COMM",
                            bms.getLastReadFailBoard());
    if (!okT) Serial.printf("[BQ] T FALLO %s en board %d\n",
                            rT == BQResult::CRC_ERROR ? "CRC" : "COMM",
                            bms.getLastReadFailBoard());
    return okV && okT;
}

// ============================================================================
//  Lectura del pack (V/T de celda vía BQ79606)
//
//  ⚠ NUNCA declara fallo a la primera. Todo error de comms —lectura fallida o
//    BQ sin direccionar— alimenta el debounce fComm. Solo devuelve false (y
//    solo entonces cae BMS_OK) si al agotarse commWindowMs el problema SIGUE
//    ahí *después* de intentar recuperarlo. Dentro de la ventana se conserva
//    la última medida buena y el pin sigue HIGH.
// ============================================================================
bool readPack()
{
    unsigned long now = millis();

    // ── 1. Lectura normal ───────────────────────────────────────────────────
    // Si la cadena no está direccionada (begin() falló al arrancar, o un
    // reInit() no la recuperó) no hay nada que leer: cuenta como fallo de
    // comms más y va al MISMO debounce, no a BMS_OK directo.
    bool readOk = bmsInitOk && readVT();
    if (readOk) everRead = true;          // ya hay medida buena que conservar
    fComm.sample(!readOk, now);
    if (readOk) return true;

    // ── 2. ¿Queda ventana de gracia? ────────────────────────────────────────
    // Dentro de la ventana NO se toca la cadena: se seguirá pidiendo datos en
    // el próximo ciclo (cada SAMPLE_MS) y BMS_OK sigue HIGH con la última
    // medida buena, así un glitch de ruido se recupera solo. Excepción de
    // arranque en frío: sin ninguna lectura buena desde el reset (!everRead)
    // no hay medida que conservar → sin gracia, BMS_OK LOW fail-safe.
    if (everRead && !fComm.confirmed(now, commWindowMs)) return true;

    // ── 3. Ventana agotada: ÚLTIMO CARTUCHO antes de declarar fallo ─────────
    // Reconectar la cadena (auto-address) y RE-LEER aquí mismo. El requisito
    // es bajar BMS_OK solo "si al expirar el problema SIGUE presente": un
    // auto-address correcto seguido de una lectura buena demuestra que ya no
    // sigue, así que NO se declara fallo y el pin no llega a caer.
    // ⚠ Antes se lanzaba el reInit() aquí pero se devolvía el veredicto de
    //   fallo igualmente → el auto-address terminaba OK y BMS_OK caía a LOW
    //   en el mismo ciclo (abriendo el SDC y desarmando el TSON) para volver
    //   a HIGH 250 ms después. Ese falso disparo es lo que se corrige aquí.
    if (!tryReinit(now)) return false;    // rate-limited, o reInit falló

    now = millis();                       // reInit() bloquea varios segundos
    if (!readVT()) {                      // reconectó pero sigue sin leer
        fComm.sample(true, now);
        return false;
    }

    everRead = true;
    fComm.sample(false, now);             // un OK rompe la serie: fallo resuelto
    Serial.println(F("[BQ] comms restablecidas tras reInit (BMS_OK no llegó a caer)."));
    return true;
}

// ============================================================================
//  ¿Se permite cargar?  (corte por celda/temperatura/lectura)
// ============================================================================
bool chargeAllowed(bool readOk)
{
    unsigned long now = millis();

    // Comm confirmado: readPack devuelve false solo cuando fComm ya está
    // confirmado (readOk = !fComm.confirmed()), es decir, cuando la ventana
    // commWindowMs se agotó con el fallo aún presente. → return false directo.
    // Se avisa solo en el FLANCO (si no, un mensaje cada SAMPLE_MS ahoga el
    // serial justo cuando se está diagnosticando la caída).
    static bool commFailPrev = false;
    if (!readOk) {
        if (!commFailPrev)
            Serial.printf("[SAFE] comms BQ caídas > %lu ms y sin recuperar → BMS_OK LOW, parar carga.\n",
                          commWindowMs);
        commFailPrev = true;
        return false;
    }
    if (commFailPrev) {
        Serial.println(F("[SAFE] comms BQ recuperadas → BMS_OK HIGH."));
        commFailPrev = false;
    }

    // Dentro de la ventana de gracia: el último intento de lectura falló
    // (fComm.cond == true) pero el fallo AÚN no está confirmado. El driver
    // puede devolver caché parcial o valores centinela para NTC/T → NO evaluar
    // V/T/NTC; se asume seguro y se espera a que el debounce confirme o
    // el BQ se reconecte. La ventana máxima es commWindowMs.
    if (fComm.cond) {
        return true;
    }

    // ── Debounce de celda (como main.cpp): muestrea la condición cada llamada
    //    (cada SAMPLE_MS) y solo confirma si persiste la ventana + ≥2 muestras.
    //    Así un valor espurio por ruido (con CRC válido) NO corta la carga; el
    //    retry del driver ya cubre el ruido de transporte (COMM/CRC). ──
    bool badV   = (bms.getMaxVoltage() >= CELL_VMAX_HARD_V);
    bool badT   = (bms.getMaxTemp() >= CELL_TMAX_CHG_C) ||
                  (bms.getMinTemp() <= CELL_TMIN_CHG_C);
    bool badNtc = bms.hasOpenNtc();
    fV.sample(badV, now);
    fT.sample(badT, now);
    fNtc.sample(badNtc, now);

    if (fV.confirmed(now, FAULT_V_MS)) {
        Serial.printf("[SAFE] celda %.3f V >= %.2f V (OV, confirmado) → parar.\n",
                      bms.getMaxVoltage(), CELL_VMAX_HARD_V);
        return false;
    }
    if (fT.confirmed(now, FAULT_T_MS)) {
        Serial.printf("[SAFE] T fuera de rango (min=%.1f max=%.1f, confirmado) → parar.\n",
                      bms.getMinTemp(), bms.getMaxTemp());
        return false;
    }
    // NTC abierto/inválido: pérdida de medida térmica. El driver EXCLUYE los NTC
    // abiertos de get{Min,Max}Temp (centinela -1000) → hay que mirarlo aparte.
    if (fNtc.confirmed(now, FAULT_NTC_MS)) {
        Serial.printf("[SAFE] %u NTC abierto(s) (confirmado) → parar.\n",
                      bms.getOpenNtcCount());
        return false;
    }
    // Amperímetro: fallo confirmado (desconexión, stuck, ruido, sobre-I, ADC
    // saturado) → no fiarse de la medida de corriente de carga.
    // isOK() ya viene con el debounce propio del HallSensor.
    if (!hall.isOK()) {
#if CHG_HALL_BLOCKS
        Serial.printf("[SAFE] Hall FALLO (%s%s%s%s%s) → parar.\n",
                      hall.isDisconnected() ? "desc "   : "",
                      hall.isStuck()        ? "stuck "  : "",
                      hall.isNoisy()        ? "noisy "  : "",
                      hall.isOverCurrent()  ? "sobreI " : "",
                      hall.isAdcSaturated() ? "adcSat"  : "");
        return false;
#else
        // TEMP: no corta (divisor 350A sin arreglar). Aviso cada 5 s, no cada ciclo.
        static unsigned long tHallWarn = 0;
        if (millis() - tHallWarn >= 5000) {
            tHallWarn = millis();
            Serial.println(F("[HALL] fallo IGNORADO (temporal, CHG_HALL_BLOCKS=0)"));
        }
#endif
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
    case 'r':
        Serial.println(F("Restart..."));
        delay(100);
        NVIC_SystemReset();
        break;
    default:
        break;
    }
}
