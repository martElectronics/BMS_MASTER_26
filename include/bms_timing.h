#pragma once
// =============================================================================
//  bms_timing.h — UMBRALES Y TIEMPOS del BMS Master (única fuente de verdad)
//
//  Extraído de main.cpp para que el firmware de producción y el banco de
//  inyección de fallos (src/fault_bench.cpp, env `fault_bench`) validen contra
//  EXACTAMENTE las mismas constantes. Si el banco tuviera su propia copia,
//  pasar sus tests no demostraría nada sobre el firmware real.
//
//  ⚠ Aquí SOLO van constantes. Nada de estado ni de código.
// =============================================================================

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
#define CELL_OV_V      5.2f     ///< Overvoltage (V)
#define CELL_UT_C    -20.0f     ///< Undertemperature (°C)
#define CELL_OT_C     60.0f     ///< Overtemperature (°C) — EV5.8.4: ≤60

// Debounce por normativa FS EV5.8
#define FAULT_V_MS     500UL    ///< V debe persistir ≥500 ms
#define FAULT_T_MS    1000UL    ///< T debe persistir ≥1000 ms
#define FAULT_NTC_MS  1000UL    ///< NTC abierto (pérdida de medida, clase T)

// Rearme del debounce V/T/NTC tras un hueco largo de datos. fV/fT/fNtc solo se
// muestrean cuando la lectura sale OK; si las comms se caen, su badRun/tStart
// quedan CONGELADOS. Sin este rearme, tras un apagón de 30 s bastarían k
// muestras malas para confirmar AL INSTANTE (now - tStart ≫ ventana) en vez de
// re-medir los 500/1000 ms contra datos frescos. El umbral se pone por encima
// de la cadencia de muestreo para que un fallo de lectura AISLADO (ruido) NO
// rearme y no retrase un fallo de celda real que se estaba acumulando.
#define FAULT_REARM_GAP_MS 1000UL  ///< hueco sin lectura buena que invalida el debounce

// ---- COMUNICACIÓN CON LA CADENA BQ ----------------------------------------
// Ventana LARGA y deliberadamente relajada: NO es un fallo de celda FS (esos
// son V/T/NTC arriba, con sus 500/1000 ms intocables). El criterio es
// "¿sigue viva la cadena?", y merece tiempo para recuperarse sola.
//
//   BMS_OK cae por comms SOLO si pasan FAULT_COMM_MS enteros sin UN SOLO ciclo
//   de lectura bueno (V y T OK en la misma pasada). Cualquier ciclo bueno
//   REINICIA el reloj a 0 (FaultTimer::sample(false) borra badRun y tStart), y
//   el proceso entero puede repetirse si vuelve a fallar.
//
// Escalado de la recuperación DENTRO de la ventana (peldaños, doc §9.4):
//   t < COMM_SOFT_RETRY_MS  → solo RE-LEER. El driver ya reintenta la trama
//                             (BQ_READ_ATTEMPTS); casi todo episodio es ruido y
//                             muere aquí, sin tocar el direccionamiento.
//   t ≥ COMM_SOFT_RETRY_MS  → además AUTO-ADDRESS (reInit) cada
//                             COMM_REINIT_RETRY_MS, con lecturas normales entre
//                             intento e intento. Se repite hasta que un ciclo
//                             bueno reinicie el reloj o expire la ventana.
// ⚠ reInit() BLOQUEA ~1.5 s por intento de auto-address con la cadena muerta
//   (240 ms de WAKE + 600 ms de delays fijos + ~450 ms de verificación a 10 ms
//   de timeout por board + trazas por serie; ~1.65 s con TOTALBOARDS=24), y
//   aquí se le llama repetidamente durante hasta FAULT_COMM_MS. Con los 5
//   intentos del driver serían ~7.5 s (~8.2 s a 24 boards) contra los 8 s de
//   WDG_TIMEOUT_US → COMM_REINIT_ATTEMPTS=1. La repetición la da la ventana,
//   no los reintentos internos del driver.
// ⚠ Coste asumido — el loop se para mientras reInit bloquea. Con cadencia 2 s y
//   bloqueo 1.5 s el loop está parado ~75 % del episodio, y eso tiene DOS
//   efectos, ambos acotados por la DURACIÓN DEL BLOQUEO (1.5 s), no por la
//   cadencia:
//     · hall.update() se queda ciego a ratos: una sobreintensidad TRANSITORIA
//       (HALL_FAULT_MS=250 ms) que empiece y acabe dentro de un bloqueo se
//       pierde entera (el debounce del Hall es wall-time y repinea tStart, así
//       que una sobre-I que PERSISTA solo se detecta tarde, no se pierde).
//     · una ventana de comms BUENA más corta que el bloqueo puede caer entera
//       dentro de él → no se muestrea y NO reinicia el reloj de 30 s. Solo
//       afecta a comms intermitentes: una cadena que se recupera y SE QUEDA se
//       detecta en la primera lectura tras el bloqueo.
//   Subir COMM_REINIT_RETRY_MS baja el % de tiempo ciego (3 s → 50 %) pero NO
//   el peor caso, que lo fija el bloqueo. La solución de fondo es el reInit no
//   bloqueante (doc §9.5, mejora D).
#define FAULT_COMM_MS        30000UL  ///< 30 s sin NI UNA comunicación buena → fallo
#define COMM_SOFT_RETRY_MS    1000UL  ///< fase blanda: solo re-leer, sin reInit
#define COMM_REINIT_RETRY_MS  2000UL  ///< cadencia entre auto-address en la fase dura
#define COMM_REINIT_ATTEMPTS       1  ///< intentos de auto-address por llamada a reInit()
#define BQ_AUTOADDR_ATTEMPTS       5  ///< default del driver, restaurado tras cada reInit()

// INIT: el arranque NO entra en la ventana relajada de comms. Si la cadena no
// está al boot, BMS_OK debe caer YA: el firmware nunca ha hablado con el stack,
// no puede afirmar salud. Es inocuo — el TSON no puede armar con BMS_OK LOW.
// La relajación de 30 s cubre PERDER una cadena que ya funcionaba, no el nunca
// haberla tenido.
#define FAULT_INIT_MS 200UL     ///< Init BQ fallido persistente (antes: inmediato)

// Cadencias de muestreo: 2× respecto al mínimo FS para tener ≥2 muestras
// dentro de cada ventana de debounce → mejor filtrado de ruido transitorio.
// Las ventanas FAULT_V_MS / FAULT_T_MS NO se tocan (las marca FS EV5.8).
#define SAMPLE_V_MS    100UL
#define SAMPLE_T_MS    100UL
#define PRINT_MS      2000UL

// Watchdog HW independiente: si el loop() se cuelga y no se refresca,
// el IWDG resetea el MCU → BMS_OK cae a LOW (fail-safe). 8 s: por encima
// del peor caso normal incl. reInit() (que bloquea — ver §9.4 del doc;
// bajar este valor exige hacer reInit no bloqueante).
#define WDG_TIMEOUT_US        8000000UL

// Tras armar SDC_TSON, PRECHARGE_DONE debe llegar antes de este tiempo o
// PRECHARGE_FAIL se enclava HIGH (solo se quita con reset de alimentación).
#define PRECHARGE_TIMEOUT_MS  5000UL

// Filtro anti-ruido de SDC_3V3 (PC7): tiempo que el nivel debe mantenerse
// ESTABLE antes de aceptarlo. Sube este valor si siguen colandose glitches;
// es tambien el retardo con el que el firmware ve una apertura REAL del SDC
// (los AIRs ya han abierto por HW para entonces).
#define SDC_FILTER_MS         150UL

// Cadencia del camino de ARRANQUE (!bmsInitOk). El camino de pérdida de comms
// en caliente tiene la suya (COMM_REINIT_RETRY_MS): son escenarios distintos y
// se afinan por separado. tLastReinit lo comparten porque son excluyentes (el
// camino de arranque hace return antes de llegar al de comms).
#define BMS_REINIT_RETRY_MS  2000UL          ///< cadencia mín. entre reintentos al boot
#define BOOT_REINIT_ATTEMPTS      1          ///< auto-address por reintento (WDG: ver sampleAndEvaluate)
