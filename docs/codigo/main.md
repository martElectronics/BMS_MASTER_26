# `main.cpp` — Firmware del BMS Master

## Propósito
Orquesta todo el BMS: lee el pack (tensiones, temperaturas, corriente), decide si
es seguro (`BMS_OK`), gestiona el arranque del Tractive System (**TSON** + precarga),
publica telemetría por CAN y deja rastro en FRAM para diagnóstico. Es firmware de
**producción** (PCB nueva) en la rama `main`.

## Arquitectura de seguridad (leer primero)
- El **latch que abre el SDC es hardware** no programable (EV6.1.6). El firmware
  **nunca** sostiene el latch.
- `BMS_OK` (PB5) es una señal de **salud**: NO-latching, con **debounce**, y
  **auto-rearma** cuando el fallo se despeja. **Polaridad: OK = HIGH, fallo = LOW**
  → un micro muerto deja LOW → SDC abre (fail-safe).
- El indicador rojo "AMS" lo enciende el latch HW → **sin código**.

## Flujo (`loop()`)
1. **`sampleAndEvaluate()`** — lee tensión cada `SAMPLE_V_MS` (250 ms) y temperatura
   cada `SAMPLE_T_MS` (500 ms) del BQ79606; cuenta `COMM_ERROR`/`CRC_ERROR`; reintenta
   `reInit()` rate-limited si las comms caen.
2. **`updateBmsOk()`** — confirma fallos con **debounce K-de-N + ventana** (V≥500 ms,
   T≥1000 ms, comms≥500 ms); calcula `bmsFault`; escribe `BMS_OK`; incrementa los
   **contadores por causa** (flanco) y loguea transiciones en FRAM.
3. **`updateTson()`** — máquina de estados del TSON + temporizador de precarga.
4. `hall.update()` · `soc.update()` · `fan.update()` — corriente, SOC, ventiladores.
5. **CAN** (IDs 10-16) — telemetría a 125 kbps.
6. `IWatchdog.reload()` — si el loop se cuelga → reset → `BMS_OK` LOW.

## Máquina TSON + precarga (`updateTson()`)
- **Armado:** el latch `SDC_TSON`(PA6) se pone HIGH al **pulsar el botón** (PB9) **solo
  si**: `BMS_OK`=HIGH, `HV_ACCU_VIL`(PB4)=LOW, `SDC_3V3`(PC7)=HIGH y `TSON_FAIL`(PB8)=LOW.
- **Mantenimiento:** sigue HIGH mientras `SDC_3V3`=HIGH y `!TSON_FAIL`; si cualquiera
  cambia, cae y hay que **re-pulsar**. Anti auto-arme en boot (lee el botón al arrancar).
- **Precarga:** al armar `SDC_TSON`, `PRECHARGE_DONE`(PA7) debe llegar antes de
  `PRECHARGE_TIMEOUT_MS` (5 s) o `PRECHARGE_FAIL`(PB6) se **enclava HIGH** (solo se
  quita con reset de alimentación).

## Drivers de `bmsFault`
`faultV` · `faultT` · `faultNtc` (NTC abierto = pérdida de medida) · `faultComm`
(comms BQ) · `faultHall` (amperímetro) · `faultInit` (BQ no inicializado). Cualquiera
confirmado → `BMS_OK` LOW.

## Telemetría CAN (IDs 10-16, 125 kbps)
Estado general (0x0A), GEN_STATUS bitmap por módulo (0x0C), señales de SDC/TSON (ID15),
contador de fallos por causa (ID16), V/T, SOC. Ver [docs/CAN_Solo2DL.md](../CAN_Solo2DL.md).

## Comandos serie (115200)
`v` voltajes · `t` temps · `a` amperímetro · `s` status · `f` fallos · `c` limpiar
fallos BQ · `i` re-init BQ · `r` restart · `d` volcar log FRAM · `D` reset índice log
· `C` reset contadores.

## Configuración que tocarás
| Qué | Dónde | Nota |
|---|---|---|
| **`TOTALBOARDS`** | `BQ79606.h` | nº EXACTO de ICs (banco 20 / pack 24). Recompilar + RE-VALIDAR. |
| Umbrales celda UV/OV/UT/OT | `main.cpp` (`CELL_*`) | confirmados vs datasheet 40T |
| Ventanas de fallo | `main.cpp` (`FAULT_*_MS`) | las marca FS EV5.8 — **no relajar** (ver build de banco en rama `testing`) |
| Pines PCB | `main.cpp` (`PIN_*`) | pinout de la PCB nueva |
| `WDG_TIMEOUT_US` | `main.cpp` | 8 s (cubre el `reInit()` que bloquea) |

## Gotchas / pendientes
- **`TOTALBOARDS`** mal = la cadena no responde o lee de menos. Es el error nº1.
- `reInit()` **bloquea** unos segundos → el IWDG está a 8 s para cubrirlo. Bajar el
  watchdog exige hacer `reInit` no bloqueante.
- Pérdida de comms BQ = pérdida de medida → fallo (EV5.8.13). El debounce ya filtra
  glitches; **no** subir las ventanas en producción (usa el env `bms_bench` de `testing`).
