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
`v` voltajes · `t` temps · `a` amperímetro · `s` status · `f` fallos · **`k` estado
de comms BQ** · `c` limpiar fallos BQ · `i` re-init BQ · `r` restart · `d` volcar log
FRAM · `D` reset índice log · `C` reset contadores (incluye las stats de comms).

### `k` — estado de las comunicaciones
También sale dentro de `s` y del volcado periódico (cada `PRINT_MS`):

```
--- COMMS BQ -------------------------------------------
Estado : DEGRADADO         init=1 autoaddr=0  ultV=COMM ultT=COMM
Tiempo : actual=12400 ms | max=12400 ms | acumulado=17270 ms (2.29% del uptime)
Ventana: 12400/30000 ms consumidos | ult.ciclo bueno hace 12.4 s
Racha  : badRun=255  episodios=15 (1 confirmados)
Errores: COMM=190 CRC=9  ult.board que fallo=0
Recuper: DURA (auto-address periodico)  auto-address=9 intentos  proximo en 800 ms
--------------------------------------------------------
```

| Campo | Significado |
|---|---|
| `Estado` | `OK` / `DEGRADADO` (racha mala en curso, ventana aún no agotada) / `FALLO CONFIRMADO` (BMS_OK LOW por comms) |
| `init` / `autoaddr` | `bmsInitOk` y `bms.isOK()`: pueden diverger — tras un `reInit()` fallido el driver queda sin direccionar (`autoaddr=0`) pero se sigue intentando leer |
| `Tiempo` | **actual** = episodio en curso · **max** = episodio más largo desde el boot · **acumulado** = suma de todo el tiempo en fallo desde el boot (incluye el episodio en curso) |
| `Ventana` | cuánto se lleva consumido de `FAULT_COMM_MS` antes de tumbar BMS_OK; `(AGOTADA)` cuando ya confirmó |
| `Racha` | `badRun` del `FaultTimer`; **episodios** = todos, incluidos los que se despejaron dentro de la ventana; **confirmados** = los que llegaron a tumbar BMS_OK (`cntFltComm`, ID 16) |
| `Recuper` | peldaño actual: `BLANDA` (solo re-leer) / `DURA` (auto-address periódico) / `PRECARGA`. `proximo en` solo aparece si de verdad se va a escalar |

Los tres tiempos son `unsigned long` **sin** el clamp de 16 bits de los campos CAN
(`canLastCommFailMs` / `canMaxCommFailMs` saturan a 65535 ms = 65,5 s; un episodio
con la cadena muerta dura indefinidamente).

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

## Banco de inyección de fallos (`env:fault_bench`)

Valida **en HW real** que las ventanas de debounce y el escalado de recuperación
de comms se comportan como dice el diseño.

```
pio run -e fault_bench -t upload
pio device monitor -e fault_bench      # luego '?' para la ayuda
```

⚠ **El binario del banco fuerza `BMS_OK` a LOW a propósito. NO SUBIRLO AL COCHE.**
Está deliberadamente fuera de `default_envs`, así que `pio run` nunca lo construye.

### Cómo está montado
No duplica lógica. Compila el **`main.cpp` de producción tal cual** más
`src/fault_inject.cpp`, con `-D BMS_FAULT_INJECT`, y solo intercepta las
**entradas** de `sampleAndEvaluate()` (resultado de las lecturas, condiciones
`badV`/`badT`/`badNtc`, flag de precarga) observando las **salidas** (fallos
confirmados). Las ventanas contra las que compara salen de
`include/bms_timing.h`, las mismas que usa el firmware — si el banco tuviera su
propia copia, aprobar no demostraría nada.

Sin el flag, los ganchos de `include/fault_inject.h` son funciones inline vacías
que devuelven su argumento. **Verificado**: el `firmware.bin` de producción es
byte a byte idéntico con y sin los ganchos en el fuente (mismo md5, 92568 B).
Los dos ganchos que el optimizador no puede eliminar solo (la llamada a
`millis()` que cronometra el `reInit`, y el `struct FiState`) van bajo `#ifdef`
por eso.

### Escenarios (miden y dan PASS/FAIL con el valor medido)
| | Qué prueba | Esperado |
|---|---|---|
| `1` | debounce V | `FAULT_V_MS` (500 ms) |
| `2` | debounce T | `FAULT_T_MS` (1000 ms) |
| `3` | NTC abierto | `FAULT_NTC_MS` (1000 ms) |
| `4` | ventana de comms completa | `FAULT_COMM_MS` (30 s) |
| `5` | una lectura buena reinicia el reloj | fallo a ~42 s, no a 30 s |
| `6` | fase blanda: 1er auto-address | `COMM_SOFT_RETRY_MS` (1 s) |
| `7` | fase dura: cadencia | `COMM_REINIT_RETRY_MS` (2 s) |
| `8` | glitch V < ventana | **no** debe confirmar |
| `9` | rearme V tras hueco de comms | `FAULT_V_MS` desde la recuperación |
| `0` | precarga | auto-address al primer error |
| `A` | todos en secuencia (~2 min) | |

`X` aborta y limpia · `Z` traza on/off · `?` ayuda.
Inyección manual (toggles): `Q` badV · `W` badT · `E` badNtc · `Y` lectura V
falla · `U` lectura T falla · `P` precarga.

**Requiere la cadena BQ conectada e inicializada.** Sin ella `sampleAndEvaluate()`
hace `return` antes de leer y no hay entradas que interceptar; los escenarios se
niegan a arrancar con un mensaje explícito.

### Traza en tiempo real
```
[FI] ---------- ESCENARIO #4 ----------
[FI]      +0 ms  INYECTA lecturas V y T fallando (ventana 30000 ms)
[FI]   +2900 ms  REINIT #1  ini=+1800 ms  BLOQUEO=1100 ms  separacion=0 ms  ok=1
[FI]   +5000 ms  ... 4900/30000 ms  badRun=255  reInit=2
[FI]  +30900 ms  SUBE faultCOMM
[FI]  +30900 ms  SUBE bmsFault   (BMS_OK -> LOW)
[FI] === #4 ventana comms      PASS  medido=30900 ms  esperado=30000 +-2500
```
`BLOQUEO` es la duración real medida del `reInit()` — sirve para contrastar en HW
la estimación de ~1,5 s con la cadena muerta / ~1,1 s con cadena viva.

### Tolerancias
Son anchas a propósito y **siempre se imprime el valor medido** para poder
juzgar a ojo. Las dos fuentes de desviación esperables:
- El primer muestreo cae en el siguiente múltiplo de `SAMPLE_V_MS`/`SAMPLE_T_MS`
  (hasta +100 ms).
- El `reInit()` bloquea el loop, así que cruzar la ventana puede detectarse hasta
  ~1,5 s tarde (por eso `#4` tolera ±2500 ms).

Los escenarios `6` y `0` esperan a que el rate-limit de `COMM_REINIT_RETRY_MS`
esté libre antes de medir: si el episodio arrancase dentro de la sombra de un
auto-address anterior, el primero se retrasaría por una razón legítima que no es
la que se quiere probar.
