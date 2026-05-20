# BMS Master — Arquitectura y explicación del código

**Target:** STM32G474RE (NUCLEO-G474RE) · PlatformIO `env:nucleo_g474re`
**Vehículo:** Formula Student Electric · Normativa: FS Rules 2026 (EV5.8 / EV6)

> Estado: firmware **funcionalmente completo y compilando**, **NO validado
> en HW**. Ver §9 (Limitaciones) antes de confiar en él como sistema de
> seguridad.

---

## 1. Propósito

Monitorizar el acumulador de tracción (TS) y gobernar la señal `BMS_OK`
del Shutdown Circuit (SDC): medida de corriente, estimación de SOC,
refrigeración y telemetría CAN. **El balanceo NO forma parte de este
firmware** — se hace off-car con una herramienta aparte.

El acumulador: cadena daisy-chain de BQ79606A-Q1 (N ICs), agrupados en
módulos de 2 boards (par: 6 celdas + 6 NTC; impar: 5 celdas + 3 NTC).
Celda: **Samsung INR21700-40T**, 4.0 Ah NCA, carga 4.2 V, corte 2.5 V.

---

## 2. Mapa de pines (STM32G474RE)

| Función | Pin | Notas |
|---|---|---|
| BQ WAKE / FAULT / RX / TX | PB7 / PB2 / PC5 / PC4 | en `BQConfig` |
| `BMS_OK` → SDC | PB5 | lo escribe el driver (`setBmsOk`) |
| `MC_OK` (micro vivo) | PA6 | HIGH en setup |
| `PRE_FAIL` | PA7 | precarga fallida |
| `OE_TXS` (level shifter) | PB10 | gated por VIO_3V3 |
| VIO_3V3 / PRECHARGE_DONE / SDC_3V3 | PB0 / PA5 / PC7 | entradas |
| Amperímetro DHAB 30A / 350A | PA1 / PA0 | HallSensor |
| CAN (FDCAN1) RX / TX | PA11 / PA12 | fijo en la lib |
| PWM ventiladores | PB4 | 2/3 hilos, baja frecuencia |
| FRAM I²C1 SCL / SDA | PB8 / PB9 | MB85RC256V @ 0x50 (FaultLogger) |

> ⚠ Los pines no-BQ se usan con la API Arduino; **verificar el mapeo
> físico en HW** (que compile no garantiza el pin correcto, ni que PB4
> tenga PWM en ese timer).

---

## 3. Modelo de seguridad (lo más importante)

### 3.1 `BMS_OK` — NO-latching, con debounce, auto-rearma

El **latch que abre el SDC es HARDWARE no programable** (EV6.1.6). Por
tanto el firmware trata `BMS_OK` como una **señal de salud**:

- Fallo confirmado presente → `BMS_OK = LOW`.
- Fallo despejado → `BMS_OK` vuelve a `HIGH` automáticamente.
- El HW mantiene el SDC abierto hasta el **reset manual humano**.
- El firmware **nunca** sostiene ni rearma el latch (eso es del HW).
- El indicador rojo "AMS" lo enciende el latch HW → **cero código FW**.

> Esto invierte el P0 #1 del gap-analysis original (que asumía latch SW).

### 3.2 Debounce por normativa (EV5.8)

Un fallo solo baja `BMS_OK` si **persiste**:

| Fallo | Ventana | Fuente |
|---|---|---|
| Voltaje (UV/OV) | ≥ 500 ms | V cada **250 ms**, solo si lectura fiable |
| Temperatura (UT/OT) | ≥ 1000 ms | T cada **500 ms** |
| NTC abierto (pérdida de medida, EV5.8.13) | ≥ 1000 ms | `bms.hasOpenNtc()` |
| Comms BQ caídas | ≥ 500 ms | con `reInit()` rate-limited (#3) |
| Corriente (sobre-I, Hall) | ≥ 500 ms | debounce interno de HallSensor |

Las cadencias de muestreo son **2× más rápidas que la ventana FS** ⇒
hay ≥ 2 muestras dentro de cada ventana. El `FaultTimer` exige
**K = 2 muestras malas consecutivas** *Y* la ventana de wall-time
para confirmar el fallo (un "OK" cualquiera resetea la serie). Con
esto: una muestra ruidosa aislada se filtra (no llega a tripar), un
fallo persistente sigue tripando a los 500 ms / 1000 ms de wall-time
exactos (FS-compliant).

Implementado con `struct FaultTimer` (timestamp por condición). Un error
que se corrige antes de su ventana **no** dispara `BMS_OK`.

El balanceo no forma parte de este firmware (se hace off-car); este
master nunca activa los FETs → la V siempre es fiable (sin ventana
ciega de tensión).

---

## 4. Estructura del proyecto

```
src/main.cpp            Orquestación (setup/loop) + lógica BMS_OK + CAN
lib/BQ79606/            Driver de la cadena BQ (VALIDADO HW, banco 20 ICs)
lib/HallSensor/          Amperímetro DHAB S/118 (portado + corregido)
lib/SocEstimator/        SOC: coulomb counting + OCV (header-only)
lib/FanController/       Ventiladores: curva Tmax + feed-forward (header-only)
lib/FaultLogger/         Log persistente en FRAM MB85RC256V (I²C1 PB8/PB9)
```

Cada lib encapsula una responsabilidad; `main.cpp` solo orquesta y
mantiene la cadena de fallo / `BMS_OK`.

---

## 5. Flujo del `loop()`

```
updateVio()           OE_TXS = VIO_3V3
hall.update()         lee amperímetro (cada ciclo, máx resolución)
sampleAndEvaluate()   V@500ms + T@1000ms + NTC + comms → fija
                      las condiciones de fallo (FaultTimer.cond)
soc.update()          coulomb counting + re-snap OCV en reposo
fan.update()          curva sobre Tmax + feed-forward por I
updateBmsOk()         confirma fallos (debounce) → bms.setBmsOk(!fault);
                      gestiona episodio de fallo (telemetría)
updatePrecharge()     timer 5 s → PRE_FAIL
updateCanTx()         arma IDs 10-14, 386-392; send() (throttle por timer)
handleSerial()        comandos de diagnóstico
printStatus() @2s     volcado por serie
```

---

## 6. Componentes

### 6.1 BQ79606 (driver) — validado HW
V/T de toda la cadena, autoaddressing, detección de **NTC abierto**,
control de balanceo HW. API usada: `begin/reInit`, `readVoltages/
readTemperatures`, `getMin/MaxVoltage/Temp`, `hasOpenNtc`, `setBmsOk`,
`getFaultStatus`, etc.

### 6.2 Balanceo — FUERA DE ALCANCE
El balanceo de celdas NO forma parte de este firmware: se hace
off-car con una herramienta aparte. La implementación validada en HW
vive en el repo BQ_CLASS (rama `refactor-opt-stm32`), no aquí.

### 6.3 HallSensor — DHAB S/118 doble rango
Coulomb-source y watchdog (desconexión incl. 1 canal, congelado por
dither ADC, ruido, sobre-I) con debounce 500 ms. `begin()` fuerza
`analogReadResolution(12)` (STM32 es 10-bit por defecto). Umbrales
`[TUNE]` pendientes de ruido real.

### 6.4 SocEstimator (header-only)
Híbrido: **coulomb counting** (∫I·dt / capacidad) + **re-snap OCV** en
reposo (|I| bajo y estable). Tabla OCV→SOC **NCA genérica** (el
datasheet no trae curva OCV) → SOC **orientativo** hasta caracterizar.
`SOC_PACK_CAPACITY_AH = 4.0 × Np` (ajustar Np).

### 6.5 FaultLogger (FRAM MB85RC256V via I²C)
Log persistente de eventos en FRAM externa (32 KB, I²C1 PB8/PB9, addr 0x50).
Resuelve "BMS_OK se cayó en pista y el CAN tampoco funcionaba" — los
eventos quedan grabados aunque no haya telemetría. Ring buffer de 2047
records de 16 bytes (BOOT, BMS_OK fall/rise, reInit, precharge fail).
Lectura por comando serie `d` (dump) / `D` (clear). Endurance FRAM
10¹² ciclos → seguro toda la vida del coche.

### 6.6 FanController (header-only)
Curva sobre **Tmax** del pack: OFF<32 °C / ON≥35 °C (histéresis) /
rampa 30→100 % entre 35–50 °C / 100 % ≥50 °C. **Feed-forward**: |I|
alta fuerza piso de duty (anticipa inercia térmica). 100 % a 50 °C deja
10 °C de margen al corte FS de 60 °C. PWM 2/3 hilos baja-f (PB4).

---

## 7. Protocolo CAN (docs externo: `Mapa_CAN.txt`)

FDCAN1, 500 kbps, perfil BMS. `setPacketTimer` con los periodos del
mapa; `setPacket`+`send()` cada loop (throttle por timer).

| IDs | Contenido |
|---|---|
| 10 | Estado general (bits = fallo CONFIRMADO actual, no-latch) |
| 11 | MaxT, MaxV(mV), MinV(mV), MinT |
| 12/13 | GEN_STATUS / contadores — **encoding PROVISIONAL** |
| 14 | lastFailTime, numCommFails, numCrcFails, numTriesReset |
| **15** | **BMS_DEBUG (8 B)** — granularidad de fallos para diagnóstico: B0 confirmados, B1 sub-fallos Hall, B2 V/T snapshot, B3 state, B4 primer trigger, B5-6 duración, B7 reset cause |
| 386–389 | por módulo: IDmod + V1..V11 + VTotal (paginado) |
| 390–391 | por módulo: IDmod + T1..T9, Tmax/min, status |
| 392 | SOC (%) |

386–392 **paginado**: 1 módulo por ronda (~556 ms), el receptor lo
identifica por el campo IDmodule. Mapeo Vn↔paralelo CONFIRMADO
(2026-05-20): V1..V6 = par[0..5], V7..V11 = impar[0..4], canal 5
del impar sin usar. VTotal = suma de los 11 paralelos en mV.

---

## 8. Configuración (knobs marcados en el código)

| Marca | Significado |
|---|---|
| `[TUNE]` | umbral dependiente del HW real (ruido, corriente, PWM) |
| `[AJUSTAR]` | depende de la topología del pack (p.ej. `Np`) |
| `PROVISIONAL` | semántica asumida, falta confirmación oficial |
| `[TUNE-datasheet]` | confirmar contra el datasheet de la celda |

Principales: `CELL_UV/OV/UT/OT_*`, `TOTALBOARDS` (en BQ79606.h, **debe
ser el nº exacto de ICs**: 20 banco / 24 pack), `SOC_PACK_CAPACITY_AH`,
`FAN_*`, `[TUNE]` del HallSensor, periodos/IDs CAN.

---

## 9. Limitaciones y trabajo pendiente (LEER)

### 9.1 Nada validado en HW
La lib BQ79606 sí (banco 20 ICs). El `main` nuevo
(BMS_OK, precarga, integración), HallSensor, SOC, fans y CAN: **solo
compilan**. No es producción hasta validar en banco.

### 9.2 V real durante el balanceo — N/A (resuelto por diseño)
Este firmware **no balancea** (el balanceo se hace off-car con otra
herramienta). Por tanto los FETs de balanceo nunca se activan desde
aquí y la tensión de celda es **siempre fiable**: la OV/UV se evalúa
de forma continua sin ventana ciega. El riesgo original desaparece.

### 9.3 Watchdog independiente — RESUELTO (#1)
IWDG hardware (LSI), 8 s. `IWatchdog.begin()` al final de `setup()`
(tras init/calib acotados; una vez armado no se puede parar) y
`IWatchdog.reload()` al final de cada `loop()` completo. Un cuelgue
en cualquier punto → no se refresca → reset → `BMS_OK` cae
(fail-safe). Traza si el reset previo fue del WDG.

### 9.4 Init / reInit no bloqueantes — MITIGADO a nivel `main` (#3 opción B)
- Boot: si `bms.begin()` falla, NO se cuelga esperando 'i'. Se marca
  `bmsInitOk=false` (con `bmsFault=true` → `BMS_OK` LOW) y el loop
  arranca. Cada `BMS_REINIT_RETRY_MS=2 s` se reintenta `reInit()`
  desde `sampleAndEvaluate()` hasta éxito. Comando 'i' sigue como
  disparo manual.
- Pérdida de comms en operación: `reInit()` se llama con rate-limit
  (uno cada 2 s), no en cada flanco/loop. Entre intentos el loop
  corre a velocidad normal (CAN/fans/eval/IWDG siguen activos).
- Residuo: el `reInit()` cuando *sí* se llama bloquea unos segundos
  (toca el driver validado, opción C, para eliminarlo del todo). El
  IWDG de 8 s sigue cubriéndolo. Si se mide en banco y queda con
  margen, se puede bajar el IWDG para detectar cuelgues antes.

### 9.5 CAN bus-off / gate TX — RESUELTO (#4)
`updateCanTx()` se inhibe si el FDCAN no inicializó OK (`gCanOk`).
Cada 1 s llama a `rebootBusFromError()` (barato si el bus está sano:
solo chequea `FDCAN_PSR_BO`; solo recupera si bus-off).

### 9.6 Otros
- Fans fail-safe — RESUELTO (#5): T no fiable (lectura T fallida /
  comms / NTC abierto) → ventiladores 100 %.
- Umbrales UV/OV/UT/OT — CONFIRMADOS contra datasheet INR21700-40T
  (rev. 2026-05-20): UV 2.80 V, OV 4.20 V (estricto al máx.), UT −20 °C,
  OT 60 °C (= FS EV5.8.4).
- `Np` — CONFIRMADO = 11 (12 módulos × 11s11p, capacidad pack 44 Ah).
- Mapeo Vn↔paralelo en CAN 386-389 — CONFIRMADO (2026-05-20): par antes
  que impar, canales del BQ en orden ascendente, dummy del impar arriba.
- Datos a confirmar (banco/equipo): caracterización OCV-SOC, baud FDCAN
  (asume reloj kernel 24 MHz), semántica oficial ID 12/13.
- `TOTALBOARDS` 20→24 para el pack completo (+ re-validar).

---

## 10. Build / flash

```
pio run   -e nucleo_g474re                 # compila
pio run   -e nucleo_g474re -t upload       # flashea (OpenOCD/ST-Link)
pio device monitor -b 115200               # monitor serie
```

Ocupación actual: RAM ~4 % / Flash ~14 % de un STM32G474RE.

---

## 11. Comandos serie (diagnóstico)

`v` voltajes · `t` temps · `a` amperímetro · `s` status · `f` fallos
BQ · `c` limpiar fallos BQ · `i` re-init BQ · `r` reset MCU ·
`d` dump del log FRAM · `D` clear del log FRAM.
