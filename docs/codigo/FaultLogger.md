# `FaultLogger` — Log de eventos en FRAM

`lib/FaultLogger/FaultLogger.h` · `.cpp`

## Propósito
Registra eventos de fallo en una **FRAM no volátil** para **diagnóstico post-mortem**:
cuando el CAN falla o el coche se apaga, los datos persisten. Es la pieza que permite
saber **por qué** se abrió el SDC después de que pasara (junto al contador de fallos
y la telemetría de la VCU).

## Hardware
- **MB85RC256V** (Fujitsu/Adafruit) — 32 KB, I²C @ 400 kHz.
- **I²C3: SCL = PC8, SDA = PC9** (en la PCB nueva; PB8/PB9 son el TSON).
- Dirección 0x50. Endurance 10¹² ciclos → loguear cada evento es seguro de por vida.

## Estructura en FRAM
- `[0..15]` **Header** (magic "MART", versión, writeIdx, rollovers, totalEv).
- `[16..32767]` **Ring buffer** de 2047 eventos × 16 bytes. Al llenarse, da la vuelta
  (incrementa `rollovers`) — siempre tienes los últimos 2047.

## Eventos (`EventType`)
| Código | Evento |
|---|---|
| `EVT_BOOT` (1) | arrancó `setup()` (con `resetCause` válido) |
| `EVT_BMS_OK_FALL` (2) | `bmsFault` pasó a true (SDC se va a abrir) |
| `EVT_BMS_OK_RISE` (3) | el fallo se despejó |
| `EVT_REINIT_TRY` (4) | intento de `reInit()` del BQ |
| `EVT_PRECHARGE_FAIL` (5) | timeout de 5 s de precarga |

Cada `FaultRecord` (16 B) guarda además: primer trigger del episodio, los flags del
CAN ID15 (snapshot de estado), `resetCause`, y min/max V (mV) + Tmax (°C).

## Cómo se usa (`main.cpp`)
```cpp
FaultLogger logger;
logger.begin();                 // setup: init I²C3, valida/forma el header
logger.log(record);            // en cada transición relevante
logger.dumpToSerial(tick);     // comando 'd' — vuelca todo en orden cronológico
logger.clearLog();             // comando 'D' — resetea índices (no borra datos)
```
> El `tick` de `dumpToSerial` es un callback para **refrescar el IWDG** durante volcados
> largos (>8 s) y que no salte el watchdog.

## Diagnóstico (caza de fallos)
Para los dos fallos que perseguimos: tras un episodio, **volcar con `d`** y leer el
`resetCause` del `EVT_BOOT` (¿brownout? ¿IWDG?) y la secuencia de `BMS_OK_FALL/RISE`.
Ver el plan de pruebas [PRUEBAS_ELECTRONICA.md](../PRUEBAS_ELECTRONICA.md) (FM-1/FM-2).
