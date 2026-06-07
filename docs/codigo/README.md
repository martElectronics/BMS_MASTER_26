# Documentación de código — BMS Master (rama `main`)

Guías de **onboarding** por módulo: para que alguien que toque el código entienda
qué hace cada parte, cómo encaja y qué es delicado, **sin** tener que leer 1000
líneas de golpe. El código fuente ya está muy comentado (docblocks por función);
estos documentos son la **vista de pájaro** y apuntan al código para el detalle.

## Índice

| Documento | Qué cubre | Fichero(s) |
|---|---|---|
| [main.md](main.md) | Firmware principal del BMS: orquesta todo, seguridad SDC, TSON, precarga, CAN | `src/main.cpp` |
| [BQ79606.md](BQ79606.md) | Driver del monitor de celdas (V/T, fallos, balanceo) | `lib/BQ79606/` |
| [HallSensor.md](HallSensor.md) | Amperímetro de pack (doble rango + watchdog) | `lib/HallSensor/` |
| [SocEstimator.md](SocEstimator.md) | Estimación de SOC (coulomb counting + OCV) | `lib/SocEstimator/` |
| [FanController.md](FanController.md) | Ventiladores (curva sobre Tmax + feed-forward) | `lib/FanController/` |
| [FaultLogger.md](FaultLogger.md) | Log de eventos en FRAM (post-mortem) | `lib/FaultLogger/` |
| [charger.md](charger.md) | Firmware de carga (OBC TC HK-L, env aparte) | `src/charger.cpp` |

> La VCU (otro repo, `mart-cockpit`) tiene su propio documento en `docs/codigo/VCU.md`.

## Arquitectura en 1 minuto

El **STM32G474RE** corre `main.cpp`. En cada `loop()`:

```
sampleAndEvaluate()  → lee V/T del pack (BQ79606) a cadencia + detecta comms caídas
updateBmsOk()        → confirma fallos (debounce) → bmsFault → BMS_OK (HIGH=OK)
                       + contadores de fallo + log en FRAM
updateTson()         → máquina TSON (botón/SDC) + precarga (timeout 5 s)
hall.update()        → corriente de pack (amperímetro)
soc.update()         → SOC del pack
fan.update()         → ventiladores según Tmax + corriente
CAN (IDs 10-16)      → telemetría al datalogger / VCU (125 kbps)
IWatchdog.reload()   → si el loop se cuelga → reset → BMS_OK LOW (fail-safe)
```

**Principio de seguridad (FS EV5.8/EV6):** el latch que abre el SDC es **hardware**.
`BMS_OK` es una señal de **salud** (no-latching, con debounce, auto-rearma).
Polaridad en `main`: **OK = HIGH, fallo = LOW** (micro muerto → LOW → SDC abre).

## Hardware (resumen)

- **MCU:** STM32G474RE (NUCLEO-G474RE).
- **Monitor de celdas:** cadena daisy-chain de ICs BQ79606 (UART 125 kbps, PC4/PC5).
- **Amperímetro:** DHAB S/118 (doble rango, PA0/PA1).
- **FRAM:** MB85RC256V (I²C3 PC8/PC9, 0x50).
- **CAN:** FDCAN1 (PA11/PA12, 125 kbps).
- **SDC/TSON/precarga:** GPIOs (ver [main.md](main.md)).
