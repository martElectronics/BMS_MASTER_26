# `HallSensor` — Amperímetro de pack (DHAB S/118)

`lib/HallSensor/HallSensor.h` · `.cpp`

## Propósito
Mide la corriente del pack con el sensor Hall **DHAB S/118**, que tiene **dos salidas**
analógicas (rangos distintos). La clase elige el rango óptimo, filtra, y vigila el
sensor (watchdog) para gobernar `BMS_OK` por sobre-corriente o por fallo del propio sensor.

## Doble rango (con histéresis)
- **Canal 30 A:** alta resolución (66 mV/A nativo). Activo por defecto.
- **Canal 350 A:** baja resolución (4 mV/A). Se usa con |I| grande.
- Cambia 30→350 A a `HALL_THRESH_UP` (30 A) y vuelve a `HALL_THRESH_DOWN` (25 A).

## Cómo se usa (`main.cpp`)
```cpp
HallSensor hall(PIN_AMP_30A, PIN_AMP_350A);   // PA1, PA0
hall.begin();          // setup: fija ADC 12-bit + autocalibra offset (~1 s, EN REPOSO)
hall.update();         // cada loop: lee, filtra, evalúa watchdog
float I = hall.getCurrent();   // A, + = descarga, − = carga
bool  ok = hall.isOK();        // false solo por desconexion / sobre-I / offset de boot malo
```
`isOK()` se usa como driver de `bmsFault` (`faultHall`).

## Watchdog (qué detecta)

**Solo estas dos tumban `isOK()`** (más un offset de boot implausible), y por tanto
`BMS_OK` → el latch **hardware** del SDC:

| Causa de FALLO | Significado |
|---|---|
| `isDisconnected()` | un canal en el raíl sin que el otro corrobore corriente real |
| `isOverCurrent()` | fuera de límites FS (descarga 170 A / carga −7 A) |
| `isOffsetValid()` | offset de `begin()` fuera de rango plausible (~1.62 V). Se evalúa **una vez**, al arrancar |

**Diagnóstico — se mide y se publica, pero NO abre el SDC:**

| Señal | Significado |
|---|---|
| `isStuck()` | el canal activo no varía ni el dither del ADC en 500 ms (congelado) |
| `isNoisy()` | salto de corriente entre lecturas > umbral (EMI) |
| `isAdcSaturated()` | algún canal ADC en el margen de raíl |

Los dos timers de fallo se confirman con `HALL_FAULT_MS` y son **no-latching**: una sola
lectura buena pone el contador a 0 y la ventana vuelve a empezar entera; `isOK()` se
rearma solo en cuanto la condición se despeja.

### Por qué congelado y ruido no son fallo
Sus umbrales son heurísticos y siguen `[TUNE]` sin validar contra el ruido real del HW.
Aquí un falso positivo no degrada nada: **abre el SDC**, y ese latch es hardware y
*latching*, así que un episodio de 250 ms deja el coche parado hasta reset manual.
Desconexión y sobre-I tienen un criterio físico, no estadístico. Reintroducirlos en
`_updateFaultTimers()` es una línea si el banco demuestra que sus umbrales son fiables.

⚠ `_updateFaultTimers()` mide **solo tiempo de pared**, sin exigir N muestras
consecutivas (no hay `k` como en `FaultTimer`). Si el loop se para —p.ej. un `reInit()`
bloqueante de ~1,5 s— y la condición estaba activa antes y sigue activa después, el fallo
confirma con **dos** observaciones. Subir `HALL_FAULT_MS` no protege de eso.

## ⚠ Pendiente de ajuste en banco `[TUNE]`
La **lógica está bien**, pero estos umbrales dependen del **ruido real del HW** y no se
ajustan a ojo — validar con el sensor montado y el coche en marcha:
- `HALL_WD_STUCK_ADC_LSB` — dither mínimo de un canal vivo (congelado vs real). *Solo
  diagnóstico: ya no abre el SDC, así que afinarlo dejó de ser urgente.*
- `HALL_WD_NOISE_DELTA_A` — ΔI de EMI vs el dI/dt legítimo de un acelerón. *Ídem.*
- `HALL_WD_DISC_CORROB_A` — corriente que corrobora un raíl real vs desconexión.
  **Este sí es camino de seguridad** y sigue sin validar.

## Gotchas
- **`begin()` debe calibrar con 0 A reales.** Si el cero sale mal, toda la escala se
  desplaza. En reposo el offset debe salir ≈ **1.62 V**; fuera de `[1.40, 1.85] V` →
  `isOffsetValid()=false` (sensor desconectado/mal alimentado al arrancar).
- `begin()` fuerza `analogReadResolution(12)` — sin esto, en STM32 (10-bit por defecto)
  las lecturas saldrían ~4× mal.
- `HALL_DIVIDER = 0.652` (18k/33k) está **validado en banco** por dos métodos.
