# Inmunidad al ruido — palancas disponibles y cuál usar

Documento de trabajo para atacar los fallos por ruido del BMS. Recoge **qué
protege hoy el código**, **qué no**, y las palancas concretas ordenadas por
relación valor/coste. Pendiente de implementar — aquí solo está el análisis y
el plan.

---

## 1. Lo que YA filtra el código (tres capas)

| Capa | Qué cubre | Dónde |
|---|---|---|
| **CRC-16 por trama** | Ruido de transporte: una trama corrupta se descarta ENTERA, su valor ni se evalúa | `checkCRC()` |
| **Retry por lectura** | Re-pide al board que falla hasta `BQ_READ_ATTEMPTS`=3 veces (~1 ms) | [BQ79606.cpp `_readBoardRetry`](../lib/BQ79606/BQ79606.cpp) |
| **Debounce K-de-N** | Exige **k=2 muestras malas CONSECUTIVAS** + ventana de tiempo. Una sola lectura buena resetea la racha | [FaultTimer.h](../lib/FaultTimer/FaultTimer.h) |

**Consecuencia importante:** el ruido del **bus de comunicación NO puede
disparar un fallo de V o T**. `fV.sample()` solo se ejecuta `if (lastResV ==
BQResult::OK)`, es decir, solo con trama que pasó CRC. Para colarse, el ruido
tendría que producir un CRC-16 válido por casualidad (~1/65536) y repetirlo en
2 lecturas seguidas.

Lo que **sí** llega es el ruido en la **medida analógica**: el BQ mide un valor
sucio y lo transmite perfectamente. Ahí el CRC no protege de nada.

---

## 2. Diagnóstico ANTES de tocar nada

No disparar a ciegas. Cuando falle, pulsar `s` y mirar:

```
Cnt(ID16): V=.. T=.. NTC=.. COMM=.. HALL=..
```

| Sube | Es | Ir a |
|---|---|---|
| **COMM** | Problema de comunicación | §3 |
| **V** o **T** | Problema de medida | §4 |
| Ambos | EMI general que pega a las dos cosas | §5 (hardware) |

Y del serial, el tipo concreto:

| Mensaje | Interpretación |
|---|---|
| `[BQ] V FALLO CRC en board N` | Trama corrupta → **ruido en la señal** |
| `[BQ] V FALLO COMM en board N` | Board mudo → alimentación, reset o cadena rota |
| Siempre el mismo board | Problema **localizado** (conector, tramo, ese chip) |
| Boards aleatorios | Ruido **general** en la cadena |

**Punto ciego actual:** el retry del driver recupera el ruido leve **sin dejar
rastro**. Con 1-2 reintentos por lectura todo parece perfecto; al superar los 3,
falla de golpe. No se ve venir la degradación → ver mejora #1.

---

## 3. Palancas para fallos de COMUNICACIÓN

### 3.1 Contador de reintentos del driver ⭐ (la que más falta hace)

Exponer cuántos reintentos consume `_readBoardRetry` por ciclo. Es la métrica
que convierte *"falla y no sé por qué"* en *"veo el ruido subir 3 s antes"*.

- Contador en el driver + getter, y publicarlo por CAN (queda hueco en el ID 13
  o ID 17 B7 ya está ocupado por bus-off → valorar ID nuevo).
- Coste: bajo. Valor: **alto** — es la única forma de medir el margen real.

### 3.2 `BQ_READ_ATTEMPTS` 3 → 6-8

[BQ79606.h:69](../lib/BQ79606/BQ79606.h). Cada reintento es otra oportunidad de
colar una lectura buena entre el ruido. Targeted (solo el board que falla), no
cuesta duty de loop.

- Coste: 1 línea. Valor: alto si el síntoma es CRC.
- ⚠ Sube el peor caso de duración de la lectura → vigilar el watchdog.

### 3.3 `TX_HOLD_OFF` (hoy `0x00`)

Registro `0x22` del BQ, se escribe en `_initDevices()`. Un retardo entre bytes
da margen en cadenas largas o ruidosas.

- Coste: 1 línea. Valor: medio, depende de la cadena.

### 3.4 Baudrate del daisy-chain — SIN MARGEN

Ya está en **125 kbps**, el mínimo que soporta `_commReset()` (`0x303C`). No se
puede bajar más. Descartado.

---

## 4. Palancas para fallos de MEDIDA (V / T)

Aquí el CRC no ayuda: el dato viaja bien, lo que está sucio es la medida.

### 4.1 Filtro de mediana de 3 ⭐

Antes de evaluar, usar la **mediana de las últimas 3 lecturas** en vez del valor
crudo de `getMinVoltage()` / `getMaxVoltage()`.

- Mata outliers aislados **sin retrasar nada** (la mediana se calcula al
  instante, no promedia en el tiempo).
- Coste: bajo. Valor: **alto**. Es la mejora más limpia del bloque.

### 4.2 Filtro de slew-rate

Descartar una lectura si salta más de ~200 mV respecto a la anterior: una celda
**no puede** cambiar así en 250 ms sin una corriente enorme. Si lo hace, es
artefacto, no física.

- Coste: bajo. Valor: alto, y además permite distinguir **sense abierto** de
  **UV real** (ver §6).

### 4.3 Más decimación en el ADC del BQ

`CELL_ADC_CONF1` (hoy `0x67`) en `_initDevices()`. Filtrado **por hardware**
dentro del chip, antes de que el valor salga.

- Coste: 1 línea + validar contra el datasheet. Valor: alto.
- Es la opción más "honesta": no maquilla nada en software, mejora la medida de
  origen. ⚠ Más decimación = conversión más lenta; comprobar que sigue
  cumpliendo la cadencia de muestreo.

---

## 5. Lo que NO funciona: subir el debounce

**Ya comprobado en banco: con `FAULT_V_MS=500` sigue saltando.** Eso significa
que hay ≥3 lecturas malas **seguidas** — una anomalía de medio segundo continuo,
no un glitch.

Contra algo sostenido, subir la ventana **solo retrasa la reacción**, no filtra:

- El filtrado real lo hace la exigencia de **k muestras consecutivas**, no el
  tamaño de la ventana. Una sola lectura buena resetea la racha.
- Subir la ventana a 5-150 s hace que el fallo REAL tarde eso en abrir el SDC,
  incumpliendo FS EV5.8 (V ≤500 ms, T ≤1000 ms).

**Alternativa correcta si se quiere más selectividad sin perder tiempo de
reacción:** muestrear más rápido y subir `k` en `confirmed()`, manteniendo la
ventana normativa. Ej.: `SAMPLE_V_MS=100` + `k=5` → 5 consecutivas dentro de los
mismos 500 ms, mucho más selectivo que 3 consecutivas.

---

## 6. Lo que el código NO distingue hoy

**UV real vs sense abierto/flojo.** Ambos dan lectura baja y se tratan igual
(corta la carga / abre el SDC), que es lo conservador y correcto — pero no dice
cuál es. Se podrían separar con tres señales que hoy no se miran:

- Valor **no físico** (<0.5 V o negativo): una celda viva nunca baja de ~2 V.
- **Celda aislada** con las vecinas normales (sense) vs módulo/pack entero (UV real).
- Aparición **de golpe** (sense) vs progresiva (UV real).

Propuesta: umbral de "no físico" que reporte `sense abierto` en vez de `UV`,
**sin cambiar la reacción** (sigue cortando).

---

## 7. Hardware — donde está la solución de verdad

Si el ruido es **EMI sostenida** (el caso del board 0 durante la precarga), el
firmware solo puede compensar hasta cierto punto. Pasado ese punto se estaría
**enmascarando una medida que ya no es fiable**, que es justo lo que no se
quiere en un BMS.

- Trenzar / apantallar el mazo del daisy-chain.
- Ferritas en el mazo del BQ.
- Alejar el cableado de sense del **bucle de precarga / HV**.
- Revisar masas y la referencia del board base.
- Condensadores de filtro en las entradas de sense.

---

## 8. Orden de ataque propuesto

| # | Mejora | Síntoma que ataca | Coste | Valor |
|---|---|---|---|---|
| 1 | Contador de reintentos del driver | Medir antes de tocar | Bajo | ⭐⭐⭐ |
| 2 | Filtro de mediana de 3 | V/T sucios | Bajo | ⭐⭐⭐ |
| 3 | `BQ_READ_ATTEMPTS` → 6 | CRC/COMM | 1 línea | ⭐⭐ |
| 4 | Slew-rate + "no físico" | V sucios, sense abierto | Bajo | ⭐⭐ |
| 5 | Decimación del ADC | V/T sucios (de origen) | 1 línea | ⭐⭐ |
| 6 | Apantallado / ferritas / ruteo | EMI real | HW | ⭐⭐⭐ |
| 7 | **No abortar la lectura + recuperación proporcionada** (§9) | Un board tumba el pack | Medio | ⭐⭐⭐ |

**Regla:** implementar **1 primero**, medir, y elegir el resto con el dato en la
mano. Sin la métrica de reintentos se trabaja a ciegas.

> El **#7 (§9)** es el de mayor impacto estructural, pero también el único que
> toca el camino de seguridad del driver: requiere su propia sesión de banco.
> Los #1-#5 son acotados y se pueden meter de uno en uno.

---

## 9. Recuperación proporcionada ante fallo de lectura ⭐

El bloque más importante del documento. Hoy **un solo board ruidoso ciega el
pack entero y además dispara un auto-address completo** — dos reacciones
desproporcionadas al problema real.

### 9.1 El problema: la lectura se aborta al primer fallo

[BQ79606.cpp:213-221](../lib/BQ79606/BQ79606.cpp) (idéntico en `readTemperatures`):

```c
for (uint8_t board = 0; board < TOTALBOARDS; board++) {
    BQResult r = _readBoardRetry(board, VCELL1H, buf, sizeof(buf), MAXBYTES);
    if (r != BQResult::OK) { _lastReadFailBoard = board; return r; }  // ← ABORTA
    ...
}
```

Si falla el board 7, **los boards 8-19 no se leen** y se pierde el ciclo entero.
Consecuencias:

- La seguridad se queda **ciega de todo el pack** por culpa de un board. Si en
  ese instante el módulo 3 tenía un OV, no se detecta.
- `fComm` se dispara igual que si la cadena entera estuviera muerta.
- Solo queda `_lastReadFailBoard` (el último), no **cuántos** ni **cuáles**.

### 9.2 Mejora A — leer todos los boards, marcar los fallidos

Sustituir el `return` por "marcar y continuar":

```c
uint32_t failMask = 0;                     // bit b = board b fallido este ciclo
uint8_t  failCount = 0;
BQResult worst = BQResult::OK;

for (uint8_t board = 0; board < TOTALBOARDS; board++) {
    BQResult r = _readBoardRetry(board, VCELL1H, buf, sizeof(buf), MAXBYTES);
    if (r != BQResult::OK) {
        failMask |= (1UL << board);
        failCount++;
        _lastReadFailBoard = board;
        worst = r;                          // CRC_ERROR / COMM_ERROR
        continue;                           // ← SEGUIR con los demás
    }
    for (uint8_t c = 0; c < 6; c++) { ...decodificar... }
}
_boardFailMask = failMask;                  // nuevos miembros de la clase
_boardFailCount = failCount;
_updateVoltageStats();                      // ⚠ debe EXCLUIR los de failMask
return (failCount == 0) ? BQResult::OK : worst;
```

**Requisitos que NO se pueden saltar:**

1. `_updateVoltageStats()` y `_updateTempStats()` deben **excluir** los boards de
   `failMask`. Si no, entran datos **rancios** del ciclo anterior en el min/max
   → se podría perder un OV real o inventar uno falso. Es el punto crítico.
2. Getters nuevos: `getBoardFailMask()`, `getBoardFailCount()`.
3. La aplicación debe seguir tratando la pérdida parcial como **fallo**
   (EV5.8.13: pérdida de medida = fallo) — pero **sin cegar el resto del pack**.

**Ganancia:** con 1 board fallido conservas 19/20 módulos con datos **válidos**
y la vigilancia sigue viva sobre ellos.

### 9.3 Mejora B — escalar la recuperación según CUÁNTOS fallan

Sale casi gratis con la A, y es la que evita el cañonazo. `reInit()` cierra la
UART, hace WAKE, comm-reset, re-direcciona los 20 boards y reconfigura todos los
registros: **1-5 s bloqueando**, totalmente ciego. Hacer eso porque un board dio
CRC malo es desproporcionado.

| `failCount` | Interpretación | Reacción propuesta |
|---|---|---|
| **1-2 boards** | Ruido localizado: conector, tramo, ese chip | **NO tocar la cadena.** Seguir leyendo; solo esos módulos pierden medida |
| **Todos desde el board N** | Cadena rota a partir de N | Auto-address (ahí sí procede) |
| **Todos, incluido el 0** | Base muda / cadena caída | Auto-address + WAKE |

Cómo distinguir el patrón con `failMask`:

```c
bool contiguoDesdeN = (failMask != 0) &&
                      ((failMask & (failMask + (failMask & -failMask))) == 0);
bool incluyeBase    = (failMask & 1) != 0;
```

Aplicado al caso real del **board 0 en la precarga**: si falla **solo** el bit 0,
sabes que es el base y no la cadena; si se encienden todos, es la cadena entera.
Hoy los dos casos son indistinguibles.

### 9.4 Mejora C — recuperación escalonada

Entre "reintentar trama" (~1 ms) y "reinicializar la cadena" (~5 s) **no hay
nada**. Faltan peldaños intermedios y baratos:

| Nivel | Acción | Coste | Estado |
|---|---|---|---|
| 1 | Reintento de trama (`BQ_READ_ATTEMPTS`) | ~1 ms | ✅ ya está |
| 2 | **Re-sincronizar el bus** (comm-clear / comm-reset sin re-direccionar) | ~ms | ❌ `_commClear()` **ya existe** en el driver pero NO se usa en recuperación |
| 3 | Releer solo el board problemático con más reintentos | ~ms | ❌ |
| 4 | Auto-address completo | 1-5 s | ✅ es lo único que hay hoy |

⚠ `_commClear()` tal como está deja el TX en LOW y **no reabre la UART**: habría
que completarlo (o usar `_commReset()`, que sí renegocia el baudrate) antes de
meterlo en el camino de recuperación.

### 9.5 Mejora D — `reInit()` no bloqueante

Convertirlo en máquina de estados para no parar el loop varios segundos. Hoy
está parcheado con `IWatchdog.reload()` a ambos lados y con
`setMaxAttempts(1)` durante la precarga. Es la de más trabajo y la menos
urgente: dejarla para el final.

### 9.6 Orden y riesgo

1. **9.2 (A)** — leer todos y marcar. Toca el **camino de seguridad** del driver:
   el riesgo está en la exclusión de los boards fallidos del min/max. Validar en
   banco desconectando un board a propósito y comprobando que el resto sigue
   midiendo y que ese módulo NO aporta datos rancios.
2. **9.3 (B)** — decidir la recuperación por `failCount`/`failMask`.
3. **9.4 (C)** — peldaños intermedios.
4. **9.5 (D)** — no bloqueante.

**No implementar A+B con prisa antes de rodar**: cambian cómo se comporta la
seguridad ante pérdida de medida. Merecen su sesión de banco.

---

## 10. Estado actual de los parámetros (referencia)

| Parámetro | main.cpp | charger.cpp | Normativa FS |
|---|---|---|---|
| `FAULT_V_MS` | 500 | 200 | **500** |
| `FAULT_T_MS` | 1000 | 400 | **1000** |
| `FAULT_NTC_MS` | 1000 | 400 | 1000 |
| `FAULT_REARM_GAP_MS` | 1000 | — | — |
| `FAULT_COMM_MS` | **30000** | 1000 (`f,<ms>`) | — |
| `COMM_SOFT_RETRY_MS` | 1000 | — | — |
| `COMM_REINIT_RETRY_MS` | 2000 | — | — |
| `COMM_REINIT_ATTEMPTS` | 1 | — | — |
| `FAULT_INIT_MS` | 200 | — | — |
| `BMS_REINIT_RETRY_MS` | 2000 (solo boot) | — | — |
| `SAMPLE_V_MS` / `SAMPLE_MS` | 100 | 250 | — |
| `SAMPLE_T_MS` | 100 | 250 | — |
| `BQ_READ_ATTEMPTS` | 3 | 3 | — |
| `k` de `FaultTimer::confirmed` | 5 (default) | 5 (default) | — |

⚠ El default de `k` en `FaultTimer.h` es **5**, no 2, y **ningún** call site pasa
`k` explícitamente: el default gobierna los ~25 `confirmed()` de los dos
firmwares. Los tests `test_dos_malas_y_ventana` y `test_overflow_millis` de
`test/test_faulttimer/` siguen escritos para k=2 y **fallan** por eso.

`confirmed()` exige **las dos** condiciones a la vez, así que el tiempo real de
confirmación es `max(ventana, (k−1) × periodo_de_muestreo)`. **k solo manda
cuando `(k−1) × periodo > ventana`**; si no, es inerte:

| fw | timer | periodo | ventana | k=2 | k=5 |
|---|---|---|---|---|---|
| main | `fV` | 100 | 500 | 500 ms | 500 ms |
| main | `fT` / `fNtc` | 100 | 1000 | 1000 ms | 1000 ms |
| main | `fComm` / `fInit` | cada loop | 30000 / 200 | igual | igual |
| charger | `fV` | 250 | 200 | 250 ms | **1000 ms** |
| charger | `fT` / `fNtc` | 250 | 400 | 500 ms | **1000 ms** |
| charger | `fComm` | 250 | 1000 | 1000 ms | 1000 ms |

En **main** k es inerte: k=2 y k=5 dan exactamente lo mismo. En **charger**
(`SAMPLE_MS`=250, más lento que sus ventanas) k=5 **cuadruplica** la
confirmación de V (250 → 1000 ms) y duplica T/NTC (500 → 1000 ms), por encima
del límite FS de 500 ms para tensión. La intención del diseño era k=2: lo dicen
los tests, y tres comentarios del charger (`FAULT_COMM_MIN_MS` "suelo: <
`SAMPLE_MS` no da ni 2 muestras", y "≥2 muestras" en las líneas 108 y 545).

Coste de margen: con k=5 la cadencia no puede pasar de `ventana/4` sin retrasar
la confirmación. `SAMPLE_V_MS` está en 100 ms contra un techo de 125 ms — subirlo
a 150 ms rompería el límite FS de 500 ms **en silencio**. Con k=2 el techo es la
propia ventana (500 ms).

### 10.1 Comunicación BQ: ventana relajada + recuperación escalonada

Las comms **no** son un fallo de celda FS, así que tienen un criterio propio y
mucho más laxo que V/T/NTC:

> BMS_OK cae por comms solo si pasan **`FAULT_COMM_MS` (30 s) enteros sin un
> solo ciclo de lectura bueno**. Ciclo bueno = `readVoltages()` **y**
> `readTemperatures()` OK en la misma pasada. Cualquier ciclo bueno reinicia el
> reloj a 0 (`FaultTimer::sample(false)` borra `badRun` y `tStart`) y el proceso
> puede repetirse entero si vuelve a fallar.

Escalado **dentro** de la ventana (peldaños de §9.4):

| Fase | Cuándo | Qué hace |
|---|---|---|
| Trama | siempre | `BQ_READ_ATTEMPTS`=3 reintentos dentro del driver (~ms) |
| Blanda | t < `COMM_SOFT_RETRY_MS` (1 s) | solo re-leer; no se toca el direccionamiento |
| Dura | t ≥ `COMM_SOFT_RETRY_MS` | + auto-address cada `COMM_REINIT_RETRY_MS` (2 s), con lecturas normales entre intentos |
| Fallo | t ≥ `FAULT_COMM_MS` (30 s) | `bmsFault` → BMS_OK LOW. El escalado **sigue** (fallo no latcheado, debe poder rearmar) |

`COMM_REINIT_ATTEMPTS`=1: `reInit()` **bloquea ~1,5 s** por intento de
auto-address con la cadena muerta (240 ms de WAKE `delay(12×TOTALBOARDS)` +
600 ms de delays fijos + ~450 ms de verificación a 10 ms de timeout por board +
trazas por serie; ~1,65 s con `TOTALBOARDS`=24). Con los 5 del driver serían
**~7,5 s** (**~8,2 s a 24 boards**) contra los 8 s de `WDG_TIMEOUT_US`. La
repetición la da la ventana, no los reintentos del driver.

⚠ **Coste del bloqueo.** Con cadencia 2 s y bloqueo 1,5 s el loop está parado
~75 % del episodio. Dos efectos, ambos acotados por la **duración del bloqueo**,
no por la cadencia:

- `hall.update()` se queda ciego a ratos. Una sobreintensidad **transitoria**
  (`HALL_FAULT_MS`=250 ms) que empiece y acabe dentro de un bloqueo se pierde
  entera. Una que **persista** solo se detecta tarde (el debounce del Hall es
  wall-time y repinea `tStart`), no se pierde.
- Una ventana de comms **buena más corta que el bloqueo** puede caer entera
  dentro de él → no se muestrea y **no reinicia el reloj de 30 s**. Medido en
  simulación: la ventana buena mínima siempre detectable es ~1,5 s,
  independientemente de la cadencia. Solo afecta a comms **intermitentes**; una
  cadena que se recupera y se queda se detecta en la primera lectura tras el
  bloqueo.

Subir `COMM_REINIT_RETRY_MS` baja el % de tiempo ciego (3 s → 50 %) pero **no**
el peor caso. La solución de fondo es §9.5 (reInit no bloqueante).

### 10.2 Watchdog en el camino de ARRANQUE

El `bms.begin()` de `setup()` corre **antes** de `IWatchdog.begin()`, así que ahí
los 5 intentos son seguros. Pero el reintento de `sampleAndEvaluate()` cuando
`!bmsInitOk` corre con el **WDG ya armado**: llamaba a `reInit()` con los 5
intentos del driver y **sin** `IWatchdog.reload()` → ~7,5 s de bloqueo contra los
8 s del WDG (y ~8,2 s con `TOTALBOARDS`=24 → **reset garantizado**, bucle de
arranque infinito con la cadena ausente). Corregido: `BOOT_REINIT_ATTEMPTS`=1 y
`reload()` a ambos lados; la repetición la da `BMS_REINIT_RETRY_MS`.

Al recuperar la cadena por esa vía se limpia también `fComm`: durante el arranque
se muestrea a `true` en cada pasada solo para el reloj de comms de ID 13 — no es
la ventana de seguridad viva (al boot manda `fInit`). Sin limpiarlo, su `tStart`
quedaría a decenas de segundos y el primer fallo de lectura tras recuperar
confirmaría al instante en vez de estrenar la ventana de 30 s.

Un auto-address OK **no** reinicia el reloj de 30 s: dice que la cadena responde
al direccionamiento, no que se puedan leer celdas. Solo lo reinicia una lectura
buena de verdad.

⚠ **Consecuencia asumida (decisión de equipo):** hasta 30 s con datos de celda
rancios y BMS_OK aún HIGH, **con el TS encendido incluido**. FS EV5.8.13 trata
la pérdida de medida como fallo; el bit `failCondition` (ID 15 B3) sí se
enciende de inmediato vía `fComm.cond`, mucho antes del confirm.

**Rearme del debounce V/T/NTC** (`FAULT_REARM_GAP_MS`): `fV`/`fT`/`fNtc` solo se
muestrean con lectura OK, así que durante un apagón su `badRun`/`tStart` quedan
congelados. Si la última lectura buena fue hace más de 1 s, se borran antes de
meter el primer dato fresco — si no, tras un apagón de 30 s bastarían k muestras
malas para confirmar al instante (`now - tStart` ≫ ventana) en vez de re-medir
los 500/1000 ms. El umbral (1 s) está por encima de la cadencia de muestreo para
que un fallo de lectura **aislado** no rearme y no retrase un fallo de celda real
que se estaba acumulando.

### Tiempo hasta el auto-address (reInit)

`fComm.sample()` se llama **cada iteración del loop**, así que las k muestras
consecutivas se cumplen en microsegundos → **manda la ventana**.

| Firmware | Situación | Tiempo |
|---|---|---|
| main | Normal | **1 s** (`COMM_SOFT_RETRY_MS`), luego cada 2 s |
| main | Durante precarga | **Inmediato** (1er error), rate-limit 2 s |
| main | BQ sin init al boot | Cada **2 s** (`BMS_REINIT_RETRY_MS`) |
| charger | Normal | **1 s** (`commWindowMs`), rate-limit 2 s |
| charger | Durante precarga | **Inmediato**, rate-limit 2 s |

En precarga (main) se salta la fase blanda: el transitorio de HV puede dejar mudo
el board base y el pulso de WAKE del reInit lo resucita; con
`PRECHARGE_TIMEOUT_MS`=5 s no hay margen para esperar. `setMaxAttempts` ya es 1
en todos los caminos de recuperación en caliente.

**FRAM:** en el camino de comms solo se loguea el **primer** `EVT_REINIT_TRY` de
cada episodio. Loguear los ~14 de una ventana de 30 s (o uno cada 2 s
indefinidamente con la cadena muerta) llenaría las 2047 entradas y borraría el
post-mortem. El boot sí loguea cada intento (son pocos y acotados).
