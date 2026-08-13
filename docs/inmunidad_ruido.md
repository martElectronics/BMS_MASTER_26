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

### 9.3b Escalar según el TIPO de error (CRC vs COMM) — ✅ IMPLEMENTADO

Complementaria a la 9.3 (que escala según **cuántos** boards fallan): esta
escala según **qué tipo** de fallo es. La información ya estaba en
`lastResV`/`lastResT`, solo que no se usaba — ambos errores disparaban el mismo
`reInit()`.

| Error | Qué significa | Auto-address |
|---|---|---|
| **CRC_ERROR** | El board **respondió**: vivo, direccionado y contestando. Solo llegó sucio | **Inútil** — no hay dirección que reasignar |
| **COMM_ERROR** | No respondió nada: puede haber **perdido la dirección** (reset por EMI/brownout), estar sin alimentación, o cadena rota | **Útil** si perdió la dirección |

**Tres razones para NO re-direccionar ante CRC:**

1. **No arregla el problema.** El board ya tiene dirección y responde; lo que
   falla es la integridad de la señal.
2. **Ciega el BMS 1-5 s.** Durante el auto-address no se lee V ni T. Se cambia
   "una lectura corrupta" por "cinco segundos sin ninguna medida".
3. **⚠ Puede dejar la cadena PEOR.** El auto-address usa **el mismo bus sucio**,
   y su secuencia es mucho más larga y crítica que una lectura (ECC dummy →
   CONFIG → CONTROL1 → 20 escrituras de dirección → base/top → verificación). Si
   el ruido corrompe una trama de 18 bytes, corromperá eso también, y una
   secuencia interrumpida a medias puede dejar boards con direcciones
   inconsistentes. **No es una operación gratuita: tiene riesgo.**

Ante CRC, seguir pidiendo es **barato y reversible**: se reintenta cada ciclo y
en cuanto el ruido pasa se recupera al instante, sin ventana ciega.

**Implementación (main):**
```c
bool chainMute = (lastResV == BQResult::COMM_ERROR) ||
                 (lastResT == BQResult::COMM_ERROR);
bool reinitNow = prechargeRunning ? (readErr && chainMute)
                                  : (chainMute && fComm.confirmed(now, COMM_REINIT_MS));
```
En charger, `readVT()` publica `chainMute` y `readPack()` lo exige en los dos
puntos de reconexión (temprana y precarga). Si `!bmsInitOk` se fuerza a `true`:
sin direccionar, el auto-address es justo lo que falta.

**Resultado:** ruido (CRC) → sigue leyendo hasta que pase, o hasta que
`FAULT_COMM_MS` declare fallo. Cadena muda (COMM) → reconecta.

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
| `FAULT_V_MS` | 150000 ⚠ | 150 | **500** |
| `FAULT_T_MS` | 150000 ⚠ | 500 | **1000** |
| `FAULT_NTC_MS` | 150000 ⚠ | 1000 | 1000 |
| `FAULT_COMM_MS` | 6000 | 1000 (`f,<ms>`) | — |
| `FAULT_INIT_MS` | 200 | — | — |
| `SAMPLE_V_MS` / `SAMPLE_MS` | 250 | 250 | — |
| `SAMPLE_T_MS` | 500 | 250 | — |
| `BQ_READ_ATTEMPTS` | 3 | 3 | — |
| `k` de `FaultTimer::confirmed` | 2 | 2 | — |

⚠ Los marcados son valores **de banco**, muy por encima de la normativa.
Revertir a 500/1000/1000 antes de rodar.

### Tiempo hasta el auto-address (reInit)

`fComm.sample()` se llama **cada iteración del loop**, así que las 2 muestras
consecutivas se cumplen en microsegundos → **manda la ventana**.

| Firmware | Situación | Tiempo |
|---|---|---|
| main | Normal | **6 s** (`FAULT_COMM_MS`) desde el primer error |
| main | Durante precarga | **Inmediato** (1er error), rate-limit 2 s |
| main | BQ sin init al boot | Cada **2 s** (`BMS_REINIT_RETRY_MS`) |
| charger | Normal | **1 s** (`commWindowMs`), rate-limit 2 s |
| charger | Durante precarga | **Inmediato**, rate-limit 2 s |

En precarga se usa además `setMaxAttempts(1)`: un solo intento de auto-address
(~1 s) en vez de 5 (~5 s), para no comerse el `PRECHARGE_TIMEOUT_MS`.
