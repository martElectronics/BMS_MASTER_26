# BMS → AiM Solo 2 DL — Mapa CAN completo

**Fuente:** firmware BMS Master STM32G474RE · MART Formula Student
**Receptor:** AiM Solo 2 DL (RaceStudio 3 → import DBC)

> ✅ **La fuente de verdad para los bits es `docs/BMS_MART.dbc`.** Este
> documento explica **qué es cada canal y por qué importa** (unidades,
> semántica, recetas de debug) para los cuatro nodos del bus (BMS, inversor
> DTI, VCU, PDM) — **no repitas aquí el Start Bit de cada señal**. La
> numeración de bits manual fue justo la causa de varios bugs reales de
> configuración esta temporada (canales congelados en 1, corrientes ×10,
> Start Bit del inversor al revés); mantenerla en dos sitios a la vez es
> la manera más segura de que vuelva a pasar. Si necesitas el bit exacto de
> un canal, ábrelo con el DBC (RaceStudio lo muestra en el panel "Info from
> DBC file" al importar; o `cantools.database.load_file(...)` en Python).

### Glosario (siglas que aparecen todo el rato)

| Sigla | Qué es |
|---|---|
| **SDC** | Shutdown Circuit — el lazo de seguridad en serie (setas, IMD, BMS, inercia...) que corta los contactores si se abre en cualquier punto |
| **TSON** | Tractive System ON — el estado "sistema de tracción armado y con alta tensión conectada". El "latch TSON" es el enclavamiento lógico que mantiene ese estado |
| **IMD** | Insulation Monitoring Device — vigila fugas a masa del lado de alta tensión; si detecta una, abre el SDC |
| **NTC** | sensor de temperatura (termistor) pegado a los módulos de celdas |
| **BQ** (o BQ79606) | el chip de Texas Instruments que mide tensión/temperatura de cada módulo y habla por UART en cadena con el micro del BMS |
| **DHAB** | el sensor de corriente de efecto Hall que mide el amperaje del pack (lo que aquí se llama "Hall") |
| **FDCAN** | el periférico CAN del microcontrolador STM32 (el "bus CAN" físico del coche) |
| **EMI** | interferencia electromagnética — ruido eléctrico que puede colarse en una lectura o en el propio bus |
| **R2D** | Ready to Drive — el coche armado y listo para dar par (debe sonar el buzzer de R2D antes de pisar el acelerador) |
| **APPS** | Accelerator Pedal Position Sensor — los sensores (hay dos, por redundancia) que miden la posición del acelerador |
| **FOC** | Field Oriented Control — el algoritmo de control del motor que usa el inversor; `Id`/`Iq` son sus dos componentes de corriente internos |
| **ERPM** | RPM eléctricos del motor — hay que dividir por los pares de polos para tener las RPM mecánicas reales |
| **"paralelo"** | cada módulo de celdas está hecho de 11 grupos de celdas en paralelo, conectados en serie entre sí; `V1..V11` son la tensión de cada uno de esos 11 grupos dentro de un módulo |
| **debounce** | filtro de "confirmar durante X ms antes de dar el fallo por bueno", para no disparar con un pico de ruido de una sola lectura |
| **DLC** | Data Length Code — cuántos bytes de datos lleva la trama CAN |

---

## 1. Parámetros del bus

| | Valor |
|---|---|
| Bitrate | **125 kbps** (bus del coche) |
| Formato ID | **Standard 11-bit** |
| Byte order (multi-byte) | **Big-Endian (Motorola)** |
| Sample point típico | 75-87.5 % (deja que RaceStudio elija) |
| Terminación | 120 Ω en cada extremo del bus |

> Todas las tramas que llevan campos `uint16_t`/`uint32_t` se transmiten en
> BE porque la librería MART_CAN convierte por defecto. Las tramas de bytes
> sueltos (p.ej. ID 15, 390, 391, 392) no necesitan conversión — los bytes
> van en el orden escrito.

---

## 2. Cómo cargarlo en RaceStudio 3

1. **USB / WiFi → Configurations → New/Edit → Solo 2 DL**.
2. **ECU Stream / CAN2 → Import DBC** y selecciona `docs/BMS_MART.dbc`.
3. RaceStudio crea un canal por cada `SG_` del DBC, con nombre, escala,
   signo y rango ya puestos correctamente — no hace falta tocar Start
   Bit/Length/Gain a mano, que es justo lo que se escribía mal antes.
4. Vais a tener más canales de los que caben en el logger (límite típico
   120, ver aviso de RaceStudio al guardar). Antes de confirmar, desmarca
   los que no necesitas en pista — cada sección de abajo dice si ese bloque
   es telemetría en vivo o forense/post-mortem (candidato a quitar primero).
5. **Transmit configuration → Solo 2 DL** y arranca logging.

> ⚠ **Si alguna vez das de alta un canal a mano** (sin DBC): el campo
> "Start Bit" que escribes en la UI de RaceStudio **no es el mismo número**
> que el `First Bit` que el DBC guarda internamente para un campo de más de
> 1 byte — RaceStudio hace una conversión propia al mostrarlo/editarlo. No
> copies a mano el `First Bit` del DBC en el "Start Bit" manual. Mejor:
> importa el DBC, o si tienes que crear un canal nuevo, créalo y compara el
> grid de bits resaltado contra el campo real del mensaje (el grid nunca
> miente, el número sí puede confundir).

> En el panel del piloto agrupa los canales por color: rojo = `bmsFault` y
> sub-fallos; amarillo = snapshot actual; gris = state. Así de un vistazo
> en directo identificas la causa.

---

## 3. Resumen de IDs del BMS

(El inversor, la VCU y el PDM tienen su propio resumen en §20-22.)

> ⚠ **ID 10 (0x0A, BMS_Status) ELIMINADO 2026-10-10**: 6 de sus 8 flags eran
> duplicado exacto de bits ya presentes en el ID 15 (BMS_Debug) — `StsFail`=
> `BMS_Fault`, `CommErr`=`Flt_Comm`, `VoltErr`=`Flt_V`, `SDC`=`Pin_SDC_3V3`,
> `AutoAddrOk`=`State_AutoAddr` — y `Ok` era el complemento trivial de
> `StsFail`. `AmpErr` se reconstruye en RaceStudio como canal calculado = OR
> de los 5 `Hall_*` del ID 15. Se pierde `CondNow` (sin sustituto directo,
> bajo valor diagnóstico frente a los ya confirmados). Libera 8 canales.

| ID dec | ID hex | DLC | Periodo | Contenido |
|---|---|---|---|---|
| 11 | 0x0B | 8  | 100 ms | MaxT, MaxV, MinV, MinT |
| 12 | 0x0C | 6  | 799 ms | Status V / Status T (bitmap por módulo) + V_DELTA (mV) |
| 13 | 0x0D | 8  | 500 ms | maxFailTime, numTriesReset, max/lastCommFailTime |
| 14 | 0x0E | 8  | 500 ms | lastFailTime, contadores comm/CRC, retryNet |
| **15** | **0x0F** | **8** | **100 ms** | **BMS_DEBUG — granularidad por bit** |
| 16 | 0x10 | 6 | 100 ms | Contadores de fallo por causa (6×UINT8) — *forense* |
| **17** | **0x11** | **8** | **100 ms** | **STICKY SDC/TSON (latcheado) + contador de vida y uptime** |
| **18** | **0x12** | **4** | **100 ms** | **Corriente (Hall) y tensión del pack medidas por el BMS** |
| 386 | 0x182 | 8 | 557 ms paginado | IDmod, V1, V2, V3 — *forense* |
| 387 | 0x183 | 8 | 556 ms paginado | IDmod, V4, V5, V6 — *forense* |
| 388 | 0x184 | 8 | 556 ms paginado | IDmod, V7, V8, V9 — *forense* |
| 389 | 0x185 | 8 | 555 ms paginado | IDmod, V10, V11, VTotal — *forense* |
| 390 | 0x186 | 8 | 554 ms paginado | IDmod, T1..T7 — *forense* |
| 391 | 0x187 | 8 | 554 ms paginado | IDmod, T8, T9, Tmax, Tmin, stV, stT — *forense* |
| 392 | 0x188 | 1 | 553 ms | SOC (%) |

> ⚠⚠ **EL DLC REAL ES SIEMPRE 8**, en TODOS los IDs del coche (BMS, inversor,
> VCU, PDM). La columna "DLC" indica los bytes **con datos útiles**, pero
> `setPacket()` fija `size = 8` siempre y rellena lo que sobra con **0xFF**
> (no con 0). Los bytes de relleno se leen como 255; ignóralos. Esto ya lo
> resuelve el DBC (declara `BO_ ... : 8` en todos), pero si alguna vez montas
> un canal a mano, pon longitud 8 siempre.
>
> ⚠ **Trampa de `configurePacketTimersByPriority()`** (detalle de firmware,
> no de RaceStudio): asigna el periodo **invertido** — `ratio = 1 -
> (id-minId)/(maxId-minId)`, así que el ID **más bajo** se lleva el
> intervalo **más largo** y los altos 50 ms. Por eso los IDs de seguridad
> (11, 13, 14, 15, 16, 17) llevan `setPacketTimer()` explícito después de
> esa llamada en `main.cpp`. Si se añade un ID nuevo que importe, hay que
> ponerle su timer a mano o se va a ~800 ms sin que nadie se dé cuenta.

**Paginado (386-391):** un módulo distinto por ronda (~556 ms entre módulos
del mismo ID). Con 12 módulos, cada módulo se actualiza cada ~6.7 s.
El campo `IDmod` te dice de cuál es la trama.

---

## 4. ID 11 (0x0B) — Métricas V/T · 8 bytes · 100 ms

```
Bytes 0-1: MaxT (int16 BE, °C)
Bytes 2-3: MaxV (uint16 BE, mV)
Bytes 4-5: MinV (uint16 BE, mV)
Bytes 6-7: MinT (int16 BE, °C)
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_MaxT | Tmax | 0-1 | Temperatura máxima del pack (signed) |
| BMS_MaxV | Vmax | 2-3 | Tensión de celda máxima |
| BMS_MinV | Vmin | 4-5 | Tensión de celda mínima |
| BMS_MinT | Tmin | 6-7 | Temperatura mínima del pack (signed) |

> El canal de referencia para verificar que el import del DBC está bien: en
> vivo `BMS_MaxV`/`BMS_MinV` deben leer ~3.7-4.0 V por celda. Si sale un
> valor absurdo (p.ej. ~29000), el import falló o el canal está mal armado.

---

## 5. ID 12 (0x0C) — Status V/T por módulo (BITMAP) + V_DELTA · 6 bytes · 799 ms

Bitmap por módulo (alineado con el Excel CAN): cada bit i = estado del
módulo i, `0=OK 1=FAIL`. Los `NUM_MODULES` bits menos significativos son
válidos (resto = 0). V_DELTA = tensión de celda máxima − mínima del pack
entero, en mV: indicador de desbalanceo.

```
Bytes 0-1: GEN_STATUS_VOLT (uint16 BE) — bit i = módulo i con algún paralelo fuera de [UV,OV]
Bytes 2-3: GEN_STATUS_TEMP (uint16 BE) — bit i = módulo i con algún NTC fuera de [UT,OT]
Bytes 4-5: V_DELTA         (uint16 BE) — (celda máx − celda mín) en mV
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_StatusV | GSV_ | 0-1 | bitmap volt por módulo |
| BMS_StatusT | GST_ | 2-3 | bitmap temp por módulo |
| BMS_VDelta  | VDLT | 4-5 | delta de celda máx−mín (mV) |

> V_DELTA solo es fiable cuando la lectura de celdas lo es: una celda abierta
> (0 V) lo dispara. Léelo junto a `Flt_Comm`/`ReadV_CommErr` del ID 15 para
> no asustarte con un pico espurio.
>
> El detalle celda a celda (qué paralelo/NTC concreto) está en los frames
> paginados 386-391 y en el snapshot del ID 15 byte 2 — ver §8.

---

## 6. ID 13 (0x0D) — Estadística acumulada · 8 bytes · 500 ms

```
Bytes 0-1: maxFailMs (uint16 BE, ms)        — duración máx vista desde boot
Bytes 2-3: numTriesReset (uint16 BE)        — reintentos de reInit BQ
Bytes 4-5: maxCommFailMs (uint16 BE, ms)    — duración máx de un episodio de comms
Bytes 6-7: lastCommFailMs (uint16 BE, ms)   — duración del episodio de comms actual/último
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_MaxFailMs | MFLM | 0-1 | duración máx de un episodio de fallo (ms) |
| BMS_NumTryReset | NTR_ | 2-3 | reintentos de reinicialización del BQ |
| BMS_MaxCommFailMs | MCFM | 4-5 | duración máx de un episodio de comms (ms) |
| BMS_LastCommFailMs | LCFM | 6-7 | duración del episodio de comms actual/último (ms) |

> ⏱ **Los tiempos cuentan desde que se DETECTA la anomalía**, no desde que el
> debounce la confirma: el reloj arranca en la primera muestra mala y para
> cuando la condición se despeja, así que incluye la ventana de debounce.
>
> Consecuencia útil: también se cronometran los episodios que **no llegaron a
> confirmar**. Si `MaxFailMs` es menor que la ventana de debounce (500 ms
> para V, 1000 para T, 2000 para comms), hubo una anomalía real que se
> resolvió sola **sin abrir el SDC** — justo lo que interesa para cazar ruido.
>
> `MaxCommFailMs`/`LastCommFailMs` son el mismo reloj pero solo para las
> comms del BQ, para separar "cuánto duró el fallo de comunicación" de
> "cuánto duró el episodio de fallo en general".

---

## 7. ID 14 (0x0E) — Último episodio · 8 bytes · 500 ms

```
Bytes 0-1: lastFailMs (uint16 BE)           — duración del episodio actual/último
Bytes 2-3: numCommFails (uint16 BE)
Bytes 4-5: numCrcFails (uint16 BE)
Byte  6  : retryNet (int8)                  — §3.1/§3.3 doc ruido, ver nota abajo
Byte  7  : relleno (0xFF)
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_LastFailMs  | LTFT | 0-1 | duración del episodio actual/último (ms) |
| BMS_NumCommFail | NCFA | 2-3 | nº de fallos de comunicación del episodio |
| BMS_NumCrcFail  | NCRC | 4-5 | nº de fallos de CRC del episodio |
| BMS_RetryNet    | RTRN | 6   | ver nota — **signed** |

> `BMS_RetryNet` = reintentos de `_readBoardRetry` del **último ciclo** V+T
> (ver `docs/inmunidad_ruido.md` §3.1), con **signo** en vez de dos canales
> separados, para no gastar un canal nuevo de RaceStudio:
> - **`> 0`** → `N` reintentos por **COMM** (board mudo) en el último ciclo.
> - **`< 0`** → `N` reintentos por **CRC** (dato corrupto). Lee el valor
>   absoluto como el recuento.
> - **`= 0`** → ciclo limpio, ningún reintento.
> - Es `(comm_retries) − (crc_retries)` saturado a `[-128, 127]`. En todas
>   las pruebas de banco COMM y CRC no han aparecido a la vez en el mismo
>   ciclo, así que la resta no cancela en la práctica. Si predominan los
>   positivos → candidato a `BQ_READ_ATTEMPTS`; si predominan los negativos
>   → candidato a mediana/slew-rate — ver el árbol de decisión del doc de
>   pruebas de ruido.

---

## 8. ID 15 (0x0F) — BMS_DEBUG · 8 bytes · 100 ms

**El frame clave para diagnosticar caídas de BMS_OK en pista.**

```
B0: confirmados (drivers de BMS_OK LOW)
B1: sub-fallos del Hall
B2: snapshot ACTUAL V/T (sin debounce)
B3: state (init/CAN/precharge/pines)
B4: enum FirstFault del episodio
B5-6: duración del episodio (u16 ms BE)
B7: causa del último reset
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_Flt_V       | FV__ | 0 | UV/OV confirmado (≥500 ms) |
| BMS_Flt_T       | FT__ | 0 | UT/OT confirmado (≥1000 ms) |
| BMS_Flt_NTC     | FNTC | 0 | NTC abierto confirmado |
| BMS_Flt_Comm    | FCOM | 0 | Comms BQ caídas confirmadas |
| BMS_Flt_Hall    | FHAL | 0 | HallSensor !OK |
| BMS_Flt_NoInit  | FINI | 0 | `bmsInitOk = false` |
| BMS_Fault       | BFLT | 0 | **OR global → BMS_OK LOW** |
| Hall_Disc       | HDIS | 1 | DHAB desconectado |
| Hall_Stuck      | HSTK | 1 | Sensor congelado |
| Hall_Noisy      | HNOI | 1 | EMI / ΔI extremo |
| Hall_OverI      | HOVI | 1 | Sobre-I FS (>170 A desc / <−7 A carga) |
| Hall_AdcSat     | HSAT | 1 | ADC saturado |
| V_UV_now        | NUV_ | 2 | minV < 2.80 V (snapshot, sin debounce) |
| V_OV_now        | NOV_ | 2 | maxV > 4.20 V (snapshot, sin debounce) |
| T_UT_now        | NUT_ | 2 | minT < −20 °C (snapshot, sin debounce) |
| T_OT_now        | NOT_ | 2 | maxT > 60 °C (snapshot, sin debounce) |
| NTC_open_now    | NNTC | 2 | `hasOpenNtc` (snapshot, sin debounce) |
| ReadV_CommErr   | RVCO | 2 | última lectura V = COMM_ERROR |
| ReadV_CrcErr    | RVCR | 2 | última lectura V = CRC_ERROR |
| ReadT_Err       | RTER | 2 | última lectura T ≠ OK |
| State_AutoAddr  | SADR | 3 | `bms.isOK` |
| State_BmsInit   | SINI | 3 | `bmsInitOk` |
| State_CanOk     | SCAN | 3 | FDCAN inicializó OK |
| SDC_TSON        | STSN | 3 | Latch TSON cerrado |
| Precharge_Fail  | PFAI | 3 | Precarga falló (enclavado → reset alim.) |
| IMD_OK          | IMDK | 3 | IMD OK |
| Pin_SDC_3V3     | PSDC | 3 | SDC presente |
| TSON_Fail       | TSNF | 3 | TSON_FAIL activo |
| FirstFault      | FFLT | 4 | **Enum**: 0=none 1=V 2=T 3=NTC 4=Comm 5=Hall 6=!Init |
| EpisodeMs       | EPMS | 5-6 | Duración episodio actual (ms, clamp 65535) |
| Rst_LPWR        | RLPW | 7 | Reset por low-power |
| Rst_WWDG        | RWWD | 7 | Reset por window-WDG |
| Rst_IWDG        | RIWD | 7 | **Reset por IWDG (loop colgado)** |
| Rst_SOFT        | RSFT | 7 | Reset software (cmd `r`) |
| Rst_BOR         | RBOR | 7 | Power-on / brown-out |
| Rst_PIN         | RPIN | 7 | Reset por pin NRST |
| Rst_OBL         | ROBL | 7 | Option-byte loader |

> **B3 reorganizado (PCB nueva):** lleva el estado de la máquina TSON.
> `SDC_TSON` = latch cerrado; `Precharge_Fail` = enclavado hasta reset de
> alimentación; `IMD_OK`; `Pin_SDC_3V3`; `TSON_Fail`.

### Recetas de debug (post-mortem)

| Síntoma | Lectura del frame | Causa raíz probable |
|---|---|---|
| `BMS_Fault=1` y `BMS_Flt_Comm=1`, `FirstFault=4` | Comm cayó primero, el resto cascadeó | Cableado del transceiver UART al BQ, EMI |
| `BMS_Flt_Hall=1`, `Hall_Disc=1` | Hall desconectado | Cable del DHAB suelto |
| `BMS_Flt_Hall=1`, `Hall_Noisy=1` | EMI / ΔI extremo | Filtrado de inversor, malla a tierra |
| `BMS_Flt_NoInit=1`, `State_BmsInit=0` | Init falló y aún reintentando | Bus al BQ apagado / `VIO_3V3` ausente |
| `V_UV_now=1` pero `BMS_Flt_V=0` | Glitch UV filtrado por debounce | Ruido en lectura, NO es UV real |
| `Rst_IWDG=1` al boot | Loop colgado | Punto donde se quedó: revisar logs serie |

> **Candidato a desmarcar en RaceStudio si faltan canales:** los snapshots
> sin debounce (`V_UV_now`...`ReadT_Err`, 8 canales) son redundantes con los
> `Flt_*` confirmados para uso en pista — solo hacen falta si estás cazando
> ruido muy fino en banco. Los `Rst_*` (7 canales) solo importan en los
> primeros segundos tras un boot.

---

## 9. ID 16 (0x10) — Contadores de fallo por causa · 6 bytes · 100 ms — *forense*

Cuenta cuántas **veces** ha disparado cada fallo confirmado desde el último
reset del micro (incrementa en el flanco 0→1, así capta episodios cortos
que no verías en vivo). `uint8` saturado a 255. Reset por comando serie `C`
o al reiniciar.

```
Byte 0: cntFltV    — episodios de fallo de tensión
Byte 1: cntFltT    — de temperatura
Byte 2: cntFltNtc  — de NTC abierto
Byte 3: cntFltComm — de comms BQ caídas
Byte 4: cntFltHall — de fallo del Hall
Byte 5: cntFltInit — de init BQ fallido
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_CntV    | CNTV | 0 | nº episodios fallo V |
| BMS_CntT    | CNTT | 1 | nº episodios fallo T |
| BMS_CntNTC  | CNTN | 2 | nº episodios NTC abierto |
| BMS_CntComm | CNTC | 3 | nº episodios comms BQ |
| BMS_CntHall | CNTH | 4 | nº episodios Hall |
| BMS_CntInit | CNTI | 5 | nº episodios init BQ |

> Complementa al ID 15: el ID 15 dice qué pasa **ahora**; el ID 16 dice
> **cuántas veces** ha pasado cada cosa aunque ya se haya despejado. Son
> contadores de banco/post-sesión, no telemetría de pista — primer
> candidato a desmarcar si falta presupuesto de canales.

---

## 10. ID 17 (0x11) — STICKY SDC/TSON · 8 bytes · 100 ms

El ID 15 B3 manda el **nivel actual** de SDC_3V3/TSON_FAIL/IMD_OK: si el
evento dura menos que el periodo de envío, cae entre dos tramas y no deja
rastro. Las 6 flags sticky de aquí quedan **latcheadas desde el boot** —
se muestrean por FLANCO y con la señal **cruda** (antes del debounce), así
que un glitch de 10 ms que ni siquiera llegó a desarmar el TSON sigue
visible después. Reset por comando serie `C` o al reiniciar el micro.

```
Byte 0   : bits sticky (ver tabla)
Byte 1   : cntTsonDisarm — nº de desarmes del TSON
Byte 2   : cntSdcLost    — nº de caídas de SDC_3V3
Byte 3   : cntImdLost    — nº de caídas de IMD_OK
Byte 4   : rolling counter (contador de vida, vuelta cada 25,6 s)
Byte 5-6 : uptime en segundos (uint16 BE, satura a 65535 ≈ 18 h)
Byte 7   : cntBusOff — nº de episodios de bus-off del FDCAN
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_StkSdcLost  | SKSD | 0 | SDC_3V3 cayó alguna vez — **mantener, es la que dice si el coche paró por el BMS o por otro nodo** |
| BMS_StkTsonDis  | SKTD | 0 | el latch TSON se desarmó alguna vez |
| BMS_StkTsonFail | SKTF | 0 | TSON_FAIL se activó alguna vez |
| BMS_StkImdLost  | SKIM | 0 | IMD_OK cayó alguna vez |
| BMS_StkPreFail  | SKPF | 0 | precarga falló (latch duro) |
| BMS_StkBmsFault | SKBF | 0 | bmsFault confirmado alguna vez |
| BMS_CntTsonDis  | CNTD | 1 | nº de desarmes del TSON — *forense* |
| BMS_CntSdcLost  | CNSD | 2 | nº de caídas de SDC_3V3 — *forense* |
| BMS_CntImdLost  | CNIM | 3 | nº de caídas de IMD_OK — *forense* |
| BMS_AliveCnt    | ALIV | 4 | contador de vida (+1 cada 100 ms) — **mantener, heartbeat del BMS** |
| BMS_Uptime      | UPTI | 5-6 | segundos desde el arranque |
| BMS_CntBusOff   | CNBO | 7 | nº de episodios de bus-off del FDCAN — *forense* |

### Contador de vida (B4) y uptime (B5-6)

`ALIV` se deriva del reloj (`millis()/100`), no de un contador incrementado a
mano: va sincronizado con el periodo de envío y no depende de la velocidad
del loop.

| Síntoma | Significado |
|---|---|
| `ALIV` salta más de 1 entre tramas consecutivas | **tramas perdidas** en la recepción (el hueco del log NO es que el BMS dejara de mandar) |
| `ALIV` congelado y siguen llegando tramas | **BMS colgado** (la lógica no avanza) |
| `UPTI` vuelve a 0 | **el MCU se reseteó** → mirar ID 15 B7 para la causa (bit2 = watchdog) |
| Dejan de llegar tramas | BMS sin alimentación, colgado del todo, o bus caído |

### Receta: ¿paró el coche el BMS, o fue otro nodo del SDC?

| SKBF | SKSD | Lectura |
|---|---|---|
| **1** | — | **Fue el BMS.** Mira ID 15 B4 (primer trigger) e ID 16 (qué contador subió) |
| 0 | **1** | **El SDC lo abrió OTRO nodo** (IMD / BSPD / paro / inercia). El BMS solo reaccionó desarmando el TSON |
| 0 | 0 | Ni BMS ni SDC → mirar reset del micro (ID 15 B7; contadores del ID 16 cayendo a 0 = hubo reset) |

---

## 10b. ID 18 (0x12) — Corriente y tensión del pack · 4 bytes · 100 ms

La medida de batería **del propio BMS**, independiente del inversor. La
corriente DC que manda el inversor (`INV DC Current`) es una estimación muy
ruidosa; con esta trama se puede calcular la potencia de batería
(`BMS_PackVoltage × BMS_PackCurrent`) y hacer balances de energía batería ↔
motor fiables. Es también la corriente que de verdad vigila la protección del
BMS.

```
Bytes 0-1: corriente del Hall (int16 BE, 0,1 A)  — + descarga, − carga
Bytes 2-3: tensión del pack   (uint16 BE, 0,1 V) — suma de todas las celdas
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_PackCurrent | IPCK | 0-1 | corriente del pack medida por el sensor Hall del BMS (A, **signed**: positiva descargando, negativa cargando o regenerando) |
| BMS_PackVoltage | VPCK | 2-3 | tensión del pack (V) = suma de las tensiones de celda que leen los BQ |

> `BMS_PackVoltage` es la suma de celdas, no una medida entre bornes: no
> incluye la caída en pletinas, conectores y fusible, así que bajo carga
> saldrá algo por encima de `INV VinDC`. Esa diferencia es justo la caída en
> el cableado de alta tensión.

---

## 11. ID 386 (0x182) — Tensiones V1-V3 · 8 bytes · paginado 557 ms — *forense*

```
Bytes 0-1: IDmod  (uint16 BE)               — índice del módulo (0..N-1)
Bytes 2-3: V1     (uint16 BE, mV)           — paralelo 1 = par[0]
Bytes 4-5: V2     (uint16 BE, mV)
Bytes 6-7: V3     (uint16 BE, mV)
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| Mod_IDmod_386 | IM86 | 0-1 | índice del módulo que ocupa este turno de la ronda |
| Mod_V1        | MV01 | 2-3 | tensión del paralelo 1 del módulo |
| Mod_V2        | MV02 | 4-5 | tensión del paralelo 2 |
| Mod_V3        | MV03 | 6-7 | tensión del paralelo 3 |

> Los 4 IDs 386-389 comparten el mismo `IDmod` en bytes 0-1; cada uno
> necesita su propio short name (`IM86/IM87/IM88/IM89`) porque RaceStudio
> exige nombres únicos. **Bloque forense**: solo hace falta para diagnosticar
> una celda/módulo concreto offline — el dato en vivo que importa
> (`MaxV`/`MinV`/`MaxT`/`MinT`) ya está en ID 0x0B. Primer candidato a
> desmarcar en RaceStudio si falta presupuesto de canales (junto con 387,
> 388, 389, 390, 391 — 32 canales en total).

---

## 12. ID 387 (0x183) — Tensiones V4-V6 · 8 bytes · paginado 556 ms — *forense*

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| Mod_IDmod_387 | IM87 | 0-1 | índice del módulo |
| Mod_V4        | MV04 | 2-3 | tensión del paralelo 4 |
| Mod_V5        | MV05 | 4-5 | tensión del paralelo 5 |
| Mod_V6        | MV06 | 6-7 | tensión del paralelo 6 |

---

## 13. ID 388 (0x184) — Tensiones V7-V9 · 8 bytes · paginado 556 ms — *forense*

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| Mod_IDmod_388 | IM88 | 0-1 | índice del módulo |
| Mod_V7        | MV07 | 2-3 | tensión del paralelo 7 |
| Mod_V8        | MV08 | 4-5 | tensión del paralelo 8 |
| Mod_V9        | MV09 | 6-7 | tensión del paralelo 9 |

---

## 14. ID 389 (0x185) — Tensiones V10-V11 + VTotal · 8 bytes · paginado 555 ms — *forense*

```
Bytes 0-1: IDmod
Bytes 2-3: V10   (uint16 BE, mV)
Bytes 4-5: V11   (uint16 BE, mV)
Bytes 6-7: VTot  (uint16 BE, mV) — suma de los 11 paralelos del módulo
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| Mod_IDmod_389 | IM89 | 0-1 | índice del módulo |
| Mod_V10       | MV10 | 2-3 | tensión del paralelo 10 |
| Mod_V11       | MV11 | 4-5 | tensión del paralelo 11 |
| Mod_VTotal    | MVTT | 6-7 | suma de los 11 paralelos del módulo |

> `Mod_VTotal` debe coincidir con la tensión medida entre terminales del
> módulo (margen ±20 mV por offset del ADC del BQ × 11 celdas).

---

## 15. ID 390 (0x186) — Temperaturas T1-T7 · 8 bytes · paginado 554 ms — *forense*

```
Byte 0: IDmod (uint8)
Byte 1-7: T1..T7 (int8 cada uno, °C)
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| Mod_IDmod_390 | IM90 | 0 | índice del módulo |
| Mod_T1..T7    | MT01..MT07 | 1-7 | temperatura de cada NTC (**signed**) |

> Las temperaturas son **signed int8** (rango −128..+127 °C) — si sale 236
> en vez de −20, el canal no está marcado como Signed (el DBC ya lo marca
> bien). El módulo solo tiene 9 NTCs reales (T1..T9, repartidos en los IDs
> 390 y 391); no hay T10+.

---

## 16. ID 391 (0x187) — T8-T9 + Tmax/min + Status · 8 bytes · paginado 554 ms — *forense*

```
Byte 0: IDmod
Byte 1: T8  (int8 °C)
Byte 2: T9  (int8 °C)
Byte 3: Tmax (int8 °C)  — máx de T1..T9 del módulo
Byte 4: Tmin (int8 °C)  — mín de T1..T9 del módulo
Byte 5: stV (uint8)      — 0=OK 1=UV 2=OV  (módulo)
Byte 6: stT (uint8)      — 0=OK 1=UT 2=OT  (módulo)
Byte 7: reservado (0)
```

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| Mod_IDmod_391 | IM91 | 0 | índice del módulo |
| Mod_T8        | MT08 | 1 | temperatura NTC 8 (signed) |
| Mod_T9        | MT09 | 2 | temperatura NTC 9 (signed) |
| Mod_Tmax      | MTMX | 3 | máx de T1..T9 del módulo |
| Mod_Tmin      | MTMN | 4 | mín de T1..T9 del módulo |
| Mod_StatusV   | MSV_ | 5 | estado de tensión del módulo |
| Mod_StatusT   | MST_ | 6 | estado de temperatura del módulo |

---

## 17. ID 392 (0x188) — SOC · 1 byte · 553 ms

| Canal | Short | Byte | Descripción |
|---|---|---|---|
| BMS_SOC | SOC_ | 0 | Estado de carga del pack (%) |

> SOC **orientativo** hasta caracterizar la curva OCV-SOC real de las
> Samsung INR21700-40T (ver `ARQUITECTURA.md §9.6`). El coulomb counting
> con `Np=11` (44 Ah pack) ya está bien.

---

## 18. Convención de naming

Dos sistemas en paralelo: **nombre largo** (descriptivo, el que ya trae el
DBC) y **short name** (4 chars, el que se ve en gauges/displays de AiM —
el DBC no los fija, tócalos en RaceStudio si colisionan con otro protocolo).

### Nombre largo (grupos por prefijo)

- `BMS_*` → telemetría general (IDs 11-14, 392).
- `Hall_*` → sub-fallos del amperímetro (ID 15 B1).
- `V_*` / `T_*` / `NTC_*` → snapshots (ID 15 B2).
- `State_*` / `Pre_*` / `Pin_*` → state del sistema (ID 15 B3).
- `Rst_*` → causa del último reset (ID 15 B7).
- `Stk_*` → sticky latcheados desde el boot (ID 17).
- `Mod_*` → detalle por módulo (IDs 386-391).
- `INV_*` → inversor DTI HV-500 (§20).
- `VCU_*` / `APPS*` / `Brake*` → VCU (§21).
- `PDM_*` → refrigeración (§22).

### Short name (4 chars, AiM RaceStudio)

| Prefijo | Categoría | Ejemplos |
|---|---|---|
| `F` | Fault confirmado (driver de BMS_OK) | FV, FT, FNTC, FCOM, FHAL, FINI |
| `B` | BMS global / boolean | BFLT (BMS_Fault) |
| `E` | Error específico (ya retirado con el ID10, se mantiene por historial) | — |
| `H` | Hall sub-fault | HDIS, HSTK, HNOI, HOVI, HSAT |
| `N` | Snapshot "ahora" (sin debounce) | NUV, NOV, NUT, NOT, NNTC |
| `R` | Reset / Read-error | RIWD, RBOR, RVCO, RVCR, RTER |
| `S` | State / status | SADR, SINI, SCAN, SOC |
| `P` | Precarga / Pin físico | POK, PFAI, PSDC |
| `M` | Módulo (paginados 386-391) | MV01..MV11, MVTT, MT01..MT09, MTMX/MTMN |
| `IM` | IDmod (uno por ID paginado) | IM86..IM91 |
| `FF` | First Fault (enum) | FFLT |
| `EP` | Episodio | EPMS |
| `SK` | Sticky latcheado desde el boot (ID 17) | SKSD, SKTD, SKTF, SKIM, SKPF, SKBF |
| `CN` | Contador (IDs 16 y 17) | CNTV..CNTI, CNTD, CNSD, CNIM |

Si un short colisiona con otro de tu protocolo, renómbralo libremente en
RaceStudio tras el import — el firmware no se entera del short name, solo
de byte/bit, y eso lo fija el DBC.

---

## 19. Checks rápidos

Tras flashear y poner el coche en marcha (sin tracción):

1. **Frame 0x0F llega cada ~100-200 ms** → si no, revisar `setPacketTimer`.
2. **`BMS_Fault = 0` y `State_BmsInit = 1`** → BMS arrancó bien.
3. **`Mod_VTotal` ≈ tensión medida con multímetro entre terminales del módulo** → mapeo V_n↔paralelo bien (si tienes ese bloque forense activado).
4. **`Rst_IWDG = 0`** en el frame de los primeros segundos → si está a 1, el coche reseteó por watchdog (revisar).
5. **`BMS_AliveCnt` sube de forma continua** (no se congela) → el BMS no está colgado.

---

## 20. Inversor DTI HV-500 (CAN Manual v2.5)

El AiM es un **logger**: del inversor lee lo que **transmite** (tabla de
abajo). Los **comandos** los envía la VCU; loguearlos es opcional y no
están en el DBC.

> **Confirmado** (DTI CAN Tool + manual v2.5): Standard ID, Node ID = 1,
> numeración de packets V2.5. CAN ID = `(Packet << 5) | 1`. Transmite
> packets 0x1F-0x26 (→ 0x3E1-0x4C1). Big Endian, DLC 8 siempre (bytes no
> usados = 0xFF, igual que el BMS). Casi todo Signed, Gain = 1/Scale.
>
> ✅ **Verificado 2026-10-10** contra el manual DTI v2.5 (`pdftotext` sobre
> el PDF, Tablas 4-8) **y** los 4 ejemplos numéricos reales que trae el
> propio manual (ERPM=9310, Duty=11.3%, Vin=390V, AC=9.2A, DC=1.7A,
> CtrlTemp=33.9°C, MotorTemp=27.9°C, Iq=6.58A). El DBC decodifica estos 4
> ejemplos exacto (comprobado con `cantools`). Una versión anterior de esta
> tabla tenía el Start Bit de los campos multi-byte al revés (apuntaba al
> último byte del campo en vez de al primero); ya corregido en el DBC.

| ID | Packet | Canal | Byte | Descripción |
|---|---|---|---|---|
| 0x3E1 | 0x1F | INV_ControlMode | 0 | 1=SPEED 2=CURRENT 3=CURRENT_BRAKE 4=POS 7=NONE |
| 0x3E1 | 0x1F | INV_TargetIq    | 1-2 | objetivo de corriente Iq (excluye límites) |
| 0x3E1 | 0x1F | INV_MotorPos    | 3-4 | posición del motor (°) |
| 0x3E1 | 0x1F | INV_MotorStill  | 5 | 1=parado 0=girando |
| 0x401 | 0x20 | INV_ERPM        | 0-3 | RPM eléctrico (÷ pares de polos = RPM mecánico) |
| 0x401 | 0x20 | INV_Duty        | 4-5 | duty cycle del inversor (%, signo = motor/regen) |
| 0x401 | 0x20 | INV_VinDC       | 6-7 | tensión DC de entrada |
| 0x421 | 0x21 | INV_AC_Current  | 0-1 | corriente de fase (AC) |
| 0x421 | 0x21 | INV_DC_Current  | 2-3 | corriente de bus (DC, signo = motor/regen) |
| 0x441 | 0x22 | INV_CtrlTemp    | 0-1 | temperatura de los semiconductores |
| 0x441 | 0x22 | INV_MotorTemp   | 2-3 | temperatura del motor (sensor del inversor) |
| 0x441 | 0x22 | INV_FaultCode   | 4 | ver tabla de abajo |
| 0x461 | 0x23 | INV_Id          | 0-3 | componente Id del FOC (debug avanzado) |
| 0x461 | 0x23 | INV_Iq          | 4-7 | componente Iq del FOC (debug avanzado) |
| 0x481 | 0x24 | INV_Throttle    | 0 | señal de acelerador que ve el inversor |
| 0x481 | 0x24 | INV_Brake       | 1 | señal de freno que ve el inversor |
| 0x481 | 0x24 | INV_DriveEnable | 3, bit0 | 1=drive habilitado |
| 0x481 | 0x24 | INV_CANmapVer   | 7 | versión del mapa CAN (25 = v2.5) |
| 0x4A1 | 0x25 | INV_Max/MinAC_cfg/avail | 0-7 | límites de corriente AC configurados/disponibles |
| 0x4C1 | 0x26 | INV_Max/MinDC_cfg/avail | 0-7 | límites de corriente DC configurados/disponibles |

**`INV_FaultCode` (0x441 byte 4):** 0=OK · 1=Overvoltage · 2=Undervoltage ·
3=DRV · 4=Overcurrent · 5=Ctrl OverTemp · 6=Motor OverTemp · 7=Sensor wire ·
8=Sensor general · 9=CAN cmd error · 10=Analog input error.

**Bits del 0x481** no usados en el DBC por defecto (digital IO del
inversor, confirmado sin cablear en el coche — ver IO digital/analógica
real más abajo): `DigIn1-4`, `DigOut1-4` (byte 2), y los 11 bits de "límite
activo" (`CapTempLim`...`PowerLim`, bytes 4-5) que dicen *por qué* el
inversor está derateando. Si hiláis fino el derating, esos 11 bits son los
primeros a recuperar.

> **IO real usada:** 1 pin digital del inversor → señal de R2D (es
> `INV_DriveEnable`, no uno de los `DigIn1-4` genéricos); 1 entrada analógica
> → acelerador (es `INV_Throttle`). Por eso `DigIn1-4`/`DigOut1-4` están
> fuera del DBC: no hay nada cableado a esos pines.
>
> **`INV_MaxDC_avail`** = corriente DC máx disponible tras las limitaciones
> internas del inversor. Útil para cerrar el lazo del **derating** del BMS:
> `CMD_SetMaxDCCurr`/`CMD_SetMaxDCBrake` (comandos del VCU al inversor,
> packets 0x0A/0x0B) limitan descarga/carga; el BMS puede pedirlos según T/V
> de celda y leer aquí lo que el inversor realmente aplica.

---

## 21. VCU (VCU_26) — telemetría que transmite

Standard ID, Big Endian, 125 kbps. La VCU **solo transmite**, no recibe
nada (los comandos hacia el inversor del §20 los manda ella, pero no hay
bucle de vuelta).

> Fuente: repo `VCU_26`, `src/Control.cpp` → `publishTelemetry()`.

| ID | Canal | Byte | Descripción |
|---|---|---|---|
| 0x488 (1160) | VCU_FaultCause | 0 | causa de no-R2D, ver tabla |
| 0x488 | VCU_ResetCause | 1 | 0=desconocido 1=power/BOR 2=NRST 3=SW 4=IWDG 5=WWDG 6=lowpwr |
| 0x488 | VCU_Heartbeat  | 2-3 | cuenta de loop; **se congela si el firmware se cuelga** |
| 0x48B (1163) | APPS1_pct | 0-1 | sensor 1 del acelerador (%) |
| 0x48B | APPS2_pct | 2-3 | sensor 2 del acelerador (%) — se compara con el 1 para detectar implausibilidad |
| 0x48B | APPS1_raw | 4-5 | lectura cruda del ADC del sensor 1 |
| 0x48B | APPS2_raw | 6-7 | lectura cruda del ADC del sensor 2 |
| 0x48C (1164) | Brake1_pct | 0-1 | sensor 1 de freno (%, TODO: hoy siempre 0) |
| 0x48C | Brake2_pct | 2-3 | sensor 2 de freno (%, TODO: hoy siempre 0) |
| 0x48C | Brake1_raw | 4-5 | lectura cruda del ADC del sensor 1 |
| 0x48C | Brake2_raw | 6-7 | lectura cruda del ADC del sensor 2 |
| 0x48E (1166) | VCU_Vbat   | 0-1 | tensión de batería de baja (ADC crudo, TODO escalar a V) |
| 0x48E | VCU_SDC    | 2 | SDC presente (visto por la VCU) |
| 0x48E | VCU_Start  | 3 | pulsador Start |
| 0x48E | VCU_R2D    | 4 | Ready-to-Drive (lógica positiva: 1=activo) |
| 0x48E | VCU_AppsSt | 5 | 0=NORMAL 1=IMPL 2=PENDING |
| 0x48E | VCU_BrakeFlt | 6 | freno desconectado/en corto → tumba el R2D |
| 0x48E | VCU_Bypass   | 7 | ⚠ bypass de banco: R2D forzado sin enclavamientos |

**`VCU_FaultCause`** — el primero que aplica, en este orden:

| Valor | Significado |
|---|---|
| **7** | ⚠ **BYPASS DE BANCO ACTIVO** — la salida R2D está forzada, sin ningún enclavamiento |
| 0 | R2D activo / OK |
| 2 | SDC abierto |
| 6 | freno desconectado o en corto |
| 3 | APPS implausible |
| 1 | esperando start + freno |

> El **7 tiene prioridad sobre todos los demás**: si aparece, la VCU está en
> modo banco con el par habilitado saltándose SDC, secuencia de armado,
> freno e implausibilidad del APPS. Nunca debería verse en pista. Coincide
> con `VCU_Bypass = 1` del 0x48E — **merece alarma en el dashboard**. No
> persiste entre reinicios.
>
> `VCU_Heartbeat` es el canal más útil para vigilancia en pista: si deja de
> incrementar con la VCU alimentada, el loop está colgado. Combínalo con
> `VCU_ResetCause = 4` (IWDG) para detectar el ciclo cuelgue → reset.
>
> IDs VCU aún sin implementar en el firmware (no están en el DBC): 0x190,
> 0x489/0x48A (fail codes), 0x48D (steer/ruedas), 0x48F (PWM).

---

## 22. PDM (mart-PDM) — refrigeración

Standard ID, Big Endian, 125 kbps, nodo PDM (ID CAN 5). Controla la bomba
Pierburg CWA y los 2 ventiladores Delta según temperatura del inversor
(curva propia) o consigna de la VCU.

> Fuente: repo `mart-PDM`, `src/main.cpp` → `sendPdmStatus()`.

| ID | Canal | Byte | Descripción |
|---|---|---|---|
| 0x301 (769) | PDM_Mode | 0 | 0=AUTO (curva térmica) 1=CAN (consigna VCU) 2=SERIAL (override de banco) |
| 0x301 | PDM_PumpPct | 1 | % de duty de bomba aplicado |
| 0x301 | PDM_FanPct | 2 | % de duty de ventiladores aplicado |
| 0x301 | PDM_Failsafe | 3, bit0 | 1 = sin temperaturas del inversor → demanda fija de emergencia |
| 0x301 | PDM_SafetyFloor | 3, bit1 | 1 = cerca del límite térmico, la curva subió la consigna por encima de lo pedido |

> El PDM lee la temperatura del inversor directamente del CAN del propio
> inversor (0x441, §20) y la consigna de refrigeración de la VCU en 0x300
> (no transmitida por el PDM, por eso no está en esta lista). `PDM_Mode`
> dice quién manda ahora mismo sobre bomba/ventiladores — imprescindible
> para no confundir un override de banco con la curva automática al mirar
> el log después.

---

## 23. Historial de cambios

- **2026-05-20** — generado a partir de `src/main.cpp`.
- **2026-05-28** — poda de `PFAI`, tabla de Start Bit (rama `testing`).
- **2026-05-30 / 05-31** — primera versión de §20 (inversor DTI) y §21 (VCU,
  generada entonces desde `mart-cockpit`, firmware equivocado).
- **2026-08-10** — §21 (VCU) rehecha contra el repo real `VCU_26`: 0x488
  corregido, + `VCU_BrakeFlt`.
- **2026-10-10** — reescritura grande:
  - Baja del ID 10 (`BMS_Status`): duplicaba 6 de 8 bits del ID 15.
  - §20 (inversor): recontrastado contra el manual DTI v2.5 con los 4
    ejemplos numéricos reales — varios Start Bit multi-byte estaban al
    revés (apuntaban al último byte, no al primero). Corregido en el DBC.
  - Añadido §22 (PDM, antes inexistente en este doc).
  - Creado `docs/BMS_MART.dbc` como fuente de verdad única para bits/
    escalas/signos de los 4 nodos (BMS, inversor, VCU, PDM), validado con
    `cantools`. Este documento se reescribió para dejar de duplicar esa
    información y centrarse en explicar qué es cada canal.
- **2026-10-15** — nuevo ID 18 (0x12): corriente del Hall y tensión del pack
  medidas por el BMS, para tener la potencia de batería independiente del
  inversor (§10b).

Si cambia alguna trama, actualiza primero `docs/BMS_MART.dbc` (y
valídalo con `cantools`) y luego este doc — nunca al revés.
