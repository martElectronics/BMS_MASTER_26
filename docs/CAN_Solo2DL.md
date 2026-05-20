# BMS → AiM Solo 2 DL — Mapa CAN completo

**Fuente:** firmware BMS Master STM32G474RE · MART Formula Student
**Receptor:** AiM Solo 2 DL (RaceStudio 3 → Custom CAN protocol)

---

## 1. Parámetros del bus

| | Valor |
|---|---|
| Bitrate | **500 kbps** |
| Formato ID | **Standard 11-bit** |
| Byte order (multi-byte) | **Big-Endian (Motorola)** |
| Sample point típico | 75-87.5 % (deja que RaceStudio elija) |
| Terminación | 120 Ω en cada extremo del bus |

> Todas las tramas que llevan campos `uint16_t` se transmiten en BE porque la
> librería MART_CAN convierte por defecto. Las tramas de bytes sueltos
> (ID 15, 390, 391, 392) NO necesitan conversión — los bytes van en orden.

---

## 2. Setup en RaceStudio 3 (paso a paso)

1. **USB / WiFi → Configurations → New → Solo 2 DL**.
2. En la pestaña **ECU Stream / CAN2 → Custom protocol → New**.
3. Rellenar:
   - **Name:** `MART_BMS_500k`
   - **Bitrate:** 500 kbps
   - **ID type:** Standard
   - **Byte order:** Big Endian (Motorola)
4. **Add channel** por cada fila de las tablas siguientes.
   - Para bits (bool, 1 bit): pon **Length = 1**, **Type = Bit**.
   - Para campos numéricos: **Length** en bits, **Type = Unsigned/Signed**, escala y unidad de la tabla.
5. **Transmit configuration → Solo 2 DL** y arrancar logging.

> En el panel del piloto agrupa los canales por color: rojo = `bmsFault` y
> sub-fallos; amarillo = snapshot actual; gris = state. Así de un vistazo
> en directo identificas la causa.

---

## 3. Resumen de IDs

| ID dec | ID hex | DLC | Periodo | Contenido |
|---|---|---|---|---|
| 10 | 0x0A | 1  | 800 ms | Estado general (8 flags) |
| 11 | 0x0B | 8  | 799 ms | MaxT, MaxV, MinV, MinT |
| 12 | 0x0C | 4  | 799 ms | Status V / Status T (provisional) |
| 13 | 0x0D | 4  | 798 ms | maxFailTime, numTriesReset |
| 14 | 0x0E | 8  | 200 ms | lastFailTime, contadores comm/CRC/reset |
| **15** | **0x0F** | **8** | **200 ms** | **BMS_DEBUG — granularidad por bit** |
| 386 | 0x182 | 8 | 557 ms paginado | IDmod, V1, V2, V3 |
| 387 | 0x183 | 8 | 556 ms paginado | IDmod, V4, V5, V6 |
| 388 | 0x184 | 8 | 556 ms paginado | IDmod, V7, V8, V9 |
| 389 | 0x185 | 8 | 555 ms paginado | IDmod, V10, V11, VTotal |
| 390 | 0x186 | 8 | 554 ms paginado | IDmod, T1..T7 |
| 391 | 0x187 | 8 | 554 ms paginado | IDmod, T8, T9, Tmax, Tmin, stV, stT |
| 392 | 0x188 | 1 | 553 ms | SOC (%) |

**Paginado (386-391):** un módulo distinto por ronda (~556 ms entre módulos
del mismo ID). Con 12 módulos, cada módulo se actualiza cada ~6.7 s.
El campo `IDmod` (byte 0-1) te dice de cuál es la trama.

---

## 4. ID 10 (0x0A) — Estado general · 1 byte · 800 ms

```
Byte 0:  [b7][b6][b5][b4][b3][b2][b1][b0]
          AUTO AMP VOLT COMM cond res res Sts
          ADDR             Now              Fail
```

| Canal | Short | Byte | Bit | Len | Tipo | Descripción |
|---|---|---|---|---|---|---|
| BMS_StsFail    | SFAI | 0 | 0 | 1 | bool | `bmsFault` global (OR confirmado) |
| BMS_CondNow    | FCON | 0 | 3 | 1 | bool | Alguna condición SAMPLE activa (sin debounce) |
| BMS_CommErr    | ECOM | 0 | 4 | 1 | bool | `fComm.confirmed` (≥500 ms) |
| BMS_VoltErr    | EVOL | 0 | 5 | 1 | bool | `fV.confirmed` (≥500 ms) |
| BMS_AmpErr     | EAMP | 0 | 6 | 1 | bool | `!hall.isOK()` |
| BMS_AutoAddrOk | EAUT | 0 | 7 | 1 | bool | AutoAddress completado |

> Los bits 1 y 2 están reservados (siempre 0). Sin bit propio para `fT`,
> `fNtc`, `!bmsInitOk`, `prechargeFailed` — usar **ID 15** para esos.
>
> Nota nomenclatura: he renombrado los antiguos `*ErrorLatch` por `*Err`
> porque el firmware nuevo es **no-latching** (el latch es HW). Mantén los
> short names ECOL/EVOL si prefieres compatibilidad con tu config previa.

---

## 5. ID 11 (0x0B) — Métricas V/T · 8 bytes · 799 ms

```
Bytes 0-1: MaxT (int16 BE, °C)
Bytes 2-3: MaxV (uint16 BE, mV)
Bytes 4-5: MinV (uint16 BE, mV)
Bytes 6-7: MinT (int16 BE, °C)
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| BMS_MaxT | Tmax | 0 | 0 | 16 | int16 BE  | 1     | °C |
| BMS_MaxV | Vmax | 2 | 0 | 16 | uint16 BE | 0.001 | V  |
| BMS_MinV | Vmin | 4 | 0 | 16 | uint16 BE | 0.001 | V  |
| BMS_MinT | Tmin | 6 | 0 | 16 | int16 BE  | 1     | °C |

> Las temperaturas son **signed**. Pon "Signed" en RaceStudio o leerás −20 °C
> como 65516. Los voltajes están en mV: usa multiplicador 0.001 para verlo en V.
> Los short names coinciden con los que ya tenías en `01_MART`.

---

## 6. ID 12 (0x0C) — Status V/T (provisional) · 4 bytes · 799 ms

```
Bytes 0-1: gsV (uint16 BE)  0=OK 1=UV 2=OV
Bytes 2-3: gsT (uint16 BE)  0=OK 1=UT 2=OT 3=NTC_open
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| BMS_StatusV | GSV_ | 0 | 0 | 16 | uint16 BE | 1 | — |
| BMS_StatusT | GST_ | 2 | 0 | 16 | uint16 BE | 1 | — |

> ⚠ Encoding **PROVISIONAL** (a confirmar con el equipo). Si bajáis los
> valores a 8-bit ahorraría medio frame (sin impacto).

---

## 7. ID 13 (0x0D) — Estadística acumulada · 4 bytes · 798 ms

```
Bytes 0-1: maxFailMs (uint16 BE, ms)        — duración máx vista desde boot
Bytes 2-3: numTriesReset (uint16 BE)        — reintentos de reInit BQ
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| BMS_MaxFailMs   | MFLM | 0 | 0 | 16 | uint16 BE | 1 | ms |
| BMS_NumTryReset | NTR_ | 2 | 0 | 16 | uint16 BE | 1 | —  |

---

## 8. ID 14 (0x0E) — Último episodio · 8 bytes · 200 ms

```
Bytes 0-1: lastFailMs (uint16 BE)           — duración del episodio actual/último
Bytes 2-3: numCommFails (uint16 BE)
Bytes 4-5: numCrcFails (uint16 BE)
Bytes 6-7: numTriesReset (uint16 BE)        — idéntico al de ID 13
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| BMS_LastFailMs  | LTFT | 0 | 0 | 16 | uint16 BE | 1 | ms |
| BMS_NumCommFail | NCFA | 2 | 0 | 16 | uint16 BE | 1 | —  |
| BMS_NumCrcFail  | NCRC | 4 | 0 | 16 | uint16 BE | 1 | —  |
| BMS_NumTryRst14 | NTRR | 6 | 0 | 16 | uint16 BE | 1 | —  |

> Los 3 primeros short names (LTFT/NCFA/NCRC) coinciden con tu config previa.
> El 4º (NTRR) es nuevo. Si te pide nombre único, usa `NTRR` en ID 14 y
> `NTR_` en ID 13 — son el mismo contador pero distintos slots.

---

## 9. ID 15 (0x0F) — BMS_DEBUG · 8 bytes · 200 ms

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

| Canal | Short | Byte | Bit | Len | Tipo | Descripción |
|---|---|---|---|---|---|---|
| BMS_Flt_V       | FV__ | 0 | 0 | 1  | bool      | UV/OV confirmado (≥500 ms) |
| BMS_Flt_T       | FT__ | 0 | 1 | 1  | bool      | UT/OT confirmado (≥1000 ms) |
| BMS_Flt_NTC     | FNTC | 0 | 2 | 1  | bool      | NTC abierto confirmado |
| BMS_Flt_Comm    | FCOM | 0 | 3 | 1  | bool      | Comms BQ caídas confirmadas |
| BMS_Flt_Hall    | FHAL | 0 | 4 | 1  | bool      | HallSensor !OK |
| BMS_Flt_NoInit  | FINI | 0 | 5 | 1  | bool      | `bmsInitOk = false` |
| BMS_Fault       | BFLT | 0 | 7 | 1  | bool      | **OR global → BMS_OK LOW** |
| Hall_Disc       | HDIS | 1 | 0 | 1  | bool      | DHAB desconectado |
| Hall_Stuck      | HSTK | 1 | 1 | 1  | bool      | Sensor congelado |
| Hall_Noisy      | HNOI | 1 | 2 | 1  | bool      | EMI / ΔI extremo |
| Hall_OverI      | HOVI | 1 | 3 | 1  | bool      | Sobre-I FS (>170 A desc / <−7 A carga) |
| Hall_AdcSat     | HSAT | 1 | 4 | 1  | bool      | ADC saturado |
| V_UV_now        | NUV_ | 2 | 0 | 1  | bool      | minV < 2.80 V (snapshot) |
| V_OV_now        | NOV_ | 2 | 1 | 1  | bool      | maxV > 4.20 V (snapshot) |
| T_UT_now        | NUT_ | 2 | 2 | 1  | bool      | minT < −20 °C (snapshot) |
| T_OT_now        | NOT_ | 2 | 3 | 1  | bool      | maxT > 60 °C (snapshot) |
| NTC_open_now    | NNTC | 2 | 4 | 1  | bool      | `hasOpenNtc` (snapshot) |
| ReadV_CommErr   | RVCO | 2 | 5 | 1  | bool      | última lectura V = COMM_ERROR |
| ReadV_CrcErr    | RVCR | 2 | 6 | 1  | bool      | última lectura V = CRC_ERROR |
| ReadT_Err       | RTER | 2 | 7 | 1  | bool      | última lectura T ≠ OK |
| State_AutoAddr  | SADR | 3 | 0 | 1  | bool      | `bms.isOK` |
| State_BmsInit   | SINI | 3 | 1 | 1  | bool      | `bmsInitOk` |
| State_CanOk     | SCAN | 3 | 2 | 1  | bool      | FDCAN inicializó OK |
| Pre_Started     | PSTR | 3 | 3 | 1  | bool      | Precarga iniciada |
| Pre_Ok          | POK_ | 3 | 4 | 1  | bool      | Precarga completada |
| Pre_Failed      | PFAI | 3 | 5 | 1  | bool      | Timeout 5 s de precarga |
| Pin_SDC_3V3     | PSDC | 3 | 6 | 1  | bool      | SDC alimentado |
| Pin_VIO_3V3     | PVIO | 3 | 7 | 1  | bool      | VIO_3V3 presente |
| FirstFault      | FFLT | 4 | 0 | 8  | uint8     | **Enum**: 0=none 1=V 2=T 3=NTC 4=Comm 5=Hall 6=!Init |
| EpisodeMs       | EPMS | 5 | 0 | 16 | uint16 BE | Duración episodio actual (ms, clamp 65535) |
| Rst_LPWR        | RLPW | 7 | 0 | 1  | bool      | Reset por low-power |
| Rst_WWDG        | RWWD | 7 | 1 | 1  | bool      | Reset por window-WDG |
| Rst_IWDG        | RIWD | 7 | 2 | 1  | bool      | **Reset por IWDG (loop colgado)** |
| Rst_SOFT        | RSFT | 7 | 3 | 1  | bool      | Reset software (cmd `r`) |
| Rst_BOR         | RBOR | 7 | 4 | 1  | bool      | Power-on / brown-out |
| Rst_PIN         | RPIN | 7 | 5 | 1  | bool      | Reset por pin NRST |
| Rst_OBL         | ROBL | 7 | 7 | 1  | bool      | Option-byte loader |

### Recetas de debug (post-mortem)

| Síntoma | Lectura del frame | Causa raíz probable |
|---|---|---|
| `BMS_Fault=1` y `BMS_Flt_Comm=1`, `FirstFault=4` | Comm cayó primero, el resto cascadeó | Cableado del transceiver UART al BQ, EMI |
| `BMS_Flt_Hall=1`, `Hall_Disc=1` | Hall desconectado | Cable del DHAB suelto |
| `BMS_Flt_Hall=1`, `Hall_Noisy=1` | EMI / ΔI extremo | Filtrado de inversor, malla a tierra |
| `BMS_Flt_NoInit=1`, `State_BmsInit=0` | Init falló y aún reintentando | Bus al BQ apagado / `VIO_3V3` ausente |
| `V_UV_now=1` pero `BMS_Flt_V=0` | Glitch UV filtrado por debounce | Ruido en lectura, NO es UV real |
| `Rst_IWDG=1` al boot | Loop colgado | Punto donde se quedó: revisar logs serie |
| `Pre_Failed=1` | Timeout 5 s precarga | Resistencia de precarga / contactores |

---

## 10. ID 386 (0x182) — Tensiones V1-V3 · 8 bytes · paginado 557 ms

```
Bytes 0-1: IDmod  (uint16 BE)               — índice del módulo (0..N-1)
Bytes 2-3: V1     (uint16 BE, mV)           — paralelo 1 = par[0]
Bytes 4-5: V2     (uint16 BE, mV)
Bytes 6-7: V3     (uint16 BE, mV)
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| Mod_IDmod_386 | IM86 | 0 | 0 | 16 | uint16 BE | 1     | — |
| Mod_V1        | MV01 | 2 | 0 | 16 | uint16 BE | 0.001 | V |
| Mod_V2        | MV02 | 4 | 0 | 16 | uint16 BE | 0.001 | V |
| Mod_V3        | MV03 | 6 | 0 | 16 | uint16 BE | 0.001 | V |

> Los 4 IDs 386-389 comparten el mismo `IDmod` en byte 0-1. RaceStudio
> exige nombres únicos: usa `IM86/IM87/IM88/IM89` (uno por ID).

---

## 11. ID 387 (0x183) — Tensiones V4-V6 · 8 bytes · paginado 556 ms

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| Mod_IDmod_387 | IM87 | 0 | 0 | 16 | uint16 BE | 1     | — |
| Mod_V4        | MV04 | 2 | 0 | 16 | uint16 BE | 0.001 | V |
| Mod_V5        | MV05 | 4 | 0 | 16 | uint16 BE | 0.001 | V |
| Mod_V6        | MV06 | 6 | 0 | 16 | uint16 BE | 0.001 | V |

---

## 12. ID 388 (0x184) — Tensiones V7-V9 · 8 bytes · paginado 556 ms

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| Mod_IDmod_388 | IM88 | 0 | 0 | 16 | uint16 BE | 1     | — |
| Mod_V7        | MV07 | 2 | 0 | 16 | uint16 BE | 0.001 | V |
| Mod_V8        | MV08 | 4 | 0 | 16 | uint16 BE | 0.001 | V |
| Mod_V9        | MV09 | 6 | 0 | 16 | uint16 BE | 0.001 | V |

---

## 13. ID 389 (0x185) — Tensiones V10-V11 + VTotal · 8 bytes · paginado 555 ms

```
Bytes 0-1: IDmod
Bytes 2-3: V10   (uint16 BE, mV)
Bytes 4-5: V11   (uint16 BE, mV)
Bytes 6-7: VTot  (uint16 BE, mV) — suma de los 11 paralelos del módulo
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| Mod_IDmod_389 | IM89 | 0 | 0 | 16 | uint16 BE | 1     | — |
| Mod_V10       | MV10 | 2 | 0 | 16 | uint16 BE | 0.001 | V |
| Mod_V11       | MV11 | 4 | 0 | 16 | uint16 BE | 0.001 | V |
| Mod_VTotal    | MVTT | 6 | 0 | 16 | uint16 BE | 0.001 | V |

> `Mod_VTotal` debe coincidir con la tensión medida entre terminales del
> módulo (margen ±20 mV por offset del ADC del BQ × 11 celdas).

---

## 14. ID 390 (0x186) — Temperaturas T1-T7 · 8 bytes · paginado 554 ms

```
Byte 0: IDmod (uint8)
Byte 1: T1   (int8, °C)
Byte 2: T2   (int8, °C)
Byte 3: T3   (int8, °C)
Byte 4: T4   (int8, °C)
Byte 5: T5   (int8, °C)
Byte 6: T6   (int8, °C)
Byte 7: T7   (int8, °C)
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| Mod_IDmod_390 | IM90 | 0 | 0 | 8 | uint8    | 1 | —  |
| Mod_T1        | MT01 | 1 | 0 | 8 | **int8** | 1 | °C |
| Mod_T2        | MT02 | 2 | 0 | 8 | **int8** | 1 | °C |
| Mod_T3        | MT03 | 3 | 0 | 8 | **int8** | 1 | °C |
| Mod_T4        | MT04 | 4 | 0 | 8 | **int8** | 1 | °C |
| Mod_T5        | MT05 | 5 | 0 | 8 | **int8** | 1 | °C |
| Mod_T6        | MT06 | 6 | 0 | 8 | **int8** | 1 | °C |
| Mod_T7        | MT07 | 7 | 0 | 8 | **int8** | 1 | °C |

> Importante: las temperaturas son **signed int8** (rango −128..+127 °C).
> En RaceStudio marca "Signed" o leerás −20 °C como 236.
> Nota: el módulo solo tiene 9 NTCs (par 6 + impar 3) → T1..T9 son los
> reales; el resto siempre será inválido si se intenta interpretar.

---

## 15. ID 391 (0x187) — Temperaturas T8-T9 + Tmax/min + Status · 8 bytes · paginado 554 ms

```
Byte 0: IDmod
Byte 1: T8  (int8 °C)
Byte 2: T9  (int8 °C)
Byte 3: Tmax (int8 °C)         — máx de T1..T9 del módulo
Byte 4: Tmin (int8 °C)         — mín de T1..T9 del módulo
Byte 5: stV (uint8)             — 0=OK 1=UV 2=OV  (módulo)
Byte 6: stT (uint8)             — 0=OK 1=UT 2=OT  (módulo)
Byte 7: reservado (0)
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| Mod_IDmod_391 | IM91 | 0 | 0 | 8 | uint8    | 1 | —  |
| Mod_T8        | MT08 | 1 | 0 | 8 | **int8** | 1 | °C |
| Mod_T9        | MT09 | 2 | 0 | 8 | **int8** | 1 | °C |
| Mod_Tmax      | MTMX | 3 | 0 | 8 | **int8** | 1 | °C |
| Mod_Tmin      | MTMN | 4 | 0 | 8 | **int8** | 1 | °C |
| Mod_StatusV   | MSV_ | 5 | 0 | 8 | uint8    | 1 | —  |
| Mod_StatusT   | MST_ | 6 | 0 | 8 | uint8    | 1 | —  |

---

## 16. ID 392 (0x188) — SOC · 1 byte · 553 ms

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| BMS_SOC | SOC_ | 0 | 0 | 8 | uint8 | 1 | % |

> SOC **orientativo** hasta caracterizar la curva OCV-SOC real de las
> Samsung INR21700-40T (ver `ARQUITECTURA.md §9.6`). El coulomb counting
> con `Np=11` (44 Ah pack) ya está bien.

---

## 17. Convención de naming

Dos sistemas en paralelo: **nombre largo** (descriptivo, hasta 16 chars) y
**short name** (4 chars, el que se ve en gauges/displays de AiM).

### Nombre largo (grupos por prefijo)

- `BMS_*` → telemetría general (IDs 10-14, 392).
- `Hall_*` → sub-fallos del amperímetro (ID 15 B1).
- `V_*` / `T_*` / `NTC_*` → snapshots (ID 15 B2).
- `State_*` / `Pre_*` / `Pin_*` → state del sistema (ID 15 B3).
- `Rst_*` → causa del último reset (ID 15 B7).
- `Mod_*` → detalle por módulo (IDs 386-391).

### Short name (4 chars, AiM RaceStudio)

Las letras de la 1ª columna dicen "categoría":

| Prefijo | Categoría | Ejemplos |
|---|---|---|
| `F` | Fault confirmado (driver de BMS_OK) | FV, FT, FNTC, FCOM, FHAL, FINI |
| `B` | BMS global / boolean | BFLT (BMS_Fault) |
| `E` | Error específico | ECOM, EVOL, EAMP, EAUT |
| `H` | Hall sub-fault | HDIS, HSTK, HNOI, HOVI, HSAT |
| `N` | Snapshot "ahora" (sin debounce) | NUV, NOV, NUT, NOT, NNTC |
| `R` | Reset / Read-error | RIWD, RBOR, RVCO, RVCR, RTER |
| `S` | State / status | SADR, SINI, SCAN, SOC |
| `P` | Precarga / Pin físico | PSTR, POK, PFAI, PSDC, PVIO |
| `M` | Módulo (paginados 386-391) | MV01..MV11, MVTT, MT01..MT09, MTMX/MTMN |
| `IM` | IDmod (uno por ID paginado) | IM86..IM91 |
| `FF` | First Fault (enum) | FFLT |
| `EP` | Episodio | EPMS |

Los short names ya existentes en tu protocolo `01_MART` (Tmax/Vmax/Vmin/Tmin
del ID 0xB, LTFT/NCFA/NCRC del ID 0xE, SOC del ID 0x188) se mantienen tal
cual. Los nuevos (ID 15 entero, los paginados 386-391, los faltantes de
0xA/0xC/0xD) siguen este patrón.

Si un short colisiona con otro de tu protocolo (improbable porque he
evitado los códigos del inverter/dashboard tipo APP1, BR1, etc.),
renómbralo libremente — el firmware no se entera del short name, solo
de byte/bit.

---

## 18. Checks rápidos

Tras flashear y poner el coche en marcha (sin tracción):

1. **Frame 0x0F llega cada ~200 ms** → si no, revisar setPacketTimer.
2. **`BMS_Fault = 0` y `State_BmsInit = 1`** → BMS arrancó bien.
3. **`Mod_VTotal` ≈ tensión medida con multímetro entre terminales del módulo** → mapeo Vn↔paralelo bien.
4. **`Rst_IWDG = 0` en el frame de los primeros segundos** → si está a 1, el coche reseteó por watchdog (revisar)..
5. **Toda la suma `Mod_V1..V11` ≈ `Mod_VTotal`** → consistencia interna.

---

## 19. Cambios pendientes en este doc

- Encoding oficial de `BMS_StatusV` / `BMS_StatusT` (ID 12) — pendiente de
  decisión con el equipo.
- Confirmar el orden `Mod_V1..V11` contra el cableado físico (ya
  validado lógicamente: par 6 + impar 5 con dummy arriba).
- Tras caracterizar la 40T, actualizar la nota del SOC en §16.

---

*Generado a partir de `src/main.cpp` (rev. 2026-05-20). Si cambia alguna
trama, regenerar este doc antes de subir el coche a pista.*
