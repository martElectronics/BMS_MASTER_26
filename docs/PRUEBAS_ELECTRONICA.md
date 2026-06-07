# Plan de pruebas — Electrónica del coche (FS 2026)

Checklist de puesta en marcha siguiendo el **orden de trabajo real (15 pasos)**, con
**gates de seguridad** en los saltos LV→HV. Cubre acumulador, cargador, BMS, VCU e
inversor DTI HV-500.

> **Regla de oro:** no se pasa al paso siguiente hasta que el actual está en **PASA**.
> Si algo falla, se para y se anota. Prioridad absoluta: **cazar los 2 fallos de 2025**
> (ver sección siguiente).

---

## Registro de la sesión

| Campo | Valor |
|---|---|
| Fecha | ____________ |
| FW BMS (commit) | ____________ |
| FW VCU (commit) | ____________ |
| Config pack | ☐ 10 módulos (testing)  ☐ 12 módulos (final) |
| Responsable HV | ____________ |
| Operador 2 | ____________ |
| Resultado global | ☐ PASA  ☐ PASA con notas  ☐ FALLA |

**Convención por prueba:** Objetivo · Procedimiento · Resultado esperado · **☐ PASA / ☐ FALLA** · Notas.

---

## Modos de fallo a cazar (temporada 2025) — **PRIORIDAD**

### FM-1 — La precarga resetea el micro del BMS (→ BMS_OK LOW → SDC abre)
Se persigue en el **Paso 13 (mover motor con TSAC)**, donde ocurre la precarga real.

**Hipótesis (de más a menos probable):**
1. **Brownout del raíl 3.3 V** por el inrush / bobina de los contactores → reset.
2. **EMI / back-EMF** del contactor acoplado en la línea de reset o en `BMS_OK`.
3. **Ground bounce** por masa compartida con el inrush.
4. **IWDG** disparado por un bloqueo en la secuencia.

**Diagnóstico:**
- [ ] Loggear **`resetCause`** en cada precarga (serie + FRAM): ¿POR brownout? ¿IWDG? ¿pin?
- [ ] **Osciloscopio en el 3.3 V** del micro durante la precarga (buscar caída < umbral BOR).
- [ ] Osciloscopio en **`BMS_OK`** y **reset** al cerrar el contactor (glitch/spike).
- [ ] Verificar **flyback/snubber** en bobinas de contactor y precarga.
- [ ] Probar **alimentación del micro separada/filtrada** → ¿desaparece?
- **PASA:** 20 ciclos de precarga seguidos **sin un reset** y `BMS_OK` estable en HIGH.

### FM-2 — El coche se para a los 100–200 m (causa desconocida)
Se persigue en el **Paso 15 (coche en el suelo)**. Este año **sí** hay datos para cazarlo.

**Fuentes de verdad (post-mortem) — loggear TODO al Solo 2 DL:**
| Dato | Dónde | Qué dice |
|---|---|---|
| `faultCause` | VCU `0x488` b0 | 0=OK · 1=sin R2D · 2=SDC abierto · 3=APPS implausible · 4=fallo inversor · 5=comms BMS |
| `heartbeat` | VCU `0x488` b6-7 | Si se **congela** → el loop de la VCU murió |
| `resetCause` | VCU `0x488` b3 | Tipo de reset si lo hubo |
| Contador de fallos | BMS (CAN) | Qué fallo del BMS subió antes |
| FRAM | BMS | Log que **sobrevive** a un corte |
| Fault inversor + límites | `0x441` b4 / `0x481` | Fallo DTI / qué límite saltó |

**Sospechas:** glitch `BMS_OK`/IMD/TSON · watchdog CAN intermitente · APPS implausible transitorio · sag de tensión / corte subtensión · térmico.
- **PASA:** parada **diagnosticada con dato concreto** y corregida.

---

## PASO 0 — Seguridad y preparación (obligatorio)

- [ ] Mínimo **2 personas**; **responsable HV** designado. Nadie manipula HV solo.
- [ ] **Extintor** adecuado (CO₂ / Li-ion) accesible. EPI: guantes HV, gafas.
- [ ] **Multímetro CAT III** verificado antes de medir HV.
- [ ] Fuente LV de banco con límite de corriente. Analizador CAN / Solo 2 DL.
- [ ] **Parada de emergencia / desconexión rápida** identificada y probada.
- [ ] Esta hoja + `Config.h` (BMS y VCU) a mano para anotar calibraciones.

**GATE 0 → no continuar sin todo lo anterior.**

---

# BLOQUE A — Acumulador y carga (pasos 1–4)

## Paso 1 — Probar comunicación con el cargador
- [ ] **1.1** Flashear `pio run -e charger_cantest -t upload`, abrir monitor. *Esperado:* `COMUNICA OK`, frames `0x18FF50E5`, `COMM_TO=0` (el OBC nos recibe). Bus **500k**. ☐ PASA ☐ FALLA
- [ ] **1.2** Anotar status del OBC (`BAT_CONN`, etc.) y `Vout`/`Iout` en reposo. ☐ PASA ☐ FALLA

## Paso 2 — Scruti ACUMULADOR (FSG25 Part IV, ítems 24–131)
> Límite 105 min. Todo trabajo sobre el acumulador lo aprueba un inspector. `#`=check, `∆`=responsabilidad del equipo, `⊙`=opcional si ya pasado en otra inspección.

### 2.1 Recursos y EPI (24–37)
- [ ] **24** ESO presente · todos los TSAC del evento. **25** Sin TSAC de repuesto. ☐
- [ ] **26–28** Herramientas aisladas: cizalla, destornillador, llaves (n/a si no hay tornillos en TS). ☐
- [ ] **29** Multímetro con puntas protegidas. **30** 2 leads banana 4 mm (600 V CAT III). ☐
- [ ] **31–34** Pantalla facial · ≥3 gafas · ≥2 pares guantes HV · 2 mantas aislantes ≥1 m² con etiqueta+datasheet. ☐
- [ ] **35–37** PCBs propias: repuesto montado · spacing según tensión · aislamiento/coating y proceso según datasheet. ☐

### 2.2 Hand cart (38–44)
- [ ] **38** 4 ruedas, máx 1200×800 mm. **39** Freno "always-on". **40** Se mueve si se suelta el freno. ☐
- [ ] **41** TSAC fijado mecánicamente al carro. **42** Protegido de vibraciones/golpes. ☐
- [ ] **43** Firewall protege al operador. **44** Etiqueta EV5.3.8, máx 1,3 m del suelo. ☐

### 2.3 Etiquetado del contenedor (45–47)
- [ ] **45** Nº vehículo, universidad y tel. ESO sobre fondo de contraste. **46** Sans-serif ≥20 mm. ☐
- [ ] **47** Pegatinas ≥100 mm "Always Energized" + "High Voltage" (rayo negro sobre amarillo). ☐

### 2.4 Conjunto del cargador (48–56)
- [ ] **48** Cerrado (no se alcanza HV con sonda 100×6 mm). **49** Interlock. **50** TSMP integrado. ☐
- [ ] **51** Botón de parada de emergencia. **52** ≥24 mm diámetro. ☐
- [ ] **53** TSAL verde (electrónica hard-wired). **54** Cable TS naranja, marcado (gauge, >85 °C, V). ☐
- [ ] **55** Partes conductoras a tierra de protección (PE) al cargar (EV3.1). **56** Switches/plugs/indicadores etiquetados. ☐

### 2.5 Circuito de descarga y BPR (57–58)
- [ ] **57** Resistencia TS+/TS− = 30 kΩ (2×BPR) + resistor de descarga. **58** BPR potencia >6,1 W. ☐

### 2.6 Contenedor abierto y assembly (59–76)
- [ ] **59** Todo fijado. **60** TS aislado de pared si conductora. **61** ≥30 % de celdas con sensor de temp. ☐
- [ ] **62** Cada sensor en terminal negativo o <10 mm en busbar. **63** Sin celdas dañables por la estructura. ☐
- [ ] **64** Sin soldadura en camino de alta corriente. **65** ≥1 fusible adecuado por contenedor. ☐
- [ ] **66** ≥2 relés de aislamiento (AIR) adecuados (I y V). **67** AIR y fusibles separados de celdas (UL94-V0). ☐
- [ ] **68** **Relé de precarga de tipo mecánico** con rating de tensión adecuado. ☐
- [ ] **69** Maintenance plugs en ambos polos de cada stack. **70** Sin herramientas. **71** Bloqueo positivo. ☐
- [ ] **72** No pueden crear cortos por error. **73** Stacks separados ≤120 V DC. **74** ≤6 MJ. **75** Barrera ignífuga UL94-V0. **76** Si cerrado, válvula de ecualización. ☐

### 2.7 Cableado (77–85)
- [ ] **77** Protección de sobrecorriente en TS. **78** Solo TS es naranja. **79** Anclado ≥200 N si fuera de caja. ☐
- [ ] **80** Fuera de zonas de roce. **81** TS y LV separados (no interlock). **82** Todo cable ≥604,8 V (½ V nominal o 460 V). ☐
- [ ] **83** Demostrable gauge/temp/V de los TS. **84** Bloqueo positivo o componentes automotive. **85** Aislamiento no es solo cinta/pintura. ☐

### 2.8 CTMD (86–87)
- [ ] **86** Sensor CTMD en tab negativo (según ESF). **87** Refrigeración no superior en esa posición. (Foto al server.) ☐

### 2.9 Voltage Indicator y TSAL verde (88–97)
- [ ] **88–91** Indicador rojo/voltímetro instalado, marcado "Voltage Indicator", visible al abrir conector, hard-wired alimentado por TS. ☐
- [ ] **92–93** Al activar LVS: TSAL verde ON, visible. **94–96** Con 60 V DC: indicador ON, brillo constante, visible a pleno sol. **97** TSAL verde OFF. ☐

### 2.10 AMS (98–104) — *enlaza con el código del BMS (Paso 7.A)*
- [ ] **98** Desconectar sensor de corriente → AMS abre SDC en **<0,5 s**. ☐
- [ ] **99** Desconectar otro conector interno → SDC en **<1 s**. ☐
- [ ] **100–102** Con laptop: ver tensiones, temperaturas y **corriente plausible**. ☐
- [ ] **103** Desconectar 1 hilo de sense de **tensión** → SDC en **<0,5 s**. **104** 1 de **temperatura** → **<1 s**. ☐

### 2.11–2.14 Cierre, aislamiento, charger SDC, IMD (105–122)
- [ ] **105** Sellar contenedor. ☐
- [ ] **106–108** Aislamiento (test 500 V): Riso+ y Riso− **≫315 kΩ** y casi iguales. ☐
- [ ] **109** IMD integrado en la carga. **110** Indicador muestra HV. **111** Botón parada → AIR abren. **112** V<60 V. ☐
- [ ] **113** Desenchufar TSAC → AIR abren. **114** Cargador deshabilitado, sin V en conector. **115** Al reconectar, sigue off. ☐
- [ ] **116** Líneas de tierra IMD: una al contenedor y otra al chasis del cargador (cableado separado). ☐
- [ ] **117** R_Test entre TS+ y GND → SDC abre en **<30 s**. **118** V<60 V en 5 s. **119–121** No reactivable (ni con reset, ni tras 40 s). **122** R_Test en TS− → SDC abre <30 s. ☐

### 2.15 ASES contenedor (123–131)
- [ ] **123–125** Fabricado según ASES; paredes internas rígidas hasta la tapa; agujeros solo para harness/ventilación/fasteners (≤25 %). ☐
- [ ] **126–128** Celdas fijadas en 3 ejes; tabs de pouch sin carga mecánica; materiales UL94-V0. ☐
- [ ] **129–131** Aberturas no hacia el piloto/operador; tapa rígidamente fijada; fasteners con bloqueo positivo. ☐

**GATE 2 → no cargar sin scruti accu en PASA.**

## Paso 3 — Probar código de carga
- [ ] **3.1** Flashear env `charger` (FW real). Arrancar **sin cargar** (por diseño). ☐ PASA ☐ FALLA
- [ ] **3.2** Lecturas de celda (V/T) por serie coherentes; `BMS_OK` refleja seguridad. ☐ PASA ☐ FALLA
- [ ] **3.3** Comando `g` con **corriente baja** (`CHG_START_CURRENT_A`) → `Vout`/`Iout` suben. ☐ PASA ☐ FALLA
- [ ] **3.4** Cortes de seguridad: forzar celda alta / T fuera de rango → la carga **para**. ☐ PASA ☐ FALLA
- [ ] **3.5** Timeout OBC: dejar de mandar Message 1 → el OBC corta a los 5 s. ☐ PASA ☐ FALLA

## Paso 4 — Cargar batería
- [ ] **4.1** Carga real a I segura. Vigilar V/T de celda y `Vout` del OBC. ☐ PASA ☐ FALLA
- [ ] **4.2** **Codo CV** al llegar a `CHG_TERM_VOLT_V` (≈456 V) → la corriente baja sola. ☐ PASA ☐ FALLA
- [ ] **4.3** Confirmar tensión final del pack con multímetro y anotar: ______ V. ☐ PASA ☐ FALLA
- [ ] **4.4** Balanceo (si aplica) y fin de carga limpio. ☐ PASA ☐ FALLA

**GATE A → acumulador cargado y validado.**

---

# BLOQUE B — Montaje y bring-up LV (pasos 5–7)

## Paso 5 — Ensamblar junta del inversor
- [ ] **5.1** Junta/sellado montado correctamente; estanqueidad si aplica. ☐ PASA ☐ FALLA
- [ ] **5.2** Conectores de potencia (fases U/V/W) y de señal apretados y marcados. ☐ PASA ☐ FALLA
- [ ] **5.3** Resolver/encoder conectado (Sensor Port). ☐ PASA ☐ FALLA

## Paso 6 — Montar PCBs en el coche
- [ ] **6.1** BMS master y VCU montadas, fijadas, sin tensión mecánica en conectores. ☐ PASA ☐ FALLA
- [ ] **6.2** **Bus CAN del coche a 125k** cableado: BMS + VCU + inversor + datalogger, con **120 Ω** en los extremos. ☐ PASA ☐ FALLA
- [ ] **6.3** Líneas SDC (BMS_OK, IMD, TSON, precarga) cableadas según pinout PCB nueva. ☐ PASA ☐ FALLA
- [ ] **6.4** Alimentación LV de cada PCB correcta y masas comunes. ☐ PASA ☐ FALLA

## Paso 7 — Probar códigos PCBs (LV, **SIN HV**)
Validación de **firmware** de cada placa con el coche montado pero **sin alta tensión**.
Referencias `(scruti N)` = nº de ítem de la hoja de inspección eléctrica FSG25.

### 7.A — Código del BMS master (`nucleo_g474re`)
- [ ] **7.A.1 Arranque** — banner + versión FW; BQ79606 init OK; **nº de módulos detectados = esperado** (`TOTALBOARDS`). ☐ PASA ☐ FALLA
- [ ] **7.A.2 Lectura de tensión** — comparar V de 2-3 celdas con multímetro. *Esperado:* error < ~10 mV. ☐ PASA ☐ FALLA
- [ ] **7.A.3 Lectura de temperatura** — sensores plausibles; **≥30 % de celdas monitorizadas** *(scruti 61)*. ☐ PASA ☐ FALLA
- [ ] **7.A.4 `BMS_OK`(PB5)** — **HIGH** en seguro; forzar cada fallo (OV/UV/OT/UT por celda) → **LOW**. ☐ PASA ☐ FALLA
- [ ] **7.A.5 AMS abre SDC** — desconectar 1 hilo de sense de **tensión** → `BMS_OK` LOW en **<0,5 s** *(scruti 103)*; 1 de **temperatura** → **<1 s** *(scruti 104)*; sensor de **corriente** → **<0,5 s** *(98)*; otro conector interno → **<1 s** *(99)*. ☐ PASA ☐ FALLA
- [ ] **7.A.6 Máquina TSON** — `SDC_3V3`(PC7)=H, `TSON_FAIL`(PB8)=L, `HV_ACCU_VIL`(PB4)=L, pulso `TSON_BTN`(PB9) → `SDC_TSON`(PA6)=H y se mantiene; al cambiar `TSON_FAIL`/`SDC_3V3` cae y exige re-pulsar. Sin auto-arme en boot. ☐ PASA ☐ FALLA
- [ ] **7.A.7 Precarga lógica** (simulada) — `PRECHARGE_DONE`(PA7) antes de 5 s → OK; si no → `PRECHARGE_FAIL`(PB6) **latch** (requiere reset). ☐ PASA ☐ FALLA
- [ ] **7.A.8 IMD por CAN** — variar `IMD_OK`(PA8) → bit viaja en el frame. ☐ PASA ☐ FALLA
- [ ] **7.A.9 Contador de fallos** — provocar varios fallos → **incrementa por causa** (flanco) y se manda en su ID CAN. ☐ PASA ☐ FALLA
- [ ] **7.A.10 GEN_STATUS bitmap** — el bit del **módulo en fallo** coincide con el real (`0x0C`). ☐ PASA ☐ FALLA
- [ ] **7.A.11 FRAM** — evento escrito (I2C3 **PC8/PC9**), **sobrevive a reset**, readback correcto. ☐ PASA ☐ FALLA
- [ ] **7.A.12 CAN BMS** — `0x0A` + contador + `0x0C` + ID15 presentes, bytes correctos, **125k**, sin error frames. ☐ PASA ☐ FALLA
- [ ] **7.A.13 Watchdog micro** — bloquear el loop / no alimentar IWDG → **reset** → `BMS_OK` LOW (fail-safe). *(Enlaza con **FM-1**: el reset debe dejar el sistema seguro.)* ☐ PASA ☐ FALLA

### 7.B — Código de la VCU (`nucleo_g474re`, **MODE_CAN**)
- [ ] **7.B.1 Arranque** — serie, modo **CAN**, `heartbeat` incrementa. ☐ PASA ☐ FALLA
- [ ] **7.B.2 APPS calibración** — reposo→fondo, anotar ADC (SINGLE_2/3), fijar `cfgAdcMin/MaxNormal`; throttle 0→1000 lineal. ☐ PASA ☐ FALLA
- [ ] **7.B.3 APPS plausibilidad** — desconectar **≥50 %** APPS y recorrer pedal → throttle/par **0** *(scruti 297)*; desconectar **todo** → 0 *(298)*. ☐ PASA ☐ FALLA
- [ ] **7.B.4 Freno** — lectura + cruza `cfgBrakeTH` para armar R2D. ☐ PASA ☐ FALLA
- [ ] **7.B.5 R2D (secuencia)** — SDC + Start + **freno** → **buzzer** + R2D. **Freno obligatorio al activar** *(291)*; pulsar freno solo una vez antes del botón → **no** R2D *(292)*; **desconectar sensor de freno** → no R2D *(293)*. ☐ PASA ☐ FALLA
- [ ] **7.B.6 Sonido R2D** — duración **1–3 s** continuo *(294)*, reconocible *(296)*. ☐ PASA ☐ FALLA
- [ ] **7.B.7 Dirección PSC-360** (SINGLE_0) y **velocidad de rueda** (PA8/PA9) leen; ajustar calibración/`WHEEL_TEETH`. ☐ PASA ☐ FALLA
- [ ] **7.B.8 Watchdog BMS** — cortar CAN del BMS → a 2,5 s `stsSDC=false` → R2D cae → DriveEnable LOW. ☐ PASA ☐ FALLA
- [ ] **7.B.9 Watchdog inversor** — cortar TX VCU→inversor → `stop()`. ☐ PASA ☐ FALLA
- [ ] **7.B.10 Telemetría + post-mortem** — `0x488` decodifica **`faultCause`/`heartbeat`/`resetCause`**; resto de IDs (0x48B/0x48C/0x48D/0x48E) presentes. *(Base de **FM-2** — sin esto no se caza la parada.)* ☐ PASA ☐ FALLA

**GATE B → toda la lógica LV en PASA antes de seguir.**

---

# BLOQUE C — Ajustes y scruti eléctrico (pasos 8–10)

## Paso 8 — Ajustar APPS
- [ ] **8.1** Pedal reposo→fondo: anotar cuentas ADC (SINGLE_2/3) → fijar `cfgAdcMin/MaxNormal`. Throttle 0→1000 lineal. ☐ PASA ☐ FALLA
- [ ] **8.2** **Plausibilidad APPS (EV.5.5)**: desviar >10% / desconectar un sensor → `throttle=0` y `driveEnabled=false`. ☐ PASA ☐ FALLA
- [ ] **8.3** Mapeo de pedal cómodo (offset de reposo, fondo). ☐ PASA ☐ FALLA

## Paso 9 — Ajustar valores BSPD / APPS-Brake (hardware)
- [ ] **9.1** **BSPD**: simular **5 kW** (con todo el circuito BSPD) + freno fuerte (>0,5 s) → **TS se apaga** *(scruti 299)*. ☐ PASA ☐ FALLA
- [ ] **9.2** **BSPD sensor**: desconectar el sensor de corriente + freno fuerte → **TS se apaga** *(scruti 300)*; reactivación solo tras **10 s sin implausibilidad** *(301)*. ☐ PASA ☐ FALLA
- [ ] **9.3** **APPS/Brake (EV.5.7, HW)**: acelerador >25 % + freno fuerte → **corta par**; no vuelve hasta acelerador <5 %. ☐ PASA ☐ FALLA
- [ ] **9.4** Umbral de freno para R2D (`cfgBrakeTH`) coherente con el sensor. ☐ PASA ☐ FALLA

## Paso 10 — Scruti ELÉCTRICO (FSG25 Part V, ítems 132–301)
> Límite 105 min. Todo trabajo lo aprueba un inspector.

### 10.1 Recursos y tensión TS (132–133)
- [ ] **132** ESO presente + datasheets/ESF/muestras de cable TS + repuestos de PCBs TS + conectores para cerrar SDC/alimentar TS. ☐
- [ ] **133** Medir TS en los TSMP → **≤60 V DC**. ☐

### 10.2 LV battery (134–141)
- [ ] **134–137** ≤60 V DC · carcasa rígida · UL94-V0 (Li-Ion) · protección de cortocircuito (fusible). ☐
- [ ] **138–141** Sobrecorriente (Li-Ion) · aislamiento interno · montaje de celdas · sensores temp ≥30 % (Li-Ion). ☐

### 10.3 PCBs propias y BSPD (142–148)
- [ ] **142–145** Repuesto montado · spacing por tensión · isolators con rating suficiente · coating según datasheet. ☐
- [ ] **146** **BSPD standalone** con interfaz mínima. **147** Alimentado directo del LVMS. **148** Devanado auxiliar del transductor de corriente aislado en sus extremos. ☐

### 10.4 Master switches TSMS/LVMS (149–162)
- [ ] **149–150** Accesibles en el lado derecho, juntos, por encima del 80 % de la altura de hombro de PERCY. ☐
- [ ] **151–156** Fijos · rotativos con mango extraíble · mango ≥50 mm · "ON" horizontal · marcados ON/OFF · TSMS con bloqueo en OFF. ☐
- [ ] **157–162** LVMS marcado "LV" (rayo rojo en triángulo azul) en círculo rojo ≥50 mm · TSMS marcado "TS" (rayo negro/amarillo) en círculo naranja ≥50 mm. ☐

### 10.5 Measuring points (163–169)
- [ ] **163–164** Dos TSMP sobre fondo naranja exclusivo + punto de masa LVS negro. ☐
- [ ] **165–169** Junto a los master switches · jacks banana 4 mm shrouded · tapa no conductora sin herramientas · marcados TS+/TS−/GND. ☐

### 10.6 TS shutdown devices (170–177)
- [ ] **170–172** Dos botones rojos enclavados junto al main hoop (izq/der, altura cabeza), pegatina rayo, ≥39 mm. ☐
- [ ] **173–176** Un botón rojo enclavado en cockpit, pegatina, fácil de accionar, ≥24 mm. ☐
- [ ] **177** **Inertia switch** vertical, fijado al chasis, desmontable para test. ☐

### 10.7 HVD (178–184)
- [ ] **178–181** Marcado "HVD" · >350 mm del suelo · sin actuación remota · interlock integrado. ☐
- [ ] **182–184** Lo quita el ESO en <10 s sin herramientas · TS sigue protegido · incluye interlock. ☐

### 10.8 APPS (185–189)
- [ ] **185** Vuelve a posición de reposo si no se acciona. ☐
- [ ] **186** ≥2 sensores con transferencias distintas (gradiente/offset; checksum si digital). ☐
- [ ] **187–189** Protegido de sobre-esfuerzo (tope) · ≥2 muelles de retorno · cada muelle devuelve el pedal con el otro desconectado. ☐

### 10.9 Brake light + AMS (190–199)
- [ ] **190–193** Una luz de freno roja, en eje central, forma válida, ≥15 cm² o tira LED >150 mm. ☐
- [ ] **194–195** Con laptop: datos del AMS del TSAC (y del LV si Li-Ion) visibles. ☐
- [ ] **196–199** Desconectar TSAC → indicador AMS **rojo**, en cockpit marcado "AMS", visible a pleno sol y para el piloto. ☐

### 10.10 TS wiring e interlocks (200–221)
- [ ] **200–206** Canales TS naranja · solo TS naranja · conducto no conductor/cable apantallado naranja · anclado ≥200 N · fuera de roce · apantallado de partes móviles · TS/LV separados. ☐
- [ ] **207–214** Demostrable gauge/temp/V · sobrecorriente en todos los TS · rating de temp · bloqueo positivo en atornilladas y TSMP · aislamiento no es cinta · **toda la energía pasa por el datalogger** · cable de motores outboard no alcanza el cockpit si se rompe. ☐
- [ ] **215–221** Interlocks en: TSAC · **inversores** · cajas de distribución · datalogger · motores outboard (hilo dedicado x2) · demostrables abriéndolos. ☐

### 10.11 TS enclosures y datalogger (222–228)
- [ ] **222–224** No se alcanza TS con sonda 100×6 mm · protegido de humedad · pegatinas HV. ☐
- [ ] **225–228** Datalogger **totalmente encerrado**, fijado solo con las 2 tiras 3M Dual Lock, funcionalidad/conectividad OK, caja sellada. ☐

### 10.12 Grounding checks (229–242)
- [ ] **229–231** Aislamiento del enclosure: ≥2 MΩ@500 V (sin capa de tierra) o capa de aluminio ≥0,5 mm / acero ≥0,9 mm (TSAC) puesta a tierra. ☐
- [ ] **232–240** Resistencia ≤100 mΩ@1 A a masa LVS de: main hoop · asiento · harness · firewall · TS firewall · TSAC · motores · enclosures · partes salientes. ☐
- [ ] **241–242** Cada tierra aguanta ≥10 % del fusible · partes conductoras a <10 cm del TS ≤100 Ω a masa. ☐

### 10.13 Descarga, BPR y aislamiento (243–248)
- [ ] **243–245** R TS+/TS− = 30 kΩ (2×BPR) + descarga · BPR >6,1 W · descarga continua suficiente. ☐
- [ ] **246–248** Aislamiento (500 V): Riso+ y Riso− **≫315 kΩ** y casi iguales. ☐

---

### ⚡ !! TEST AT HIGH VOLTAGE !! (249–301)
> Ruedas motrices fuera del suelo o quitadas. Multímetro entre TS+ y TS−.

#### 10.14 TS Power-up (249–256) — *enlaza con FM-1 (precarga) y código BMS/VCU*
- [ ] **249** TSMS on, LVMS off → TS **≤60 V**. ☐
- [ ] **250** LVMS on, TSMS off → indicadores **IMD y AMS encienden 1–3 s**. **251** TS ≤60 V. ☐
- [ ] **252** TSMS + paradas on, resetear errores → **TS aún desactivado**. ☐
- [ ] **253** Activar TS y medir → **el sistema precarga antes de cerrar el 2º AIR**. ☐
- [ ] **254** TSMS off → TS **<60 V en 5 s**. **255–256** Intentar power-up con TSMS off / on → TS sigue desactivado. ☐

#### 10.15 TS Shutdown (257–262)
- [ ] **257–262** Cada uno abre el SDC y baja TS <60 V en 5 s: botón cockpit · botón izq · botón der · inertia switch · brake-over-travel · interlocks. ☐

#### 10.16 TS Active Light / TSAL (263–275)
- [ ] **263–265** ≤75 mm bajo el punto alto del main hoop, visible a 3 m, ≤10° tapado. ☐
- [ ] **266–270** LVS on: **TSAL verde solo** · luz "TS off" verde en cockpit, visible. ☐
- [ ] **271–272** TS on: **TSAL rojo parpadeando 2–5 Hz**, CI off, bien visible. ☐
- [ ] **273–275** Desconexión de detección TSAC → rojo parpadea · fuente >60 V a TS → verde+rojo simultáneos + CI on · quitar HVD → TSAL y CI off. ☐

#### 10.17 IMD (276–289)
- [ ] **276** Tierras IMD: una al contenedor y otra al main hoop (cableado separado). ☐
- [ ] **277–278** R_Test entre TS+ y GND → SDC abre **<30 s**, TS <60 V en 5 s. ☐
- [ ] **279–282** Indicador IMD **rojo**, en cockpit marcado "IMD", visible a pleno sol y para el piloto. ☐
- [ ] **283–286** Tras fallo IMD: TS **no reactivable** (ni acción extra, ni reset LVMS, ni tras 40 s, ni botones cockpit). ☐
- [ ] **287** Pulsar reset IMD no accesible al piloto → **reactivable**. **288–289** R_Test en TS− → SDC abre <30 s, indicador IMD enciende. ☐

#### 10.18 Ready-to-Drive activation sequence (290–296) — *código VCU (R2D)*
- [ ] **290** TS activo, pisar acelerador → **el motor NO gira** (sin R2D). ☐
- [ ] **291** Poner R2D → **es necesario pisar freno MIENTRAS se activa**. ☐
- [ ] **292** Pisar freno solo una vez antes del botón → **no hay R2D**. ☐
- [ ] **293** Desconectar el sensor de freno → **no hay R2D**. ☐
- [ ] **294–296** Sonido R2D **1–3 s** continuo · ≥80 dBA a 2 m · reconocible (no canción/animal). ☐

#### 10.19 APPS y BSPD (297–301) — *código VCU (APPS) + BSPD HW*
- [ ] **297** Desconectar **≥50 %** APPS y recorrer pedal → **motores no giran**. ☐
- [ ] **298** Desconectar **todos** los APPS y recorrer pedal → **motores no giran**. ☐
- [ ] **299** Simular **5 kW** (circuito BSPD completo) + freno fuerte (>0,5 s) → **TS se apaga**. ☐
- [ ] **300** Desconectar sensor de corriente + freno fuerte → **TS se apaga**. ☐
- [ ] **301** Reactivación de TS solo posible **tras 10 s sin implausibilidad**. ☐

**GATE C → no energizar tracción (HV) sin scruti eléctrico en PASA.**

---

# BLOQUE D — Inversor y primer movimiento (pasos 11–14)

## Paso 11 — Ajustar inversor (DTI CAN Tool)
- [ ] **11.1** CAN2 = **125 kbit/s**, map **V25**, Send CAN2 status = Enabled. ☐ PASA ☐ FALLA
- [ ] **11.2** **Drive enable via CAN2 = Enabled**, **Node ID = 1**, Timeout ≈ 100 ms. ☐ PASA ☐ FALLA
- [ ] **11.3** Parámetros motor EMRAX 188 (preset), Motor Current Max = 269 Apk. ☐ PASA ☐ FALLA
- [ ] **11.4** Battery Current Max = **125 A**, Max Wattage = 65 kW, Low Voltage Limits según config. ☐ PASA ☐ FALLA

## Paso 12 — Mover motor con fuente (DC de banco, current-limited)
> Primer giro **sin** el pack: fuente HV de banco con **límite de corriente bajo**. Ruedas en el aire.

### 12.A — Motor energizado SIN girar ("cadena viva")
> Confirma VCU→inversor→motor→resolver **sin** que las ruedas lleguen a girar. El motor
> **zumba** (inyección FOC + PWM a velocidad ~0). Hazlo **desde el DTI CAN Tool** (Set
> Position / Set ERPM 0) o con **corriente relativa muy baja y freno puesto**.
> ⚠️ **Térmico**: rotor parado concentra el I²R → **corriente baja, tiempo corto**, vigilar
> temp de motor/inversor (`0x441`). No usar `Set ERPM`/`Set Position` desde la VCU (cambia el modo del DTI).
- [ ] **12.A.1** Energizar el bus DC (fuente, V moderada, **I limitada**). ☐ PASA ☐ FALLA
- [ ] **12.A.2** Comandar **hold** (posición o 0 ERPM) o corriente relativa baja con freno. *Esperado:* el motor **zumba/empuja sin girar**; sentido de empuje correcto. ☐ PASA ☐ FALLA
- [ ] **12.A.3** Vigilar **temperatura** de motor/inversor durante el test (no calentar). ☐ PASA ☐ FALLA

### 12.B — Primer giro libre
- [ ] **12.B.1** Comandar par mínimo. *Esperado:* el motor gira **en el sentido correcto**, suave. ☐ PASA ☐ FALLA
- [ ] **12.B.2** eRPM (`0x401`) coherente con el giro (eRPM = RPM_mec × 10). ☐ PASA ☐ FALLA
- [ ] **12.B.3** **Validar escala Vin (`0x401`)**: comparar V_dc reportado con la fuente. ☐ PASA ☐ FALLA

**GATE D1 → primer giro OK con fuente antes de meter el pack.**

## Paso 13 — Mover motor con TSAC (acumulador real) — **CAZA FM-1**
> ⚠️ HV real. Responsable HV. Ruedas en el aire. Límites de corriente **bajos**.
- [ ] **13.1** IMD OK + aislamiento antes de cerrar AIR. ☐ PASA ☐ FALLA
- [ ] **13.2** **TSON + precarga real**: pulsar TSON → `SDC_TSON` cierra → precarga sube el bus a ≈Vpack en <5 s → `PRECHARGE_DONE` → AIR cierran. **El sistema precarga antes de cerrar el 2º AIR** *(scruti 253)*; al apagar TSMS, TS < 60 V en 5 s *(254)*. ☐ PASA ☐ FALLA
- [ ] **13.3** **FM-1**: repetir precarga **×20** vigilando `resetCause` / 3.3 V / `BMS_OK`. **Sin un solo reset.** (Ver sección FM-1.) ☐ PASA ☐ FALLA
- [ ] **13.4** Validar V_dc del `0x401` contra multímetro del pack. ☐ PASA ☐ FALLA
- [ ] **13.5** Ajustar `V_PACK_MIN_OP` (**292** si 10 módulos / 350 si 12). ☐ PASA ☐ FALLA
- [ ] **13.6** Giro suave con el pack; sentido y eRPM correctos. ☐ PASA ☐ FALLA

## Paso 14 — Probar comunicación VCU↔Inversor
> *Nota:* conviene verificar la **RX/TX (14.1/14.2) en LV antes** de comandar par en 12/13.
- [ ] **14.1** VCU **recibe** `0x401` (eRPM/Vin), `0x441` (fault), `0x481` (drive enable). ☐ PASA ☐ FALLA
- [ ] **14.2** VCU **manda** `0x181` (drive enable), `0x0A1` (relative current), `0x101` (max AC), `0x141` (max DC). ☐ PASA ☐ FALLA
- [ ] **14.3** **Drive enable**: armar R2D → bit del `0x481` = 1; desarmar → 0. ☐ PASA ☐ FALLA
- [ ] **14.4** **PowerLimiter**: a más eRPM / V_dc bajo, el cap de throttle reduce corriente; a baja velocidad no limita. ☐ PASA ☐ FALLA
- [ ] **14.5** **Corte subtensión**: V_dc < `V_PACK_MIN_OP` → throttle 0. ☐ PASA ☐ FALLA
- [ ] **14.6** **Watchdog**: cortar TX VCU→inversor → `stop()` / free-running seguro. ☐ PASA ☐ FALLA

**GATE D → tracción validada en banco/caballetes.**

---

# BLOQUE E — Coche en el suelo (paso 15)

## Paso 15 — Probar coche en el suelo — **CAZA FM-2**
> Zona cerrada y delimitada. Piloto con EPI. Empezar **muy** despacio, límites bajos.
- [ ] **15.1** **FM-2 setup**: confirmar que **TODO** se loggea en el Solo 2 DL (VCU 0x488 con `faultCause`/`heartbeat`/`resetCause`, contador BMS, fault inversor). ☐ PASA ☐ FALLA
- [ ] **15.2** Rodadura a baja velocidad; respuesta de pedal suave, sin tirones. ☐ PASA ☐ FALLA
- [ ] **15.3** Frenado eficaz; sin par parásito. ☐ PASA ☐ FALLA
- [ ] **15.4** **Recorrer >200 m** sin parada no comandada. Si para → **leer la causa** (FM-2) y corregir. ☐ PASA ☐ FALLA
- [ ] **15.5** Velocidad de rueda real → **validar `WHEEL_TEETH`** (y diámetro si se mete km/h). ☐ PASA ☐ FALLA
- [ ] **15.6** Dirección (STEER%) coherente en marcha. ☐ PASA ☐ FALLA
- [ ] **15.7** Subir límites de corriente **progresivamente** (restaurar `CURRENT_AC_MAX`/`DC_MAX`) verificando comportamiento. ☐ PASA ☐ FALLA

---

## Apéndice A — Placeholders a fijar durante las pruebas

| Parámetro | Fichero | Prueba | Final |
|---|---|---|---|
| `STEER_ADC_LEFT/CENTER/RIGHT` | VCU `Config.h` | placeholder | ____ |
| `WHEEL_TEETH` · `PIN_WHEEL_L/R` | VCU `Config.h` | 1 · PA8/PA9 | ____ |
| `VBAT_DIVIDER` | VCU `InverterControl.cpp` | 1.0 | ____ |
| `V_PACK_MIN_OP` | VCU `Config.h` | 350 | 292 (10) / 350 (12) |
| `CURRENT_AC_MAX`/`_APK`/`DC_MAX` | VCU `Config.h` | 190/269/125 | ____ |
| APPS `cfgAdcMin/MaxNormal` | VCU `InverterControl.cpp` | actuales | ____ |
| `CHG_TERM_VOLT_V` · `CHG_MAX_CURRENT_A` | BMS `charger.cpp` | 456 · 4.0 | ____ |
| `PRECHARGE_TIMEOUT_MS` | BMS `main.cpp` | 5000 | ____ |

## Apéndice B — Config del inversor DTI (resumen)
- CAN2 **125 kbit/s** · map **V25** · Drive enable via CAN2 = **On** · Node **1** · Timeout ~100 ms
- Motor Current Max **269 Apk** · Battery Current Max **125 A** · Max Wattage **65 kW**

---

*Documento vivo: anota fallos, valores medidos y ajustes de calibración en cada pasada.
Pasos 2 y 10 (scruti): completar con la hoja oficial de scrutineering.*
