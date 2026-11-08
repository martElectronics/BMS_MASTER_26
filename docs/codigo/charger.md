# `charger.cpp` — Firmware de carga (OBC TC HK-L 6.6 kW)

`src/charger.cpp` · env **`[env:charger]`** (independiente de `main.cpp`)

## Propósito
Firmware **separado** para cargar el pack con el cargador OBC **TC HK-L 6.6 kW**.
Reutiliza el driver BQ79606 (V/T de celda para seguridad) y MART_CAN, pero **no** usa
Hall/SOC/ventiladores/telemetría del coche. Va en su **propio env** porque el CAN del
coche va a 125 kbps y el del **cargador a 500 kbps**, y son sesiones distintas.

> Lanzar: `pio run -e charger -t upload` · Monitor: `pio device monitor -e charger`
> Para verificar SOLO la comunicación CAN (sin cargar) está el env `charger_cantest`
> (en la rama `testing`).

## Protocolo OBC (J1939, IDs extendidos, 500 kbps)
- **BMS → Cargador `0x1806E5F4`** cada 1 s:
  `B0-1` Vmax (0.1 V/bit, BE) · `B2-3` Imax (0.1 A/bit, BE) · `B4` control (0=cargar, 1=parar).
- **Cargador → BMS `0x18FF50E5`** cada 1 s:
  `B0-1` Vout (0.1 V) · `B2-3` Iout (0.1 A) · `B4` status (HWFAIL/OVERTEMP/INPUT_V/BAT_CONN/COMM_TO).
- Si el OBC **no recibe** el frame del BMS en **5 s** → corta la salida.

## Estrategia: corriente DC fija
El cargador hace el codo **CV** solo al llegar a `CHG_TERM_VOLT_V`. El firmware **no**
regula la corriente para el límite AC: por eso `CHG_*_CURRENT_A` debe dimensionarse para
no pasarse del límite AC ni con el pack lleno. **Arranca SIN cargar** (`g` para empezar).

## Seguridad (corta la carga si...)
- Lectura del BQ falla · celda ≥ `CELL_VMAX_HARD_V` (4.25 V, **OV**) · celda ≤
  `CELL_VMIN_HARD_V` (2.8 V, **UV** — también atrapa un cable de *sense* abierto, que
  lee ~0 V o negativo) · `Tmax ≥ CELL_TMAX_CHG_C` (45 °C) · `Tmin ≤ CELL_TMIN_CHG_C`
  (0 °C — Li-ion no carga en frío).
- Un fallo **cancela** la orden: hay que re-armar con `g`.
- `BMS_OK` refleja la **seguridad** del pack (OK=HIGH), no si se está cargando.

## Comandos serie
`h` (o `?`) imprime la lista completa. `g` start · `x` stop · `c,<I>` fijar corriente
(capada a `CHG_MAX_CURRENT_A`) · `d` datos · `v` voltajes · `t` temps · `a` Hall
· `R` resistencia por paralelo · `j` dashboard JSON · `f,<ms>` ventana de comms
· `i` re-init BQ · `S` dormir BQ · `r` reset.

## ⚠ Config PROVISIONAL — confirmar antes de cargar un pack real
| `#define` | Valor | Nota |
|---|---|---|
| `CHG_TERM_VOLT_V` | 456 V | Vmax fin de carga (valor del FW antiguo probado). **Confirmar con multímetro** la tensión del pack al 100 %. |
| `CHG_START_CURRENT_A` | 3 A | corriente DC de arranque |
| `CHG_MAX_CURRENT_A` | 4 A | tope duro (atado al límite AC/plomos) |
| `CHG_CAN_BAUD` | 500 | baud del cargador |
| `CELL_TMAX/TMIN_CHG_C` | 45 / 0 °C | ventana térmica de carga |
| `CELL_VMAX/VMIN_HARD_V` | 4.25 / 2.8 V | ventana de tensión por celda (UV = mismo umbral que `CELL_UV_V` de `main.cpp` y que `UV_THRESH` del BQ) |

> Todos los `#define` marcados **CONFIRMAR** son provisionales — no flashear a un pack
> real sin validarlos en banco (ver [PRUEBAS_ELECTRONICA.md](../PRUEBAS_ELECTRONICA.md) pasos 1-4).

## Medir la capacidad de cada paralelo durante una carga

Herramientas en [tools/registro_carga/](../../tools/registro_carga/) (no tocan el firmware: usan el
JSON del dashboard, tecla `j`, que ya saca los 110 paralelos, los 90 NTC, el Hall y la
corriente del cargador cada 0,5 s).

1. Pack bajo (después de una tanda) y 15-20 min de reposo.
2. `registrar_carga.bat` con el PC conectado al STM **antes** de dar corriente.
3. Cargar a corriente fija (7 A en el taller) hasta ~4,05 V, sin llegar a la fase de tensión constante.
4. Parar la carga y dejar el registro 30 min más (reposo final). Ctrl+C.
5. `analizar_carga.bat` con el CSV de `registros/` → `resultados/<carga>.xlsx`.

Qué sale:
- **Capacidad relativa** de cada paralelo: Ah que necesita para subir cada tramo de 0,1 V frente a la
  mediana. 0,73 ≈ 8 de 11 celdas. No necesita reposo ni curva de la celda.
- **Capacidad absoluta** (pack y paralelo): tensión de reposo antes/después con la curva típica de la
  40T. Necesita los dos reposos (≥ 10 min el final).
- **Resistencia** de cada paralelo con el escalón de corriente al arrancar.
