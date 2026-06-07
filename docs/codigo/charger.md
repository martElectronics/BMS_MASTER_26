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
- Lectura del BQ falla · celda ≥ `CELL_VMAX_HARD_V` (4.25 V) · `Tmax ≥ CELL_TMAX_CHG_C`
  (45 °C) · `Tmin ≤ CELL_TMIN_CHG_C` (0 °C — Li-ion no carga en frío).
- Un fallo **cancela** la orden: hay que re-armar con `g`.
- `BMS_OK` refleja la **seguridad** del pack (OK=HIGH), no si se está cargando.

## Comandos serie
`g` start · `x` stop · `c,<I>` fijar corriente (capada a `CHG_MAX_CURRENT_A`) · `d` datos
· `r` reset.

## ⚠ Config PROVISIONAL — confirmar antes de cargar un pack real
| `#define` | Valor | Nota |
|---|---|---|
| `CHG_TERM_VOLT_V` | 456 V | Vmax fin de carga (valor del FW antiguo probado). **Confirmar con multímetro** la tensión del pack al 100 %. |
| `CHG_START_CURRENT_A` | 3 A | corriente DC de arranque |
| `CHG_MAX_CURRENT_A` | 4 A | tope duro (atado al límite AC/plomos) |
| `CHG_CAN_BAUD` | 500 | baud del cargador |
| `CELL_TMAX/TMIN_CHG_C` | 45 / 0 °C | ventana térmica de carga |

> Todos los `#define` marcados **CONFIRMAR** son provisionales — no flashear a un pack
> real sin validarlos en banco (ver [PRUEBAS_ELECTRONICA.md](../PRUEBAS_ELECTRONICA.md) pasos 1-4).
