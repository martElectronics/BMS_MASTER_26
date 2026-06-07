# `SocEstimator` — Estimación de State-of-Charge

`lib/SocEstimator/SocEstimator.h` (header-only)

## Propósito
Estima el **% de carga** del pack combinando dos métodos (el estándar en automoción/FS):
1. **Coulomb counting** (motor): integra la corriente del pack (Hall) sobre la capacidad.
   Preciso en dinámico, pero **deriva** con el tiempo.
2. **Corrección por OCV en reposo:** cuando la corriente es baja y estable un rato y las
   tensiones son fiables, re-ajusta el SOC hacia el que indica la curva **OCV→SOC**
   (con un nudge suave por EMA, sin saltos).
3. **Inicialización:** al arrancar en reposo, SOC = OCV→SOC de la celda más baja.

## Cómo se usa (`main.cpp`)
```cpp
#define SOC_PACK_CAPACITY_AH (4.0f * 11)   // ANTES del include. 4.0 Ah celda × Np
SocEstimator soc;
soc.begin(minCellV);                       // en reposo al arrancar
soc.update(packCurrentA, minCellV, bms.isVoltageReadingReliable());  // cada loop
uint8_t pct = soc.soc();                   // 0..100 (para CAN UINT8)
```
`voltsReliable` evita re-snappear el OCV durante el balanceo HW (tensiones falseadas).

## Configuración
| Constante | Qué |
|---|---|
| **`SOC_PACK_CAPACITY_AH`** | capacidad **real** del pack = 4.0 Ah × Np. **AJUSTAR a la topología.** |
| `SOC_REST_CURRENT_A` (2 A) | |I| por debajo = candidato a reposo |
| `SOC_REST_MS` (30 s) | estable este tiempo → reposo fiable |
| `SOC_RECAL_ALPHA` (0.05) | suavidad del re-ajuste hacia OCV |

## ⚠ Estado: APROXIMADO (pendiente de caracterizar)
La tabla **`_OCV_SOC`** es una curva **NCA 21700 genérica**, **NO medida** en estas
celdas (el datasheet del 40T no trae OCV en reposo, solo bajo carga). Para precisión
real hay que **caracterizar**: reposar el pack a SOC conocidos, registrar la OCV, y
sustituir la tabla. Mientras tanto el SOC es **orientativo** (el coulomb counting acota
la deriva entre reposos).

## Nota
La capacidad del pack depende solo de **Np** (celdas en paralelo), no del nº de módulos
en serie — por eso el banco (10 mód) y el pack final (12 mód) tienen la misma capacidad.
