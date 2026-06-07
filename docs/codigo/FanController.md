# `FanController` — Ventiladores del pack

`lib/FanController/FanController.h` (header-only)

## Propósito
Controla los ventiladores de refrigeración del pack. **Variable de control: `Tmax`**
(la celda más caliente manda). Es **gestión térmica, NO seguridad** — el corte de
seguridad sigue siendo el de 60 °C/1 s del `main` (los fans solo intentan que no se llegue).

## Curva (lineal por tramos, con histéresis)
```
Tmax < FAN_T_OFF (32°C)          → OFF (0 %)
Tmax ≥ FAN_T_ON  (35°C)          → ON
FAN_T_ON → FAN_T_FULL (35→50°C)  → rampa FAN_MIN_DUTY (30%) → 100 %
Tmax ≥ FAN_T_FULL (50°C)         → 100 %
```
`FAN_T_FULL` (50 °C) deja **10 °C de margen** al corte FS de 60 °C: los fans saturan
mucho antes de cualquier fallo térmico → si `BMS_OK` cae por T, es un fallo **real**
de refrigeración.

## Feed-forward por corriente
Con |I| de pack alta (`> FAN_FF_CURRENT_A`) se fuerza un **piso de duty** (`FAN_FF_DUTY`,
50 %) **aunque `Tmax` no haya subido aún** — anticipa la inercia térmica. Se auto-desactiva
al bajar la corriente.

## Cómo se usa (`main.cpp`)
```cpp
FanController fan(PIN_PWM);     // PB10
fan.begin();                   // setup: PWM (8-bit, FAN_PWM_HZ)
fan.update(tmax, packCurrentA, failSafe);   // cada loop → duty %
```
**`failSafe`** (lo decide el `main`): si la temperatura no es fiable/fresca (comms BQ
caídas, NTC abierto) → ventiladores al **100 %** en vez de fiarse de una `Tmax` rancia.

## Configuración `[TUNE]`
| Constante | Qué | Estado |
|---|---|---|
| `FAN_FF_CURRENT_A` (100 A) | I que dispara el feed-forward | depende de la corriente continua del pack (límite celda × Np) |
| `FAN_PWM_HZ` (1000) | frecuencia PWM | según el driver de ventilador concreto |
| `FAN_T_*` | umbrales de la curva | afinar con el comportamiento térmico real |

## HW
Ventilador 2/3 hilos → PWM de **baja frecuencia** modulando la alimentación vía
driver/MOSFET. Duty 8-bit (0..255).
