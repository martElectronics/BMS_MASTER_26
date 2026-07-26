#pragma once
// ============================================================================
//  board_pins.h — Mapa de pines de la PCB BMS Master (STM32G474RE, rev. nueva)
//
//  ÚNICA fuente de verdad de pines. Lo incluyen main.cpp y charger.cpp (y
//  cualquier firmware que corra en esta placa). Antes estaban duplicados en
//  cada .cpp y divergían → bugs.
//
//  ⚠⚠ CONVENCIÓN DE FORMATO (CRÍTICA — un error aquí acciona OTRA pata EN
//     SILENCIO, no da error de compilación):
//   · Pines que van al DRIVER BQ (dentro de BQConfig) → PinName CON guion (PA_x).
//     El driver los convierte con BQ_DPIN() = pinNametoDigitalPin().
//   · Pines de uso DIRECTO (digitalRead/digitalWrite/pinMode/analogRead/
//     analogWrite) → nº de pin Arduino SIN guion (PAx). Con PA_x (PinName) esas
//     APIs lo interpretan como ÍNDICE de pin → accionan la pata equivocada.
//     (Este bug nos costó las comms del BQ, el TSON y el PWM de los fans.)
// ============================================================================

#include <Arduino.h>

// ── BQ79606 — van en BQConfig → BQ_DPIN → PinName (CON guion) ────────────────
#define PIN_BQ_WAKE     PB_7
#define PIN_BQ_FAULT    PB_2
#define PIN_BQ_RX       PC_5
#define PIN_BQ_TX       PC_4
#define PIN_BMS_OK      PB_5    ///< BMS_OK_STM → SDC. OK=HIGH (lo gestiona el driver).

// ── SDC / TSON / precarga — digitalRead/Write/pinMode DIRECTO (SIN guion) ────
#define PIN_TSON_FAIL       PB8    ///< TSON_FAIL_STM (in). HIGH = fallo TSON
#define PIN_TSON_BTN        PB9    ///< TSON_STM (in). Pulsador de arranque del TSON
#define PIN_SDC_TSON        PA6    ///< SDC_TSON_STM (out). Latch del TSON
#define PIN_PRECHARGE_DONE  PA7    ///< PRECHARGE_DONE_STM (in). HIGH = precarga OK
#define PIN_PRECHARGE_FAIL  PB6    ///< PRECHARGE_FAIL_STM (out). HIGH enclavado si timeout 5 s
#define PIN_SDC_3V3         PC7    ///< SDC_3V3_STM (in). HIGH = SDC presente
#define PIN_IMD_OK          PA8    ///< IMD_OK_STM (in). Solo telemetría/print
#define PIN_HV_ACCU_VIL     PB4    ///< HV_ACCU_VIL_STM (in). Condición de armado del TSON

// ── Amperímetro DHAB S/118 — analogRead DIRECTO (SIN guion) ──────────────────
//   Confirmado contra el cableado: 30A (alta sens.) → PA0 ; 350A (baja) → PA1.
#define PIN_AMP_30A     PA1    ///< Canal alta resolución (±30A)
#define PIN_AMP_350A    PA0    ///< Canal baja resolución (±350A)

// ── PWM ventiladores — pinMode/analogWrite DIRECTO (SIN guion) ───────────────
#define PIN_PWM         PB10   ///< PWM_STM (out) → FanController