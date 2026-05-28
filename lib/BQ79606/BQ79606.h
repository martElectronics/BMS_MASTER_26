/**
 * @file    BQ79606.h
 * @brief   Driver para el monitor de baterías BQ79606A-Q1 de Texas Instruments.
 *
 * Este archivo define:
 *   - BQConfig:       Struct de configuración de hardware (pines, baudrate).
 *   - BQResult:       Enum con los posibles resultados de una operación.
 *   - BQFaultStatus:  Struct con el estado de fallos de un board.
 *   - BQ79606:        Clase principal del driver.
 *
 * ── ARQUITECTURA ────────────────────────────────────────────────────────────
 * El BQ79606A-Q1 es un IC que mide el voltaje de hasta 6 celdas de batería
 * y la temperatura mediante NTCs en sus GPIOs. Se pueden encadenar varios ICs
 * en una cadena daisy-chain UART con protocolo propietario de TI.
 *
 * Este driver gestiona:
 *   · Autoadressing:   asignación automática de dirección a cada IC.
 *   · Voltajes:        lectura de VCELL1H–VCELL6L de cada IC.
 *   · Temperaturas:    lectura de AUX_GPIO1H–AUX_GPIO6H y conversión a °C.
 *   · Fallos:          lectura de FAULT_SUM, UV/OV/UT/OT y SYS_FAULTx.
 *   · Balanceo:        control de timers CB_CELLx_CTRL y flag BAL_GO.
 *
 * ── PORTABILIDAD (ESP32 / STM32) ────────────────────────────────────────────
 * La clase es independiente del hardware gracias a BQConfig.
 * Para portar solo hay que cambiar la configuración en el main.cpp —
 * la clase no necesita modificarse.
 *
 *   ESP32:
 *     HardwareSerial bmsSerial(2);
 *     BQConfig cfg = {.pinWake=13, .pinFault=15, .pinBmsOk=3,
 *                     .pinRx=16, .pinTx=17, .pinTxEnable=10, .baudrate=125000};
 *     BQ79606 bms(bmsSerial, cfg);
 *
 *   STM32 (framework Arduino):
 *     HardwareSerial bmsSerial(PD_6, PD_5);
 *     BQConfig cfg = {.pinWake=PB_0, .pinFault=PC_1, .pinBmsOk=PA_5,
 *                     .pinRx=PD_6, .pinTx=PD_5, .pinTxEnable=-1, .baudrate=125000};
 *     BQ79606 bms(bmsSerial, cfg);
 *
 * ── REFERENCIAS ─────────────────────────────────────────────────────────────
 *   · BQ79606A-Q1 Datasheet (SLUSE5B)
 *   · BQ79606 Software Design Reference (SLVA970E)
 */

#ifndef BQ79606_H
#define BQ79606_H

#include <Arduino.h>

// ============================================================================
//  CONSTANTES DE PROTOCOLO
//  Solo dependen de la topología de la cadena, no del hardware del MCU.
// ============================================================================

/** Número total de ICs BQ79606 en la cadena daisy-chain. Ajustar según diseño. */
#define TOTALBOARDS   20  ///< Nº de ICs BQ79606 en la cadena (= chips, NO módulos). Debe coincidir EXACTO con el HW conectado. Recompilar+reflashear al cambiar.

/**
 * Bytes de datos por lectura de voltaje.
 * 6 celdas × 2 bytes (byte alto + byte bajo) = 12 bytes.
 */
#define MAXBYTES      12

/**
 * Número de celdas válidas de un board según su índice.
 * Board par  (0,2,4,...): 6 celdas reales.
 * Board impar (1,3,5,...): 5 celdas reales — celda 6 mide ~0V (no conectada).
 *
 * Usar CELLS_FOR_BOARD(b) en lugar de CELLS_LAST_BOARD para soportar
 * múltiples módulos correctamente.
 */
#define CELLS_FOR_BOARD(b)  ((b) % 2 == 0 ? 6 : 5)

/**
 * Número de NTCs válidos por board.
 * Board par  (0,2,4,...): 6 NTCs (GPIO1-GPIO6).
 * Board impar (1,3,5,...): 3 NTCs (GPIO1-GPIO3).
 */
static constexpr uint8_t NTCS_PER_BOARD[2] = { 6, 3 };  ///< Indexar con [b % 2]

// Centinelas y rango físico plausible de un NTC del pack. Una lectura fuera
// de [BQ_NTC_TMIN_C, BQ_NTC_TMAX_C], NaN/inf, o raw==0x8000 (diagnóstico ADC)
// se marca como NTC abierto/inválido y NO entra en _tMin/_tMax.
static constexpr float BQ_NTC_OPEN_C = -1000.0f;  ///< Centinela: NTC configurado pero abierto/inválido (distinto de -99 = no poblado)
static constexpr float BQ_NTC_TMIN_C =   -40.0f;  ///< Mínimo físico plausible (°C)
static constexpr float BQ_NTC_TMAX_C =   125.0f;  ///< Máximo físico plausible (°C)


// ============================================================================
//  CONFIGURACIÓN DE HARDWARE
//  Rellenar esta struct en el main.cpp según el PCB utilizado.
//  Los pines ajenos al BQ (ventiladores, amperímetro, SDC...)
//  deben configurarse en el main, no aquí.
// ============================================================================

/**
 * @brief Tipo de pin portable.
 *
 * ⚠ STM32 vs ESP32 — bug crítico de portabilidad:
 *   En STM32duino, los tokens PB_7 / PC_5 (con guion bajo) son valores del
 *   enum `PinName` (codifican puerto+pin), NO índices de pin Arduino.
 *   Si se guardan en uint8_t y se pasan a pinMode()/digitalWrite()/
 *   HardwareSerial(uint32_t,uint32_t), el core los reinterpreta como
 *   índices Arduino → pin equivocado (silencioso) o HardFault al resolver
 *   el periférico USART. Usando el tipo `PinName` se seleccionan las
 *   sobrecargas correctas (pinMode(PinName,...), HardwareSerial(PinName,
 *   PinName)) y todo resuelve al puerto/periférico real.
 *   En ESP32 el número de GPIO ES el número de la API → uint8_t es correcto.
 */
#if defined(ARDUINO_ARCH_STM32)
  typedef PinName BQPin;
  // STM32duino NO tiene overloads pinMode(PinName)/digitalWrite(PinName):
  // wiring_digital.h solo declara (uint32_t,uint32_t). Pasar un PinName
  // crudo se convierte a uint32_t y digitalPinToPinName() lo reinterpreta
  // como ÍNDICE de pin Arduino (digitalPin[idx]) → acciona un pin
  // EQUIVOCADO de forma silenciosa (no HardFault). Por eso el HardwareSerial
  // (que SÍ tiene ctor PinName) funcionaba pero WAKE/comm-reset/BMS_OK/FAULT
  // accionaban pines erróneos y la cadena nunca respondía.
  // pinNametoDigitalPin() (pins_arduino.h) hace la conversión correcta
  // PinName → nº de pin Arduino. Usar BQ_DPIN() en TODA llamada
  // pinMode/digitalWrite/digitalRead con pines de BQConfig.
  static inline uint32_t BQ_DPIN(BQPin p) { return pinNametoDigitalPin((PinName)p); }
#else
  typedef uint8_t BQPin;
  static inline uint8_t  BQ_DPIN(BQPin p) { return p; }   // ESP32: nº GPIO == nº API
#endif

/**
 * @brief Configuración de hardware específica del PCB.
 *
 * Se pasa al constructor de BQ79606. Permite portar el driver
 * a cualquier plataforma sin modificar la clase.
 *
 * ⚠ En STM32 los pines deben ser tokens PinName (PB_7, PC_5, …),
 *   NO números crudos. En ESP32, números de GPIO normales.
 */
struct BQConfig {
    uint8_t  uartPort;     ///< Solo ESP32 (índice de UART). No usado en STM32.
    BQPin    pinWake;      ///< Pin WAKE del BQ79606. Pulso LOW activa el IC desde shutdown.
    BQPin    pinFault;     ///< Pin FAULT del BQ79606 (entrada). LOW = hay fallo hardware.
    BQPin    pinBmsOk;     ///< Pin BMS_OK hacia el SDC. LOW = todo OK, HIGH = fallo.
    BQPin    pinRx;        ///< Pin RX de la UART del microcontrolador.
    BQPin    pinTx;        ///< Pin TX de la UART del microcontrolador.
    int32_t  pinTxEnable;  ///< Pin OE del level shifter. -1 si no se usa; si se usa, valor de pin.
    uint32_t baudrate;     ///< Baudrate de la comunicación daisy-chain (ej: 125000).
};

// ============================================================================
//  LÍMITES DE SEGURIDAD
//  Una sola fuente de verdad para las protecciones de celda/NTC que se
//  vigilan en lectura NORMAL (no solo al balancear). Se aplican con
//  setSafetyLimits() y los evalúan evalSafetyVoltage()/evalSafetyTemperature().
//
//  Anti-ruido (debounce): una condición fuera de rango debe persistir más
//  que la ventana (vDebounceMs/tDebounceMs) para confirmarse; si vuelve a
//  rango antes, se descarta como ruido.
// ============================================================================
struct BQSafetyLimits {
    float    cellOV_V;      ///< Voltaje máximo por celda (V). Por encima → OV.
    float    cellUV_V;      ///< Voltaje mínimo por celda (V). Por debajo → UV.
    float    overTemp_C;    ///< Temperatura máxima de NTC (°C). Por encima → OT.
    float    underTemp_C;   ///< Temperatura mínima de NTC (°C). Por debajo → UT.
    uint16_t vDebounceMs;   ///< Ventana anti-ruido para voltaje (ms).
    uint16_t tDebounceMs;   ///< Ventana anti-ruido para temperatura (ms).
};


// ============================================================================
//  TIPOS DE FRAME (PROTOCOLO DAISY-CHAIN BQ79606)
//  Definen cómo se dirige una trama dentro de la cadena.
//  El primer byte de cada trama incluye el tipo de frame.
//  Ver SLVA970E sección 3 para descripción completa del protocolo.
// ============================================================================

#define FRMWRT_SGL_R      0x00  ///< Lectura de un único IC por dirección
#define FRMWRT_SGL_NR     0x10  ///< Escritura en un único IC (sin respuesta)
#define FRMWRT_STK_R      0x20  ///< Lectura de todos los ICs de la cadena excepto el base
#define FRMWRT_STK_NR     0x30  ///< Escritura en la cadena excepto el base (sin respuesta)
#define FRMWRT_ALL_R      0x40  ///< Lectura broadcast: todos los ICs responden
#define FRMWRT_ALL_NR     0x50  ///< Escritura broadcast: todos los ICs ejecutan (sin respuesta)
#define FRMWRT_REV_ALL_NR 0xE0  ///< Escritura broadcast en dirección inversa (sin respuesta)


// ============================================================================
//  MAPA DE REGISTROS DEL BQ79606A-Q1
//  Direcciones de todos los registros utilizados por el driver.
//  Fuente: BQ79606A-Q1 Datasheet (SLUSE5B), sección "Register Map".
// ============================================================================

// ── Configuración general ────────────────────────────────────────────────────
#define DEVADD_OTP              0x0000  ///< Dirección del IC grabada en OTP
#define CONFIG                  0x0001  ///< Rol del IC en la cadena (base / stack / top)

// ── Máscaras de fallo ────────────────────────────────────────────────────────
// Escribir 1 en un bit enmascara (silencia) ese tipo de fallo.
// En init se enmascaran todos para evitar falsos positivos al arrancar.
#define GPIO_FLT_MSK            0x0002  ///< Máscara de fallos GPIO
#define UV_FLT_MSK              0x0003  ///< Máscara de undervoltage de celda
#define OV_FLT_MSK              0x0004  ///< Máscara de overvoltage de celda
#define UT_FLT_MSK              0x0005  ///< Máscara de undertemperature (GPIO)
#define OT_FLT_MSK              0x0006  ///< Máscara de overtemperature (GPIO)
#define TONE_FLT_MSK            0x0007  ///< Máscara de fallo de tono en bus
#define COMM_UART_FLT_MSK       0x0008  ///< Máscara de fallo UART genérico
#define COMM_UART_RC_FLT_MSK    0x0009  ///< Máscara de fallo UART recepción comando
#define COMM_UART_RR_FLT_MSK    0x000A  ///< Máscara de fallo UART recepción respuesta
#define COMM_UART_TR_FLT_MSK    0x000B  ///< Máscara de fallo UART transmisión
#define COMM_COMH_FLT_MSK       0x000C  ///< Máscara de fallo bus COMH
#define COMM_COMH_RC_FLT_MSK    0x000D  ///< Máscara de fallo COMH recepción comando
#define COMM_COMH_RR_FLT_MSK    0x000E  ///< Máscara de fallo COMH recepción respuesta
#define COMM_COMH_TR_FLT_MSK    0x000F  ///< Máscara de fallo COMH transmisión
#define COMM_COML_FLT_MSK       0x0010  ///< Máscara de fallo bus COML
#define COMM_COML_RC_FLT_MSK    0x0011  ///< Máscara de fallo COML recepción comando
#define COMM_COML_RR_FLT_MSK    0x0012  ///< Máscara de fallo COML recepción respuesta
#define COMM_COML_TR_FLT_MSK    0x0013  ///< Máscara de fallo COML transmisión
#define OTP_FLT_MSK             0x0014  ///< Máscara de fallo OTP
#define RAIL_FLT_MSK            0x0015  ///< Máscara de fallo de rails de alimentación
#define SYSFLT1_FLT_MSK         0x0016  ///< Máscara de fallo de sistema 1
#define SYSFLT2_FLT_MSK         0x0017  ///< Máscara de fallo de sistema 2
#define SYSFLT3_FLT_MSK         0x0018  ///< Máscara de fallo de sistema 3
#define OVUV_BIST_FLT_MSK       0x0019  ///< Máscara de fallo BIST del comparador OV/UV
#define OTUT_BIST_FLT_MSK       0x001A  ///< Máscara de fallo BIST del comparador OT/UT

// ── Control de comunicación ──────────────────────────────────────────────────
#define COMM_CTRL               0x0020  ///< Control de baudrate de la UART daisy-chain
#define DAISY_CHAIN_CTRL        0x0021  ///< Habilitación de TX/RX en la cadena
#define TX_HOLD_OFF             0x0022  ///< Tiempo de retención del transmisor tras stop bit
#define COMM_TO                 0x0023  ///< Timeout de comunicación (0x00 = desactivado)

// ── Configuración del ADC de celda ───────────────────────────────────────────
#define CELL_ADC_CONF1          0x0024  ///< Decimación y frecuencia de muestreo del ADC de celda
#define CELL_ADC_CONF2          0x0025  ///< Modo de conversión: single (0x00) o continuo (0x08)
#define AUX_ADC_CONF            0x0026  ///< Configuración del ADC auxiliar (GPIOs/NTCs)
#define ADC_DELAY               0x0027  ///< Retardo entre conversiones ADC
#define GPIO_ADC_CONF           0x0028  ///< Referencia del ADC GPIO: absoluta (0x00) o ratiométrica

// ── Protección hardware de celda (comparadores analógicos) ───────────────────
#define OVUV_CTRL               0x0029  ///< Habilita los comparadores OV/UV por celda (bits 0-5)
#define UV_THRESH               0x002A  ///< Umbral de undervoltage (0x53 = 2.8V)
#define OV_THRESH               0x002B  ///< Umbral de overvoltage (0x5B = 4.3V)
#define OTUT_CTRL               0x002C  ///< Habilita los comparadores OT/UT por GPIO
#define OTUT_THRESH             0x002D  ///< Umbral de OT/UT en porcentaje de TSREF
#define COMP_DG                 0x002E  ///< Deglitch (filtro antirrebote) de los comparadores

// ── Configuración de GPIOs ───────────────────────────────────────────────────
// 0x20 = entrada analógica (para NTC), 0x00 = salida digital
#define GPIO1_CONF              0x002F  ///< Configuración del GPIO1
#define GPIO2_CONF              0x0030  ///< Configuración del GPIO2
#define GPIO3_CONF              0x0031  ///< Configuración del GPIO3
#define GPIO4_CONF              0x0032  ///< Configuración del GPIO4
#define GPIO5_CONF              0x0033  ///< Configuración del GPIO5
#define GPIO6_CONF              0x0034  ///< Configuración del GPIO6

// ── Calibración de ganancia y offset de celdas ───────────────────────────────
#define CELL1_GAIN              0x0035
#define CELL2_GAIN              0x0036
#define CELL3_GAIN              0x0037
#define CELL4_GAIN              0x0038
#define CELL5_GAIN              0x0039
#define CELL6_GAIN              0x003A
#define CELL1_OFF               0x003B
#define CELL2_OFF               0x003C
#define CELL3_OFF               0x003D
#define CELL4_OFF               0x003E
#define CELL5_OFF               0x003F
#define CELL6_OFF               0x0040

// ── CRC de cliente (OTP) ─────────────────────────────────────────────────────
#define CUST_CRCH               0x00C6  ///< Byte alto del CRC de cliente (se actualiza en init)
#define CUST_CRCL               0x00C7  ///< Byte bajo del CRC de cliente

// ── Autoadressing y control del dispositivo ──────────────────────────────────
#define OTP_PROG_UNLOCK1A       0x0100  ///< Código de desbloqueo OTP (A)
#define OTP_PROG_UNLOCK1B       0x0101  ///< Código de desbloqueo OTP (B)
#define OTP_PROG_UNLOCK1C       0x0102  ///< Código de desbloqueo OTP (C)
#define OTP_PROG_UNLOCK1D       0x0103  ///< Código de desbloqueo OTP (D)
#define DEVADD_USR              0x0104  ///< Dirección programable del IC (usada en autoadressing)
#define CONTROL1                0x0105  ///< Control IC (§8.6.1.207): b0=ADD_WRITE_EN(autoaddress) b1=SOFT_RESET(reset a OTP) b2=GOTO_SLEEP b3=GOTO_SHUTDOWN b7=DIR_SEL
#define CONTROL2                0x0106  ///< Control de funciones: bit5=BAL_GO, bit4=TSREF, bit1=AUX_ADC_GO, bit0=CELL_ADC_GO
#define OTP_PROG_CTRL           0x0107  ///< Control de programación OTP
#define GPIO_OUT                0x0108  ///< Control de salidas GPIO digitales

// ── Control del ADC ──────────────────────────────────────────────────────────
#define CELL_ADC_CTRL           0x0109  ///< Habilita el ADC por canal de celda (bits 0-5 = celdas 1-6)
#define AUX_ADC_CTRL1           0x010A  ///< Control ADC auxiliar 1: habilita GPIOs, BAT, REF
#define AUX_ADC_CTRL2           0x010B  ///< Control ADC auxiliar 2
#define AUX_ADC_CTRL3           0x010C  ///< Control ADC auxiliar 3

// ── Control de balanceo de celda (Cell Balancing) ───────────────────────────
// El BQ79606 gestiona el balanceo internamente con timers hardware.
// El MCU solo escribe los registros CB_CELLx_CTRL y activa BAL_GO.
#define CB_CONFIG               0x010D  ///< Config. balanceo: duty cycle, secuencia odds/evens, reacción a fallo
#define CB_CELL1_CTRL           0x010E  ///< Timer de balanceo celda 1 (valor en minutos, 0x00 = desactivado)
#define CB_CELL2_CTRL           0x010F  ///< Timer de balanceo celda 2
#define CB_CELL3_CTRL           0x0110  ///< Timer de balanceo celda 3
#define CB_CELL4_CTRL           0x0111  ///< Timer de balanceo celda 4
#define CB_CELL5_CTRL           0x0112  ///< Timer de balanceo celda 5
#define CB_CELL6_CTRL           0x0113  ///< Timer de balanceo celda 6
#define CB_DONE_THRESHOLD       0x0114  ///< Umbral de voltaje para que el IC detenga el balanceo automáticamente
#define CB_SW_EN                0x0115  ///< Cell Balancing Manual Switch Enable
#define CB_PAUSE_BIT            0x40    ///< bit6 de CB_SW_EN — pausa el balanceo (CB_SW_EN[CB_PAUSE])
                                        ///< NUNCA usar — puede activar celdas adyacentes simultáneamente
                                        ///< y provocar cortocircuito. Usar siempre startBalancing().

// ── Control de diagnóstico ───────────────────────────────────────────────────
#define DIAG_CTRL1              0x0116
#define DIAG_CTRL2              0x0117  ///< Selección de celda para medición diagnóstico con AUX ADC
#define DIAG_CTRL3              0x0118
#define DIAG_CTRL4              0x0119
#define ECC_TEST                0x011D  ///< Registro de test ECC. También usado en autoadressing para sincronizar el DLL.

// ── Registros de estado (solo lectura) ───────────────────────────────────────
#define PARTID                  0x0200  ///< Identificador de revisión del IC
#define SYS_FAULT1              0x0201  ///< Estado de fallos de sistema 1
#define SYS_FAULT2              0x0202  ///< Estado de fallos de sistema 2
#define SYS_FAULT3              0x0203  ///< Estado de fallos de sistema 3
#define DEV_STAT                0x0204  ///< Estado del IC: bit3=CB_RUN (balanceo activo), bit1=DRDY_AUX, bit0=DRDY_CELL
#define LOOP_STAT               0x0205  ///< Estado del round-robin de medición del ADC
#define FAULT_SUM               0x0206  ///< Resumen de fallos activos. Cualquier bit a 1 = hay fallo.

// ── Voltajes de celda corregidos (resultado del ADC de celda) ────────────────
// Se leen con un burst desde VCELL1H. Los datos están en complemento a 2.
// Conversión a voltios: V = complemento_a_2(raw) × 190.73 µV
#define VCELL1H                 0x0215  ///< Voltaje celda 1, byte alto
#define VCELL1L                 0x0216  ///< Voltaje celda 1, byte bajo
#define VCELL2H                 0x0217
#define VCELL2L                 0x0218
#define VCELL3H                 0x0219
#define VCELL3L                 0x021A
#define VCELL4H                 0x021B
#define VCELL4L                 0x021C
#define VCELL5H                 0x021D
#define VCELL5L                 0x021E
#define VCELL6H                 0x021F
#define VCELL6L                 0x0220

// ── Voltajes auxiliares (pila completa, LDOs internos) ───────────────────────
#define AUX_BATH                0x0225  ///< Voltaje total del pack, byte alto
#define AUX_BATL                0x0226  ///< Voltaje total del pack, byte bajo
#define AUX_AVDDH               0x022B  ///< Voltaje LDO AVDD, byte alto
#define AUX_AVDDL               0x022C
#define AUX_CVDDH               0x024D
#define AUX_CVDDL               0x024E
#define AUX_AVAOH               0x024F
#define AUX_AVAOL               0x0250

// ── Voltajes GPIO corregidos (lectura de NTCs) ───────────────────────────────
// Se leen con un burst desde AUX_GPIO1H.
// Conversión a voltios igual que las celdas: complemento_a_2(raw) × 190.73 µV
// Luego se convierte el voltaje a temperatura con la curva del NTC del PCB.
#define AUX_GPIO1H              0x022D  ///< Voltaje GPIO1, byte alto
#define AUX_GPIO1L              0x022E
#define AUX_GPIO2H              0x022F
#define AUX_GPIO2L              0x0230
#define AUX_GPIO3H              0x0231
#define AUX_GPIO3L              0x0232
#define AUX_GPIO4H              0x0233
#define AUX_GPIO4L              0x0234
#define AUX_GPIO5H              0x0235
#define AUX_GPIO5L              0x0236
#define AUX_GPIO6H              0x0237
#define AUX_GPIO6L              0x0238

// ── Estado de balanceo (solo lectura) ────────────────────────────────────────
#define CB_DONE                 0x0261  ///< Estado de finalización del timer de balanceo por celda
#define CB_SW_STAT              0x0282  ///< Estado de los FETs de balanceo: bits 0-5 = celdas 1-6 activas

// ── Registros de fallo detallado (solo lectura) ──────────────────────────────
// Cada bit representa una celda (0-5 = celdas 1-6) o un GPIO (0-5 = GPIO1-6).
#define GPIO_FAULT              0x0290  ///< Fallo en pin GPIO
#define UV_FAULT                0x0291  ///< Undervoltage en celda
#define OV_FAULT                0x0292  ///< Overvoltage en celda
#define UT_FAULT                0x0293  ///< Undertemperature en GPIO/NTC
#define OT_FAULT                0x0294  ///< Overtemperature en GPIO/NTC
#define TONE_FAULT              0x0295  ///< Fallo de tono en el bus daisy-chain
#define COMM_UART_FAULT         0x0296
#define COMM_UART_RC_FAULT      0x0297
#define COMM_UART_RR_FAULT      0x0298
#define COMM_UART_TR_FAULT      0x0299
#define COMM_COMH_FAULT         0x029A
#define COMM_COMH_RC_FAULT      0x029B
#define COMM_COMH_RR_FAULT      0x029C
#define COMM_COMH_TR_FAULT      0x029D
#define COMM_COML_FAULT         0x029E
#define COMM_COML_RC_FAULT      0x029F
#define COMM_COML_RR_FAULT      0x02A0
#define COMM_COML_TR_FAULT      0x02A1
#define OTP_FAULT               0x02A2
#define RAIL_FAULT              0x02A3  ///< Fallo en rail de alimentación interna
#define OVUV_BIST_FAULT         0x02A4  ///< Fallo en el BIST del comparador OV/UV
#define OTUT_BIST_FAULT         0x02A5  ///< Fallo en el BIST del comparador OT/UT

// ── Registros de reset de fallo ──────────────────────────────────────────────
// Escribir 0xFF en estos registros borra el fallo correspondiente.
// Solo tiene efecto si la causa del fallo ya no está presente.
#define GPIO_FLT_RST            0x0127
#define UV_FLT_RST              0x0128
#define OV_FLT_RST              0x0129
#define UT_FLT_RST              0x012A
#define OT_FLT_RST              0x012B
#define TONE_FLT_RST            0x012C
#define COMM_UART_FLT_RST       0x012D
#define COMM_UART_RC_FLT_RST    0x012E
#define COMM_UART_RR_FLT_RST    0x012F
#define COMM_UART_TR_FLT_RST    0x0130
#define COMM_COMH_FLT_RST       0x0131
#define COMM_COMH_RC_FLT_RST    0x0132
#define COMM_COMH_RR_FLT_RST    0x0133
#define COMM_COMH_TR_FLT_RST    0x0134
#define COMM_COML_FLT_RST       0x0135
#define COMM_COML_RC_FLT_RST    0x0136
#define COMM_COML_RR_FLT_RST    0x0137
#define COMM_COML_TR_FLT_RST    0x0138
#define OTP_FLT_RST             0x0139
#define RAIL_FLT_RST            0x013A
#define SYSFLT1_FLT_RST         0x013B
#define SYSFLT2_FLT_RST         0x013C
#define SYSFLT3_FLT_RST         0x013D
#define OVUV_BIST_FLT_RST       0x013E
#define OTUT_BIST_FLT_RST       0x013F

// ── CRC calculado del cliente ────────────────────────────────────────────────
#define CUST_CRC_RSLTH          0x02E1  ///< Resultado CRC de cliente, byte alto (se usa en init)
#define CUST_CRC_RSLTL          0x02E2  ///< Resultado CRC de cliente, byte bajo


// ============================================================================
//  RESULTADO DE OPERACIONES
//  Devuelto por readVoltages(), readTemperatures(), getFaultStatus(),
//  startBalancing() y stopBalancing().
// ============================================================================

/**
 * @brief Resultado de una operación de comunicación con el BQ79606.
 *
 * Uso típico:
 * @code
 *   BQResult res = bms.readVoltages();
 *   if (res == BQResult::OK) { ... }
 *   else if (res == BQResult::CRC_ERROR) { ... }
 * @endcode
 */
enum class BQResult {
    OK,           ///< Operación completada correctamente
    COMM_ERROR,   ///< Error de comunicación: timeout, sin respuesta, bytes insuficientes
    CRC_ERROR     ///< Trama recibida pero CRC inválido — los datos pueden estar corruptos
};


// ============================================================================
//  ESTADO DE FALLOS DE UN BOARD
//  Rellenado por getFaultStatus(board, status).
// ============================================================================

/**
 * @brief Estado completo de fallos de un IC BQ79606.
 *
 * Ejemplo de uso:
 * @code
 *   BQFaultStatus fs;
 *   if (bms.getFaultStatus(0, fs) == BQResult::OK) {
 *       if (fs.hasAnyFault()) {
 *           if (fs.hasOVFault()) { // hay overvoltage en alguna celda
 *               uint8_t cellMask = fs.ovFault; // bit N = celda N+1
 *           }
 *       }
 *   }
 * @endcode
 */
struct BQFaultStatus {

    // ── Resumen ──────────────────────────────────────────────────
    uint8_t summary;    ///< FAULT_SUM (0x0206): cualquier bit a 1 = hay fallo activo

    // ── Fallos de celda ──────────────────────────────────────────
    uint8_t uvFault;    ///< UV_FAULT (0x0291): bit N = celda N+1 por debajo del umbral UV
    uint8_t ovFault;    ///< OV_FAULT (0x0292): bit N = celda N+1 por encima del umbral OV

    // ── Fallos de temperatura (GPIO / NTC) ───────────────────────
    uint8_t utFault;    ///< UT_FAULT (0x0293): bit N = GPIO N+1 por debajo del umbral UT
    uint8_t otFault;    ///< OT_FAULT (0x0294): bit N = GPIO N+1 por encima del umbral OT

    // ── Fallos de sistema interno del IC ─────────────────────────
    uint8_t sysFault1;  ///< SYS_FAULT1 (0x0201)
    uint8_t sysFault2;  ///< SYS_FAULT2 (0x0202)
    uint8_t sysFault3;  ///< SYS_FAULT3 (0x0203)

    // ── Helpers ──────────────────────────────────────────────────
    bool hasAnyFault()     const { return summary || uvFault || ovFault ||
                                          utFault || otFault ||
                                          sysFault1 || sysFault2 || sysFault3; }
    bool hasCellFault()    const { return uvFault || ovFault; }    ///< UV o OV en alguna celda
    bool hasThermalFault() const { return utFault || otFault; }    ///< UT o OT en algún NTC
    bool hasSystemFault()  const { return sysFault1 || sysFault2 || sysFault3; }
    bool hasUVFault()      const { return uvFault != 0; }
    bool hasOVFault()      const { return ovFault != 0; }
    bool hasUTFault()      const { return utFault != 0; }
    bool hasOTFault()      const { return otFault != 0; }
};


// ============================================================================
//  CLASE PRINCIPAL BQ79606
// ============================================================================

class BQ79606 {
public:

    // ─── Constructor ─────────────────────────────────────────────────────────

    /**
     * @brief Constructor. NO toca ningún pin ni periférico.
     *        La inicialización real ocurre en begin().
     *
     * @param config  Struct BQConfig con puerto UART, pines y baudrate del PCB.
     *
     * La clase crea su propia HardwareSerial internamente usando config.uartPort.
     * Esto evita problemas de orden de inicialización de objetos globales en C++,
     * que causaban fallos en ESP32 cuando se pasaba HardwareSerial por referencia.
     */
    explicit BQ79606(const BQConfig& config)
        : _uart(config.pinRx, config.pinTx), _cfg(config) {}
        // STM32: HardwareSerial recibe pines RX/TX en el constructor.
        // ESP32: cambiar a _uart(config.uartPort)


    // ─── Ciclo de vida ────────────────────────────────────────────────────────

    /**
     * @brief Inicializa el BMS: pines, UART, autoadressing y configuración de ADC.
     *
     * Secuencia interna:
     *   1. Configura pines: Wake (HIGH), BmsOk (HIGH=fallo), Fault (INPUT), TxEnable (HIGH).
     *   2. Abre la UART al baudrate configurado.
     *   3. Intenta el autoadressing hasta _maxAttempts veces:
     *      - Genera pulso WAKE
     *      - Hace CommReset para establecer baudrate en toda la cadena
     *      - Asigna dirección secuencial a cada IC
     *      - Verifica que cada IC responde con su dirección (con CRC)
     *   4. Si autoadressing OK, llama a _initDevices():
     *      - Enmascara todos los fallos residuales
     *      - Configura ADC de celda y GPIO
     *      - Configura umbrales UV (2.8V) y OV (4.3V)
     *      - Inicializa registros de balanceo (CB_CONFIG=0xDC, timers=0)
     *
     * @return true si todo OK, false si el autoadressing falló.
     */
    bool begin();

    /**
     * @brief Re-inicializa el BMS sin reiniciar el microcontrolador.
     *
     * Útil para recuperarse de un fallo de comunicación en runtime.
     * Cierra la UART limpiamente antes de volver a begin().
     * Resetea _balHwRunning porque el HW se reinicializa.
     *
     * @return true si OK.
     */
    bool reInit();

    /**
     * @brief Envía la orden de SHUTDOWN a todos los ICs de la cadena.
     *
     * Escribe CONTROL1 bit1=1 mediante broadcast FRMWRT_ALL_NR.
     * Los ICs pasan a bajo consumo hasta recibir un pulso de WAKE.
     */
    void shutdown();


    // ─── Lectura de datos ─────────────────────────────────────────────────────

    /**
     * @brief Lee los 6 voltajes de celda de cada IC y los almacena internamente.
     *
     * Lee los registros VCELL1H–VCELL6L (12 bytes) de cada IC en la cadena,
     * verifica el CRC de cada respuesta y convierte el valor raw a voltios:
     *   V = complemento_a_2(raw) × 190.73 µV
     *
     * ⚠ IMPORTANTE — OCV (Open Circuit Voltage):
     * Si _balHwRunning == true (balanceo activo), los FETs tienen corriente
     * y la tensión medida NO es OCV fiable. La máquina de estados DEBE
     * esperar ≥5s con los FETs apagados (estado SETTLING) antes de usar
     * estas lecturas para calcular timers de balanceo.
     * Usar isVoltageReadingReliable() para verificar.
     *
     * ⚠ NOTA: Esta función NO dispara el ADC si _balHwRunning == true,
     * para no escribir CONTROL2 y apagar accidentalmente BAL_GO (bit5).
     *
     * Tras una lectura exitosa, acceder a datos con:
     *   getVoltage(board, cell)
     *   getMinVoltage() / getMaxVoltage() / getVoltageDelta()
     *
     * @return BQResult::OK, COMM_ERROR o CRC_ERROR.
     */
    BQResult readVoltages();

    /**
     * @brief Lee los 6 canales GPIO de cada IC, los convierte a °C y los almacena.
     *
     * Lee AUX_GPIO1H–AUX_GPIO6L (12 bytes) de cada IC, verifica CRC
     * y convierte voltaje → temperatura usando la curva del NTC del PCB.
     *
     * Al igual que readVoltages(), no toca CONTROL2 si _balHwRunning == true.
     *
     * Tras una lectura exitosa, acceder a datos con:
     *   getTemperature(board, ntc)
     *   getMinTemp() / getMaxTemp()
     *
     * @return BQResult::OK, COMM_ERROR o CRC_ERROR.
     */
    BQResult readTemperatures();


    // ─── Getters de datos ─────────────────────────────────────────────────────

    /**
     * @param board  Índice del IC (0 a TOTALBOARDS-1)
     * @param cell   Índice de celda (0=celda1 ... 5=celda6)
     * @return Voltaje en voltios, o 0.0f si el índice está fuera de rango.
     */
    float getVoltage(uint8_t board, uint8_t cell) const;

    /**
     * @param board  Índice del IC (0 a TOTALBOARDS-1)
     * @param ntc    Índice del NTC/GPIO (0=GPIO1 ... 5=GPIO6)
     * @return Temperatura en °C, o 0.0f si el índice está fuera de rango.
     */
    float getTemperature(uint8_t board, uint8_t ntc) const;

    // Estadísticas globales — se calculan automáticamente en cada readVoltages/readTemperatures
    float getMinVoltage()   const { return _vMin; }                       ///< Voltaje mínimo de todas las celdas (V)
    float getMaxVoltage()   const { return _vMax; }                       ///< Voltaje máximo de todas las celdas (V)
    float getVoltageDelta() const { return (_vMax - _vMin) * 1000.0f; }  ///< Diferencia max-min en milivoltios (mV)
    float getMinTemp()      const { return _tMin; }                       ///< Temperatura mínima de todos los NTCs válidos (°C)
    float getMaxTemp()      const { return _tMax; }                       ///< Temperatura máxima de todos los NTCs válidos (°C)
    bool  hasOpenNtc()      const { return _ntcOpenCount > 0; }           ///< true si algún NTC configurado está abierto/inválido
    uint8_t getOpenNtcCount() const { return _ntcOpenCount; }             ///< Nº de NTCs configurados con lectura inválida


    // ─── Estado ───────────────────────────────────────────────────────────────

    /** @return true si begin() o reInit() completaron el autoadressing correctamente. */
    bool isOK() const { return _initialized; }

    /**
     * @brief Lee el pin FAULT hardware del BQ79606.
     *
     * El pin FAULT es activo en LOW según el datasheet.
     * @return true = sin fallo en el pin, false = hay fallo hardware activo.
     */
    bool getFaultPin() const;


    // ─── Lectura y reset de fallos ────────────────────────────────────────────

    /**
     * @brief Lee los registros de fallo de un IC y los devuelve en una struct.
     *
     * Hace dos lecturas burst con verificación de CRC:
     *   1. SYS_FAULT1 → FAULT_SUM (6 bytes): fallos de sistema y resumen.
     *   2. UV_FAULT → OT_FAULT (4 bytes): fallos de celda y temperatura.
     *
     * @param board   Índice del IC (0 a TOTALBOARDS-1)
     * @param status  Struct BQFaultStatus donde se escriben los resultados.
     * @return BQResult::OK, COMM_ERROR o CRC_ERROR.
     */
    BQResult getFaultStatus(uint8_t board, BQFaultStatus& status);

    /**
     * @brief Borra todos los registros de fallo en todos los ICs.
     *
     * Escribe 0xFF en todos los registros _FLT_RST mediante broadcast.
     * Solo borrar cuando la causa del fallo ya esté resuelta.
     */
    void clearAllFaults();


    // ─── Vigilancia de seguridad (lectura normal) ─────────────────────────────

    /**
     * @brief Fija los límites de seguridad. Llamar una vez tras begin().
     * Si no se llama, se usan los valores por defecto del driver.
     */
    void setSafetyLimits(const BQSafetyLimits& lim) { _safetyLim = lim; }

    /**
     * @brief Evalúa OV/UV con los voltajes ya leídos (_vMax/_vMin).
     *
     * Llamar tras un readVoltages() OK. NO evalúa si el balanceo HW está
     * activo (voltajes no fiables por el sag de los FETs) ni si el latch
     * ya está disparado. Aplica debounce vDebounceMs; si confirma →
     * trip (BMS_OK LOW + latch).
     */
    void evalSafetyVoltage();

    /**
     * @brief Evalúa OT/UT con las temperaturas ya leídas (_tMax/_tMin).
     * Llamar tras un readTemperatures() OK. Debounce tDebounceMs.
     */
    void evalSafetyTemperature();

    /** @return true si la seguridad está disparada y enclavada. */
    bool isSafetyTripped() const { return _safetyLatched; }

    /**
     * @brief Limpia el latch de seguridad y, si el driver está OK, vuelve
     * a poner BMS_OK en LOW (OK). Llamar solo cuando la causa esté resuelta.
     * Reanuda la vigilancia.
     */
    void clearSafetyLatch();

    /**
     * @brief Controla el pin BMS_OK hacia el SDC (LOW = OK, HIGH = fallo).
     * Escribe solo en los cambios de estado.
     */
    void setBmsOk(bool ok);


    // ─── Balanceo ─────────────────────────────────────────────────────────────

    /**
     * @brief Activa el balanceo hardware en las celdas indicadas de un IC.
     *
     * Secuencia de escritura (según SLVA970E):
     *   1. Valida cellMask — rechaza si tiene bits fuera de rango (>0x3F).
     *   2. CB_CONFIG = 0xDC → duty cycle 120s, alternancia odd/even, continuar en fallo.
     *      La alternancia hardware garantiza que el BQ nunca activa celdas
     *      adyacentes simultáneamente — alterna entre odds (1,3,5) y evens (2,4,6).
     *   3. CB_CELLx_CTRL = timerVal (para celdas en cellMask) o 0x00 (para el resto).
     *   4. CONTROL2 = 0x30 → BAL_GO (bit5) + TSREF (bit4).
     *
     * ⚠ La seguridad contra cortocircuito la garantiza CB_CONFIG = 0xDC.
     *   Cualquier máscara de 0x00 a 0x3F es segura con este registro.
     *   NUNCA usar CB_SW_EN (0x0115) directamente — activa los FETs sin
     *   alternancia y puede causar cortocircuito.
     *
     * ⚠ Llamar siempre desde el estado MEASURING de la máquina de estados,
     *   tras un settling de ≥5s con los FETs apagados (OCV fiable).
     *
     * @param board     Índice del IC (0 a TOTALBOARDS-1)
     * @param cellMask  Máscara de celdas. bit0=celda1 … bit5=celda6.
     *                  Rango válido: 0x00–0x3F.
     *                  Ejemplos: 0x3F = todas, 0x15 = celdas 1,3,5, 0x2A = celdas 2,4,6.
     * @param timerVal  Duración del balanceo en minutos (1–127). Default = 3 min.
     * @return BQResult::OK o COMM_ERROR.
     */
    BQResult startBalancing(uint8_t board, uint8_t cellMask, uint8_t timerVal = 0x03);

    /**
     * @brief Arranca el balanceo con timers individuales por celda.
     * @param board   Índice del IC.
     * @param timers  Array de 6 valores (0=no balancear, 1-127 minutos).
     */
    BQResult startBalancingTimers(uint8_t board, const uint8_t timers[6]);

    /**
     * @brief Actualiza los timers de balanceo de un IC durante el descanso entre fases.
     *
     * Según el datasheet, CB_CELLx_CTRL no se puede modificar mientras
     * CONTROL2[BAL_GO]=1. Sin embargo durante el descanso entre fases
     * (CB_SW_STAT=0, FETs apagados) el IC acepta escrituras a los timers.
     * Permite recalcular y ajustar los tiempos restantes sin parar el balanceo.
     *
     * ⚠ Llamar solo cuando isBalancing(board)==false (fase de descanso).
     *
     * @param board   Índice del IC.
     * @param timers  Array de 6 valores de timer (0=desactivar, 1-127 minutos).
     * @return BQResult::OK o COMM_ERROR.
     */
    BQResult updateBalancingTimers(uint8_t board, const uint8_t timers[6]);

    /**
     * @brief Configura CB_CONFIG forzando CB_LOOP=1 y CB_SEQ=1 siempre activos.
     * Solo los bits [7:4] (tiempo de fase) son configurables externamente.
     * @param val  Valor deseado — los bits [3:0] se sobreescriben con 0x0C.
     */
    void setCBConfig(uint8_t val);

    /**
     * @brief Para el balanceo en un IC específico.
     *
     * Pone todos los timers CB_CELLx_CTRL a 0x00 y escribe
     * CONTROL2 = 0x10 (solo TSREF activo, BAL_GO apagado).
     * Pone _balHwRunning = false.
     *
     * Para parar todos los ICs a la vez, usar stopAllBalancing().
     *
     * @param board  Índice del IC.
     * @return BQResult::OK o COMM_ERROR.
     */
    BQResult stopBalancing(uint8_t board);

    /**
     * @brief Para el balanceo en TODOS los ICs simultáneamente.
     *
     * Usa broadcast FRMWRT_ALL_NR — más eficiente que stopBalancing() en bucle.
     * Llamar en emergencias: fallo, shutdown, timeout de balanceo.
     * Pone _balHwRunning = false.
     */
    void stopAllBalancing();

    /**
     * @brief Comprueba si algún FET de balanceo está cerrado (CB_SW_STAT).
     *
     * Lee CB_SW_STAT (0x0282), bits 0-5 = celdas 1-6 activas.
     * Indica el estado real del hardware en este momento.
     * Distinto de isBalancingDone(): ese indica si el TIMER terminó.
     *
     * @param board  Índice del IC.
     * @return true si algún FET de balanceo está activo.
     */
    bool isBalancing(uint8_t board);

    /**
     * @brief Comprueba si el FET de balanceo de una celda concreta está cerrado.
     *
     * Lee CB_SW_STAT bit (cell) para saber si esa celda está en la fase activa
     * (odd o even) en este momento. Útil para display en tiempo real.
     *
     * @param board  Índice del IC (0..TOTALBOARDS-1).
     * @param cell   Índice de celda (0..5, donde 0=celda1).
     * @return true si el FET de esa celda está activo ahora mismo.
     */
    bool isCellBalancing(uint8_t board, uint8_t cell);

    /**
     * @brief Devuelve la máscara de FETs activos de un board (CB_SW_STAT bits 0-5).
     * bit0=C1, bit1=C2, ... bit5=C6. 0x00 = ninguno activo.
     * Útil para detectar cambio de fase odd→even comparando con la lectura anterior.
     */
    uint8_t getBalancingMask(uint8_t board);

    /**
     * @brief Comprueba si el timer de balanceo terminó (registro CB_DONE 0x0261).
     *
     * Cuando todos los ICs devuelven CB_DONE = 1, la máquina de estados
     * debe parar el HW (stopAllBalancing) y volver al estado SETTLING
     * para medir el nuevo OCV tras el reposo.
     *
     * @param board  Índice del IC.
     * @return true si el timer de balanceo terminó en este IC.
     */
    bool isBalancingDone(uint8_t board);

    /**
     * @brief Indica si BAL_GO está activo en hardware.
     *
     * readVoltages() y readTemperatures() consultan este flag
     * para no tocar CONTROL2 y apagar BAL_GO accidentalmente.
     */
    bool isBalHwRunning() const { return _balHwRunning; }

    /**
     * @brief Indica si los voltajes medidos son OCV fiable.
     *
     * Devuelve false cuando el balanceo hardware está activo:
     * los FETs tienen corriente y la tensión medida no es la real de la celda.
     * La máquina de estados debe esperar ≥5s (SETTLING) antes de usar
     * readVoltages() para calcular timers de balanceo.
     */
    bool isVoltageReadingReliable() const { return !_balHwRunning; }


    // ─── Comunicación de bajo nivel ───────────────────────────────────────────
    // Públicos para permitir debug y acceso avanzado desde el main.

    /**
     * @brief Escribe datos en un registro de un IC de la cadena.
     *
     * @param id    Dirección del IC (0 a TOTALBOARDS-1). 0 con broadcast.
     * @param addr  Dirección del registro destino.
     * @param data  Valor a escribir (hasta 8 bytes, formato big-endian).
     * @param len   Número de bytes a escribir (1–8).
     * @param type  Tipo de frame: FRMWRT_SGL_NR, FRMWRT_ALL_NR, etc.
     * @return Bytes enviados, o ≤0 si error.
     */
    int writeReg(byte id, uint16_t addr, uint64_t data, byte len, byte type);

    /**
     * @brief Lee datos de un registro de un IC de la cadena.
     *
     * Envía la petición de lectura y espera la respuesta.
     * El buffer recibido incluye: 1 byte init + 1 byte addr_IC +
     * 2 bytes reg_addr + N bytes datos + 2 bytes CRC.
     * Los datos útiles empiezan en buf[4].
     *
     * @param id       Dirección del IC.
     * @param addr     Dirección del registro de inicio.
     * @param buf      Buffer donde se almacena la respuesta completa.
     * @param len      Número de bytes de datos a leer.
     * @param timeout  Timeout en ms (0 = usar default de 10ms).
     * @param type     Tipo de frame: FRMWRT_SGL_R, FRMWRT_ALL_R, etc.
     * @return Bytes leídos, o ≤0 si error.
     */
    int readReg(byte id, uint16_t addr, byte* buf, byte len, uint32_t timeout, byte type);

    /**
     * @brief Verifica el CRC-16 de una trama recibida.
     *
     * El CRC se encuentra en los últimos 2 bytes del buffer en little-endian
     * (byte bajo en posición len-2, byte alto en posición len-1).
     *
     * @param data  Buffer completo incluyendo datos y CRC.
     * @param len   Longitud total del buffer.
     * @return true si el CRC calculado coincide con el recibido.
     */
    bool checkCRC(uint8_t* data, uint16_t len);


    // ─── Debug ────────────────────────────────────────────────────────────────

    /** Imprime por Serial los registros 0x0200–0x0250 de un IC (con nombre de registro). */
    void printRegisters(uint8_t deviceID);

    /** Imprime por Serial el valor hexadecimal de un único registro de un IC. */
    void printRegister(uint8_t deviceID, uint16_t addr);


    // ─── Configuración ────────────────────────────────────────────────────────

    /**
     * @brief Establece el número máximo de intentos de autoadressing.
     *        Llamar antes de begin() si se quiere cambiar el default.
     * @param n  Número de intentos (default: 5).
     */
    void setMaxAttempts(uint8_t n) { _maxAttempts = n; }


private:

    // ─── Hardware ─────────────────────────────────────────────────────────────
    HardwareSerial  _uart;  ///< UART propia de la clase (creada con config.uartPort)
    BQConfig        _cfg;   ///< Pines y baudrate del PCB

    // ─── Buffers internos del protocolo ──────────────────────────────────────
    uint8_t _pFrame[(MAXBYTES + 6) * TOTALBOARDS]; ///< Buffer de construcción de tramas de envío
    byte    _bBuf[8];        ///< Buffer temporal para empaquetar hasta 8 bytes de datos
    byte    _bReturn;        ///< Byte "bytesToReturn - 1" usado en ReadFrameReq
    byte    _bFrame[(2 + 6) * TOTALBOARDS]; ///< Buffer de tramas cortas (lecturas de diagnóstico)
    uint8_t _nCurrentBoard;  ///< Índice de IC actual en bucles de inicialización y autoadressing

    // ─── Datos medidos ────────────────────────────────────────────────────────
    float _voltages[TOTALBOARDS][6]; ///< Voltajes en V: [índice IC][celda 0..5]
    float _temps[TOTALBOARDS][6];    ///< Temperaturas en °C: [índice IC][NTC 0..5]

    // ─── Estadísticas ─────────────────────────────────────────────────────────
    float _vMin = 99.0f,   _vMax = 0.0f;    ///< Voltaje min/max global de todas las celdas
    float _tMin = 999.0f,  _tMax = -999.0f; ///< Temperatura min/max global de todos los NTCs

    // ─── Estado interno ───────────────────────────────────────────────────────
    bool    _initialized  = false; ///< true tras un begin() o reInit() exitoso
    bool    _balHwRunning = false; ///< true mientras BAL_GO esté activo (CONTROL2 bit5)
    uint8_t _maxAttempts  = 5;     ///< Máximo intentos de autoadressing antes de declarar fallo

    // ─── Vigilancia de seguridad ──────────────────────────────────────────────
    BQSafetyLimits _safetyLim = { 4.15f, 3.65f, 60.0f, 10.0f, 500, 1000 };
    uint32_t _ovSince = 0, _uvSince = 0;  ///< millis() en que empezó OV/UV (0 = en rango)
    uint32_t _otSince = 0, _utSince = 0;  ///< millis() en que empezó OT/UT (0 = en rango)
    uint32_t _ntcOpenSince = 0;           ///< millis() en que empezó la condición NTC abierto (0 = ok)
    uint8_t  _ntcOpenCount = 0;           ///< Nº de NTCs configurados con lectura inválida (último readTemperatures)
    int8_t   _ntcOpenBoard = -1;          ///< Primer board con NTC abierto (para el mensaje)
    int8_t   _ntcOpenCell  = -1;          ///< Primer NTC abierto en ese board (índice 0..5)
    bool     _safetyLatched = false;      ///< true tras un trip confirmado; sticky hasta clear
    bool     _bmsOkState    = false;      ///< Último estado escrito en pinBmsOk

    bool _debounce(bool outOfRange, uint32_t& since, uint32_t windowMs);
    void _safetyTrip(const char* reason, float val, float lim);

    // ─── Funciones privadas del protocolo ─────────────────────────────────────
    // Implementan la capa de transporte del protocolo daisy-chain BQ79606.
    // No deben llamarse desde fuera de la clase.

    void     _wakeUp();               ///< Genera el pulso de WAKE (LOW 300µs → HIGH)
    void     _commClear();            ///< Pone TX a LOW el tiempo de 17 bits para limpiar el bus
    void     _commSleepToWake();      ///< Transición de sleep a activo
    void     _commReset(int baud);    ///< Reset del bus + negociación de baudrate con todos los ICs
    bool     _autoAddress();          ///< Secuencia completa de autoadressing con verificación CRC
    void     _initDevices();          ///< Configuración inicial de ADC, protecciones y balanceo

    int      _writeFrame(byte id, uint16_t addr, byte* data, byte len, byte type); ///< Construye y envía una trama completa con CRC
    int      _readFrameReq(byte id, uint16_t addr, byte bytesToReturn, byte type); ///< Envía la petición de lectura y limpia el buffer
    uint16_t _crc16(byte* buf, int len);            ///< Calcula CRC-16 ITU-T (tabla de 256 entradas)
    float    _complement(uint16_t raw, float mult); ///< Convierte valor ADC raw (complemento a 2) a voltios
    float    _voltToTemp(float voltage);            ///< Convierte voltaje de NTC a temperatura en °C (curva del PCB)

    void     _updateVoltageStats(); ///< Recalcula _vMin y _vMax tras readVoltages()
    void     _updateTempStats();    ///< Recalcula _tMin y _tMax tras readTemperatures()
};

#endif // BQ79606_H