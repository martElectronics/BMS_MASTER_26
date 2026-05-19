#ifndef BQ79606_REGISTERS_H
#define BQ79606_REGISTERS_H

// ============================================================
// FRAME WRITE TYPES
// ============================================================
#define FRMWRT_SGL_R        0x00 // single device read
#define FRMWRT_SGL_NR       0x10 // single device write
#define FRMWRT_STK_R        0x20 // stack read
#define FRMWRT_STK_NR       0x30 // stack write
#define FRMWRT_ALL_R        0x40 // broadcast read
#define FRMWRT_ALL_NR       0x50 // broadcast write
#define FRMWRT_REV_ALL_NR   0xE0 // reverse broadcast write

// ============================================================
// REGISTER MAP (COMPLETO)
// ============================================================

// Device addressing
#define DEVADD_OTP          0x0000
#define CONFIG              0x0001
#define GPIO_FLT_MSK        0x0002
#define UV_FLT_MSK          0x0003
#define OV_FLT_MSK          0x0004
#define UT_FLT_MSK          0x0005
#define OT_FLT_MSK          0x0006
#define TONE_FLT_MSK        0x0007
#define COMM_UART_FLT_MSK   0x0008
#define COMM_UART_RC_FLT_MSK 0x0009
#define COMM_UART_RR_FLT_MSK 0x000A
#define COMM_UART_TR_FLT_MSK 0x000B
#define COMM_COMH_FLT_MSK   0x000C
#define COMM_COMH_RC_FLT_MSK 0x000D
#define COMM_COMH_RR_FLT_MSK 0x000E
#define COMM_COMH_TR_FLT_MSK 0x000F
#define COMM_COML_FLT_MSK   0x0010
#define COMM_COML_RC_FLT_MSK 0x0011
#define COMM_COML_RR_FLT_MSK 0x0012
#define COMM_COML_TR_FLT_MSK 0x0013
#define OTP_FLT_MSK         0x0014
#define RAIL_FLT_MSK        0x0015
#define SYSFLT1_FLT_MSK     0x0016
#define SYSFLT2_FLT_MSK     0x0017
#define SYSFLT3_FLT_MSK     0x0018
#define OVUV_BIST_FLT_MSK   0x0019
#define OTUT_BIST_FLT_MSK   0x001A

// Communication control
#define COMM_CTRL           0x0020
#define DAISY_CHAIN_CTRL    0x0021
#define TX_HOLD_OFF         0x0022
#define COMM_TO             0x0023

// ADC configuration
#define CELL_ADC_CONF1      0x0024
#define CELL_ADC_CONF2      0x0025
#define AUX_ADC_CONF        0x0026
#define ADC_DELAY           0x0027
#define GPIO_ADC_CONF       0x0028

// Cell protection
#define OVUV_CTRL           0x0029
#define UV_THRESH           0x002A
#define OV_THRESH           0x002B

// GPIO configuration
#define GPIO1_CONF          0x002F
#define GPIO2_CONF          0x0030
#define GPIO3_CONF          0x0031
#define GPIO4_CONF          0x0032
#define GPIO5_CONF          0x0033
#define GPIO6_CONF          0x0034

// Calibration registers (gain/offset)
#define CELL1_GAIN          0x0035
#define CELL2_GAIN          0x0036
#define CELL3_GAIN          0x0037
#define CELL4_GAIN          0x0038
#define CELL5_GAIN          0x0039
#define CELL6_GAIN          0x003A

#define CELL1_OFF           0x003B
#define CELL2_OFF           0x003C
#define CELL3_OFF           0x003D
#define CELL4_OFF           0x003E
#define CELL5_OFF           0x003F
#define CELL6_OFF           0x0040

// User programmable address
#define DEVADD_USR          0x0104

// Control registers
#define CONTROL1            0x0105
#define CONTROL2            0x0106

// GPIO output
#define GPIO_OUT            0x0108

// Cell ADC control
#define CELL_ADC_CTRL       0x0109

// AUX ADC control
#define AUX_ADC_CTRL1       0x010A
#define AUX_ADC_CTRL2       0x010B
#define AUX_ADC_CTRL3       0x010C

// Balancing
#define CB_CONFIG           0x010D
#define CB_CELL1_CTRL       0x010E
#define CB_CELL2_CTRL       0x010F
#define CB_CELL3_CTRL       0x0110
#define CB_CELL4_CTRL       0x0111
#define CB_CELL5_CTRL       0x0112
#define CB_CELL6_CTRL       0x0113

// Diagnostic
#define DIAG_CTRL1          0x0116
#define DIAG_CTRL2          0x0117
#define DIAG_CTRL3          0x0118
#define DIAG_CTRL4          0x0119

// ECC test
#define ECC_TEST            0x011D

// Status registers
#define PARTID              0x0200
#define SYS_FAULT1          0x0201
#define SYS_FAULT2          0x0202
#define SYS_FAULT3          0x0203
#define DEV_STAT            0x0204
#define LOOP_STAT           0x0205
#define FAULT_SUM           0x0206
#define CUST_CRC_RSLTH      0x2E04
#define CUST_CRCH           0x2E06
#define CUST_CRCL           0x2E07


// Cell voltages (filtered)
#define VCELL1_HF           0x0207
#define VCELL1_LF           0x0208
#define VCELL2_HF           0x0209
#define VCELL2_LF           0x020A
#define VCELL3_HF           0x020B
#define VCELL3_LF           0x020C
#define VCELL4_HF           0x020D
#define VCELL4_LF           0x020E
#define VCELL5_HF           0x020F
#define VCELL5_LF           0x0210
#define VCELL6_HF           0x0211
#define VCELL6_LF           0x0212

// AUX voltages
#define AUX_CELLH           0x0223
#define AUX_CELLL           0x0224

// Temperature
#define DIE_TEMPH           0x023B
#define DIE_TEMPL           0x023C

// GPIO voltages
#define AUX_GPIO1H          0x022D
#define AUX_GPIO1L          0x022E
#define AUX_GPIO2H          0x022F
#define AUX_GPIO2L          0x0230
#define AUX_GPIO3H          0x0231
#define AUX_GPIO3L          0x0232
#define AUX_GPIO4H          0x0233
#define AUX_GPIO4L          0x0234
#define AUX_GPIO5H          0x0235
#define AUX_GPIO5L          0x0236
#define AUX_GPIO6H          0x0237
#define AUX_GPIO6L          0x0238

#endif
