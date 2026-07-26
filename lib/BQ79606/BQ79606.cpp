/**
 * @file    BQ79606.cpp
 * @brief   Implementación del driver BQ79606A-Q1.
 *
 * Contiene toda la lógica de comunicación daisy-chain:
 *   - Construcción y envío de tramas (_writeFrame)
 *   - Recepción y verificación de tramas (readReg + CRC)
 *   - Autoadressing y configuración inicial de los ICs (_autoAddress, _initDevices)
 *   - Lectura de voltajes y temperaturas (readVoltages, readTemperatures)
 *   - Gestión de fallos (getFaultStatus, clearAllFaults)
 *   - Control de balanceo (startBalancing, stopBalancing, isBalancingDone)
 *
 * ── FORMATO DE TRAMA (SLVA970E sección 3) ───────────────────────────────────
 * Trama de escritura:
 *   [INIT_BYTE | DEV_ADDR | REG_ADDR_H | REG_ADDR_L | DATA... | CRC_L | CRC_H]
 *
 * Trama de respuesta (lectura):
 *   [RESP_INIT | DEV_ADDR | REG_ADDR_H | REG_ADDR_L | DATA... | CRC_L | CRC_H]
 *   Los datos útiles empiezan siempre en el byte [4] del buffer recibido.
 *
 * ── CRC-16 ITU-T ─────────────────────────────────────────────────────────────
 * Polinomio: x^16 + x^15 + x^2 + 1 (0xA001 en forma refleja).
 * Valor inicial: 0xFFFF.
 * Los 2 bytes de CRC van al final de la trama en little-endian (L primero, H después).
 */

#include "BQ79606.h"
#include <HardwareSerial.h>

/**
 * Tabla de lookup para CRC-16 ITU-T (CRC-16-IBM).
 * Generada con el polinomio reflejo 0xA001.
 * Es la misma que usa TI en su código de referencia (SLVA970E).
 * Se declara static const para que viva en flash, no en RAM.
 */
static const uint16_t crc16_table[256] = {
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
    0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
    0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
    0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
    0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
    0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
    0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
    0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
    0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
    0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
    0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
    0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
    0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
    0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
    0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
    0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
    0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
    0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
    0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
    0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
    0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
    0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
    0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
    0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
    0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
    0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
    0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
    0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
    0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
    0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
    0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

// ============================================================
//  CICLO DE VIDA
// ============================================================

bool BQ79606::begin(bool keepBmsOk)
{
    _initialized = false;

    // Arranque limpio de la vigilancia de seguridad (cubre tambien reInit()).
    _safetyLatched = false;
    _ovSince = _uvSince = _otSince = _utSince = 0;
    _ntcOpenSince = 0;

    // Solo pines que pertenecen al BQ79606:
    // Wake, Fault y BMS_OK son señales directas del IC.
    // PWM_FANS, AMP_PIN, etc. son responsabilidad del main.
    pinMode(BQ_DPIN(_cfg.pinWake),  OUTPUT); digitalWrite(BQ_DPIN(_cfg.pinWake),  HIGH);
    pinMode(BQ_DPIN(_cfg.pinFault), INPUT);
    pinMode(BQ_DPIN(_cfg.pinBmsOk), OUTPUT);

    // BMS_OK — ver keepBmsOk en el .h.
    //   Arranque en FRIO (keepBmsOk=false): LOW (fallo) hasta que el init
    //     confirme que hay cadena. Fail-safe: no se afirma "OK" sin datos.
    //   Reconexión en CALIENTE (keepBmsOk=true, camino de reInit()): el pin es
    //     del debounce de la aplicación, NO de esta función. Un reInit() es un
    //     intento de RECUPERACIÓN, no un veredicto de seguridad: si bajara el
    //     pin aquí, la ventana de gracia por fallo de comms del main/charger
    //     quedaría anulada (el pin caería al primer error, no al expirar).
    if (!keepBmsOk) {
        digitalWrite(BQ_DPIN(_cfg.pinBmsOk), LOW);
        _bmsOkState = false;
    } else {
        // Re-afirmar el nivel que ya tenía (pinMode no lo cambia, pero lo
        // dejamos explícito por si el pin venía de otra configuración).
        digitalWrite(BQ_DPIN(_cfg.pinBmsOk), _bmsOkState ? HIGH : LOW);
    }

    // Level shifter TX enable — solo si está configurado
    if (_cfg.pinTxEnable >= 0) {
        pinMode(BQ_DPIN((BQPin)_cfg.pinTxEnable), OUTPUT);
        digitalWrite(BQ_DPIN((BQPin)_cfg.pinTxEnable), HIGH);
    }

    // Abrir UART
    _uart.begin(_cfg.baudrate);
    delay(20);

    bool ok = false;
    for (uint8_t attempt = 0; attempt < _maxAttempts && !ok; attempt++) {
        { char _b[128]; snprintf(_b, sizeof(_b), "  AutoAddress intento %d/%d\n", attempt + 1, _maxAttempts); Serial.print(_b); }

        _wakeUp();
        delay(200);
        _commReset(_cfg.baudrate);
        delay(200);
        ok = _autoAddress();
        delay(200);
    }

    if (!ok) {
        Serial.println("[BQ] AutoAddress fallido.");
        return false;
    }

    _initDevices();

    // Limpieza de fallos internos tras autoaddress + configuracion.
    // La secuencia wake / comm-reset / autoaddress y la propia
    // reconfiguracion de _initDevices() generan faults espurios
    // (power-on, comm). Se limpian UNA vez aqui, ya con thresholds y
    // ADC/OVUV activos: cualquier fault real se vuelve a latchear en el
    // siguiente ciclo de ADC y queda visible via getFaultStatus().
    // No se enmascara seguridad: solo se elimina el ruido de arranque.
    delay(5);
    clearAllFaults();

    _initialized = true;
    // Solo en frío se afirma BMS_OK=HIGH desde aquí. En caliente NO: la
    // aplicación decide con su debounce (puede haber un fallo de V/T/NTC vivo
    // que este init no conoce; forzar HIGH lo borraría durante un ciclo).
    if (!keepBmsOk) setBmsOk(true);
    Serial.println("[BQ] Inicializado OK (faults de arranque limpiados).");
    return true;
}

bool BQ79606::reInit(bool keepBmsOk)
{
    _initialized  = false;
    _balHwRunning = false;  // el HW se va a resetear — flag siempre a false
    _uart.end();
    delay(10);
    return begin(keepBmsOk);
}

void BQ79606::shutdown()
{
    // CONTROL1 b3 = GOTO_SHUTDOWN (0x08) -> "Transitions device to SHUTDOWN mode"
    // (§8.6.1.207). El antiguo 0x02 era b1=SOFT_RESET (reset a OTP defaults:
    // borraba config OVUV/OTUT y dejaba la cadena sin direccionar), NO shutdown.
    writeReg(0x00, CONTROL1, 0x08, 1, FRMWRT_ALL_NR);
}

// ============================================================
//  LECTURA DE DATOS
// ============================================================

// Lee un board por single-read con reintentos ante ruido. En cuanto una
// respuesta pasa CRC, deja el frame válido en buf y devuelve OK. Si agota
// BQ_READ_ATTEMPTS, devuelve COMM_ERROR (no respondió nunca) o CRC_ERROR
// (respondió pero siempre corrupto) — misma semántica que el camino anterior,
// solo que ahora un glitch aislado se re-pide en vez de tumbar la lectura.
BQResult BQ79606::_readBoardRetry(byte board, uint16_t addr, byte* buf,
                                  size_t bufSize, byte len)
{
    BQResult last = BQResult::COMM_ERROR;
    for (uint8_t attempt = 0; attempt < BQ_READ_ATTEMPTS; attempt++) {
        memset(buf, 0, bufSize);
        int res = _readRegImpl(board, addr, buf, bufSize, len, 0, FRMWRT_SGL_R);
        if (res <= 0)            { last = BQResult::COMM_ERROR; continue; }
        if (!checkCRC(buf, res)) { last = BQResult::CRC_ERROR;  continue; }
        return BQResult::OK;     // frame válido: buf ya tiene los datos buenos
    }
    return last;                 // agotados los reintentos: último tipo de fallo
}

BQResult BQ79606::readVoltages()
{
    byte buf[MAXBYTES + 6];

    // NOTA OCV: si _balHwRunning==true, los voltajes NO son OCV fiable.
    // Hay corriente circulando por los FETs de balanceo y la tensión
    // medida está perturbada. Solo usar estas lecturas para monitorización,
    // nunca para calcular timers de balanceo.
    // La máquina de estados debe garantizar ≥5s de reposo (SETTLING)
    // antes de usar readVoltages() para decisiones de balanceo.

    // Solo disparar conversión ADC si el balanceo HW NO está activo.
    if (!_balHwRunning) {
        writeReg(0, CONTROL2, 0x13, 1, FRMWRT_ALL_NR);
        delay(10);
    }

    for (uint8_t board = 0; board < TOTALBOARDS; board++) {
        BQResult r = _readBoardRetry(board, VCELL1H, buf, sizeof(buf), MAXBYTES);
        if (r != BQResult::OK) { _lastReadFailBoard = board; return r; }

        for (uint8_t c = 0; c < 6; c++) {
            uint16_t raw = ((uint16_t)buf[4 + c * 2] << 8) | buf[5 + c * 2];
            _voltages[board][c] = _complement(raw, 0.00019073f);
        }
    }

    _updateVoltageStats();
    return BQResult::OK;
}

BQResult BQ79606::readTemperatures()
{
    byte buf[MAXBYTES + 6];

    // Disparo del AUX ADC SIEMPRE, también durante el balanceo:
    //  - El AUX ADC es solo on-demand (datasheet §8.3.4.4.1: "does not support
    //    continuous conversion"): hay que escribir CONTROL2[AUX_ADC_GO] cada
    //    vez. No existe modo continuo/free-run para los NTC.
    //  - CONTROL2=0x13 con BAL_GO(B5)=0 es SEGURO durante balanceo: BAL_GO
    //    "always reads 0" y es un disparo momentáneo (§8.6.1.208 CONTROL2);
    //    escribir 0 es no-op → NO para ni reinicia el balanceo, y no existe
    //    bit stop/pause de CB en CONTROL2 (CB_PAUSE está en CB_SW_EN).
    //  - Los NTC van por AUX_GPIO, ruta distinta al FET de balanceo → la
    //    temperatura SÍ es válida durante el balanceo (a diferencia del
    //    voltaje, que sagea — por eso readVoltages() sí mantiene el guard).
    //  - 0x13 mantiene TSREF(B4) y dispara CELL_ADC_GO(B0): §8.3.4.3 exige
    //    una conversión CELL junto a la AUX para refrescar la corrección de
    //    temperatura de die (el resultado CELL en balanceo es basura pero no
    //    se lee aquí; solo importa para esa corrección).
    {
        // Mapeado real confirmado por barrido bit a bit:
        //   CTRL1 bit4 (0x10) → GPIO1
        //   CTRL1 bit5 (0x20) → GPIO2
        //   CTRL1 bit6 (0x40) → GPIO3
        //   CTRL1 bit7 (0x80) → GPIO4
        //   CTRL2 bit0 (0x01) → GPIO5
        //   CTRL2 bit1 (0x02) → GPIO6
        // Configurar AUX_ADC por board según NTCs conectados.
        // Board par  (GPIO1-6): CTRL1=0xF0, CTRL2=0x03
        // Board impar (GPIO1-3): CTRL1=0x70, CTRL2=0x00
        writeReg(0, AUX_ADC_CTRL1, 0xF0, 1, FRMWRT_ALL_NR);
        writeReg(0, AUX_ADC_CTRL2, 0x03, 1, FRMWRT_ALL_NR);
        writeReg(0, AUX_ADC_CTRL3, 0x00, 1, FRMWRT_ALL_NR);
        for (uint8_t b = 1; b < TOTALBOARDS; b += 2) {
            writeReg(b, AUX_ADC_CTRL1, 0x70, 1, FRMWRT_SGL_NR);
            writeReg(b, AUX_ADC_CTRL2, 0x00, 1, FRMWRT_SGL_NR);
        }
        writeReg(0, CONTROL2, 0x13, 1, FRMWRT_ALL_NR);
        delay(15);
    }

    for (uint8_t board = 0; board < TOTALBOARDS; board++) {
        BQResult r = _readBoardRetry(board, AUX_GPIO1H, buf, sizeof(buf), MAXBYTES);
        if (r != BQResult::OK) { _lastReadFailBoard = board; return r; }

        uint8_t ntcs = NTCS_PER_BOARD[board % 2];
        for (uint8_t c = 0; c < ntcs; c++) {
            uint16_t raw = ((uint16_t)buf[4 + c * 2] << 8) | buf[5 + c * 2];
            if (raw == 0x8000) {                 // diagnóstico ADC / conversión no lista
                _temps[board][c] = BQ_NTC_OPEN_C;
                continue;
            }
            float voltage = _complement(raw, 0.00019073f);
            float t = _voltToTemp(voltage);
            // Rango físico plausible. El test (t > MIN && t < MAX) también
            // descarta NaN/inf y el centinela BQ_NTC_OPEN_C de _voltToTemp.
            _temps[board][c] = (t > BQ_NTC_TMIN_C && t < BQ_NTC_TMAX_C)
                                   ? t : BQ_NTC_OPEN_C;
        }
        for (uint8_t c = ntcs; c < 6; c++)
            _temps[board][c] = -99.0f;           // no poblado (distinto de NTC abierto)
    }

    _updateTempStats();
    return BQResult::OK;
}

// ============================================================
//  GETTERS
// ============================================================

float BQ79606::getVoltage(uint8_t board, uint8_t cell) const
{
    if (board >= TOTALBOARDS || cell >= 6) return 0.0f;
    return _voltages[board][cell];
}

float BQ79606::getTemperature(uint8_t board, uint8_t ntc) const
{
    if (board >= TOTALBOARDS || ntc >= 6) return 0.0f;
    return _temps[board][ntc];
}

bool BQ79606::getFaultPin() const
{
    return digitalRead(BQ_DPIN(_cfg.pinFault));
}

// ============================================================
//  FAULT STATUS
// ============================================================

BQResult BQ79606::getFaultStatus(uint8_t board, BQFaultStatus& status)
{
    // Leer 6 registros de un solo golpe desde SYS_FAULT1 (0x0201)
    // hasta FAULT_SUM (0x0206) — 6 bytes contiguos
    const uint8_t LEN = 6;
    byte buf[LEN + 6];
    memset(buf, 0, sizeof(buf));

    int res = readReg(board, SYS_FAULT1, buf, LEN, 0, FRMWRT_SGL_R);
    if (res <= 0) return BQResult::COMM_ERROR;
    if (!checkCRC(buf, res)) return BQResult::CRC_ERROR;

    // buf[4..9] = SYS_FAULT1, SYS_FAULT2, SYS_FAULT3, DEV_STAT, LOOP_STAT, FAULT_SUM
    status.sysFault1 = buf[4];
    status.sysFault2 = buf[5];
    status.sysFault3 = buf[6];
    // buf[7] = DEV_STAT, buf[8] = LOOP_STAT — no los necesitamos aquí
    status.summary   = buf[9];

    // Leer UV/OV/UT/OT desde UV_FAULT (0x0291) — 4 bytes contiguos
    const uint8_t LEN2 = 4;
    byte buf2[LEN2 + 6];
    memset(buf2, 0, sizeof(buf2));

    int res2 = readReg(board, UV_FAULT, buf2, LEN2, 0, FRMWRT_SGL_R);
    if (res2 <= 0) return BQResult::COMM_ERROR;
    if (!checkCRC(buf2, res2)) return BQResult::CRC_ERROR;

    // buf2[4..7] = UV_FAULT, OV_FAULT, UT_FAULT, OT_FAULT
    status.uvFault = buf2[4];
    status.ovFault = buf2[5];
    status.utFault = buf2[6];
    status.otFault = buf2[7];

    return BQResult::OK;
}

void BQ79606::clearAllFaults()
{
    // Reset de todos los registros de fault — write 0xFF activa el borrado
    writeReg(0, GPIO_FLT_RST,     0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, UV_FLT_RST,       0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, OV_FLT_RST,       0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, UT_FLT_RST,       0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, OT_FLT_RST,       0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, TONE_FLT_RST,     0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_UART_FLT_RST,0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_FLT_RST,0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_FLT_RST,0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, RAIL_FLT_RST,     0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, SYSFLT1_FLT_RST,  0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, SYSFLT2_FLT_RST,  0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, SYSFLT3_FLT_RST,  0xFF, 1, FRMWRT_ALL_NR);
    delay(2);
}

// ============================================================
//  VIGILANCIA DE SEGURIDAD
// ============================================================

void BQ79606::setBmsOk(bool ok)
{
    if (ok == _bmsOkState) return;          // escribir solo en los cambios
    digitalWrite(BQ_DPIN(_cfg.pinBmsOk), ok ? HIGH : LOW);  // OK=HIGH, fallo=LOW
    _bmsOkState = ok;
}

// Devuelve true cuando 'outOfRange' lleva activo de forma CONTINUA más
// que windowMs. Si vuelve a rango, resetea el contador (se trató como
// ruido). 'since' guarda millis() del primer fuera-de-rango (0 = en rango).
bool BQ79606::_debounce(bool outOfRange, uint32_t& since, uint32_t windowMs)
{
    uint32_t now = millis();
    if (now == 0) now = 1;                  // 0 es centinela de "en rango"
    if (!outOfRange) { since = 0; return false; }
    if (since == 0)  { since = now; return false; }
    return (now - since) >= windowMs;
}

void BQ79606::_safetyTrip(const char* reason, float val, float lim)
{
    _safetyLatched = true;
    setBmsOk(false);
    char b[96];
    snprintf(b, sizeof(b),
             "[SAFETY] FALLO: %s (valor=%.3f limite=%.3f) -> BMS_OK LOW (latch)\n",
             reason, val, lim);
    Serial.print(b);
    Serial.flush();
}

void BQ79606::evalSafetyVoltage()
{
    if (!_initialized || _safetyLatched) return;
    if (!isVoltageReadingReliable()) {      // balanceo HW activo: V no fiable
        _ovSince = _uvSince = 0;            // no arrastrar debounce entre estados
        return;
    }
    const uint32_t w = _safetyLim.vDebounceMs;
    bool ov = _debounce(_vMax > _safetyLim.cellOV_V, _ovSince, w);
    bool uv = _debounce(_vMin < _safetyLim.cellUV_V, _uvSince, w);
    if      (ov) _safetyTrip("OV celda", _vMax, _safetyLim.cellOV_V);
    else if (uv) _safetyTrip("UV celda", _vMin, _safetyLim.cellUV_V);
}

void BQ79606::evalSafetyTemperature()
{
    if (!_initialized || _safetyLatched) return;
    const uint32_t w = _safetyLim.tDebounceMs;

    // NTC abierto/inválido tiene PRIORIDAD: sin ese sensor no se puede
    // garantizar la seguridad térmica de ese grupo (escenario real de FS:
    // conector de NTC suelto por vibración). Política estricta: latch.
    // Debounce con la misma ventana que OT/UT para descartar glitches.
    if (_debounce(_ntcOpenCount > 0, _ntcOpenSince, w)) {
        char msg[40];
        snprintf(msg, sizeof(msg), "NTC abierto B%d C%d (n=%u)",
                 _ntcOpenBoard, _ntcOpenCell + 1, _ntcOpenCount);
        _safetyTrip(msg, (float)_ntcOpenCount, 0.0f);
        return;
    }

    bool ot = _debounce(_tMax > _safetyLim.overTemp_C,  _otSince, w);
    bool ut = _debounce(_tMin < _safetyLim.underTemp_C, _utSince, w);
    if      (ot) _safetyTrip("OT (sobretemp)", _tMax, _safetyLim.overTemp_C);
    else if (ut) _safetyTrip("UT (subtemp)",   _tMin, _safetyLim.underTemp_C);
}

void BQ79606::clearSafetyLatch()
{
    _safetyLatched = false;
    _ovSince = _uvSince = _otSince = _utSince = 0;
    _ntcOpenSince = 0;
    if (_initialized) setBmsOk(true);
    Serial.println(F("[SAFETY] Latch limpiado, vigilancia reanudada."));
    Serial.flush();
}

// ============================================================
//  BALANCEO
// ============================================================

BQResult BQ79606::startBalancing(uint8_t board, uint8_t cellMask, uint8_t timerVal)
{
    if (board >= TOTALBOARDS) return BQResult::COMM_ERROR;
    if (cellMask == 0)        return BQResult::OK;
    if (cellMask > 0x3F) {
        { char _b[128]; snprintf(_b, sizeof(_b), "[BQ] startBalancing board %d: mascara 0x%02X invalida\n", board, cellMask); Serial.print(_b); }
        return BQResult::COMM_ERROR;
    }

    // Todos los timers activos al mismo valor (compatibilidad con llamadas legacy)
    uint8_t timers[6];
    for (uint8_t c = 0; c < 6; c++)
        timers[c] = (cellMask & (1 << c)) ? timerVal : 0;
    return startBalancingTimers(board, timers);
}

BQResult BQ79606::startBalancingTimers(uint8_t board, const uint8_t timers[6])
{
    if (board >= TOTALBOARDS) return BQResult::COMM_ERROR;

    // Verificar que hay al menos una celda con timer>0
    uint8_t cellMask = 0;
    for (uint8_t c = 0; c < 6; c++)
        if (timers[c] > 0) cellMask |= (1 << c);
    if (cellMask == 0) return BQResult::OK;
    if (cellMask > 0x3F) return BQResult::COMM_ERROR;

    // CB_CONFIG = 0x0A: 2min por fase, odds->evens (SEQ=10), continuar en fallo.
    writeReg(board, CB_CONFIG, 0x0A, 1, FRMWRT_SGL_NR);
    delay(5);

    // Escribir timer individual por celda
    for (uint8_t c = 0; c < 6; c++) {
        int res = writeReg(board, CB_CELL1_CTRL + c, timers[c], 1, FRMWRT_SGL_NR);
        if (res <= 0) return BQResult::COMM_ERROR;
    }
    delay(5);

    // Activar BAL_GO — _balHwRunning antes para bloquear readVoltages
    _balHwRunning = true;
    int res = writeReg(board, CONTROL2, 0x30, 1, FRMWRT_SGL_NR);
    if (res <= 0) { _balHwRunning = false; return BQResult::COMM_ERROR; }

    // Verificar FETs via CB_SW_STAT
    delay(200);
    {
        byte verBuf[7] = {0};
        int vr = readReg(board, CB_SW_STAT, verBuf, 1, 0, FRMWRT_SGL_R);
        if (vr > 0 && checkCRC(verBuf, vr)) {
            uint8_t sw = verBuf[4] & 0x3F;
            { char _b[128]; snprintf(_b, sizeof(_b), "[BQ] startBalancing B%d: CB_SW_STAT=0x%02X (%s)\n",
                          board, sw, sw ? "FETs activos OK" : "ADVERTENCIA: ningun FET activo"); Serial.print(_b); }
            if (sw == 0) { _balHwRunning = false; return BQResult::COMM_ERROR; }
        }
    }
    return BQResult::OK;
}

BQResult BQ79606::updateBalancingTimers(uint8_t board, const uint8_t timers[6])
{
    if (board >= TOTALBOARDS) return BQResult::COMM_ERROR;

    // Actualizar timers individualmente durante el descanso entre fases.
    // El datasheet prohíbe escribir CB_CELLx_CTRL con BAL_GO=1 (activo),
    // pero durante el descanso (CB_SW_STAT=0) el IC acepta las escrituras.
    for (uint8_t c = 0; c < 6; c++) {
        int res = writeReg(board, CB_CELL1_CTRL + c, timers[c], 1, FRMWRT_SGL_NR);
        if (res <= 0) return BQResult::COMM_ERROR;
    }
    return BQResult::OK;
}

BQResult BQ79606::stopBalancing(uint8_t board)
{
    if (board >= TOTALBOARDS) return BQResult::COMM_ERROR;

    // Paso 1: Pausar para congelar el ciclo
    writeReg(board, CB_SW_EN, CB_PAUSE_BIT, 1, FRMWRT_SGL_NR);
    delay(10);

    // Paso 2: Timers a 0 (aceptados mientras está pausado)
    for (uint8_t c = 0; c < 6; c++)
        writeReg(board, CB_CELL1_CTRL + c, 0x00, 1, FRMWRT_SGL_NR);
    delay(5);

    // Paso 3: Quitar pausa y disparar BAL_GO con timers=0 para terminar el ciclo
    writeReg(board, CB_SW_EN, 0x00, 1, FRMWRT_SGL_NR);
    delay(5);
    int res = writeReg(board, CONTROL2, 0x30, 1, FRMWRT_SGL_NR);
    if (res <= 0) return BQResult::COMM_ERROR;
    delay(50);

    _balHwRunning = false;
    return BQResult::OK;
}

void BQ79606::stopAllBalancing()
{
    // Según el datasheet BQ79606A-Q1 sección 8.3.5.1.3:
    //
    // Para PAUSAR: CB_SW_EN[CB_PAUSE] (bit6 del registro 0x0115)
    //   "Cell balancing is paused using the CB_SW_EN[CB_PAUSE] bit.
    //    When set, the state machine is frozen and all switches are turned off."
    //
    // Para PARAR completamente:
    //   "To stop cell balancing before completion, all timers must be set to 0
    //    and then write CONTROL2[BAL_GO] = 1."
    //   El nuevo BAL_GO con timers=0 arranca y termina un ciclo vacío al instante.
    //
    // Secuencia:
    //   1. CB_PAUSE para congelar el ciclo activo inmediatamente
    //   2. Timers a 0 en todos los boards (ahora el IC los acepta porque está pausado)
    //   3. BAL_GO=1 para arrancar ciclo vacío — el IC lo termina solo y queda parado
    //   4. Quitar CB_PAUSE

    // Paso 1: Pausar el balanceo en todos los boards
    for (uint8_t b = 0; b < TOTALBOARDS; b++)
        writeReg(b, CB_SW_EN, CB_PAUSE_BIT, 1, FRMWRT_SGL_NR);
    delay(10);

    // Paso 2: Timers a 0 (el IC los acepta mientras está pausado)
    for (uint8_t b = 0; b < TOTALBOARDS; b++)
        for (uint8_t c = 0; c < 6; c++)
            writeReg(b, CB_CELL1_CTRL + c, 0x00, 1, FRMWRT_SGL_NR);
    delay(5);

    // Paso 3: Quitar CB_PAUSE y arrancar BAL_GO con timers=0
    // El IC inicia un ciclo vacío y lo termina inmediatamente
    for (uint8_t b = 0; b < TOTALBOARDS; b++)
        writeReg(b, CB_SW_EN, 0x00, 1, FRMWRT_SGL_NR);
    delay(5);
    writeReg(0, CONTROL2, 0x30, 1, FRMWRT_ALL_NR);  // BAL_GO + TSREF
    delay(50);  // esperar que el ciclo vacío termine

    // Verificar que los FETs se apagaron
    bool allOff = true;
    for (uint8_t b = 0; b < TOTALBOARDS; b++) {
        byte buf[7] = {0};
        int r = readReg(b, CB_SW_STAT, buf, 1, 0, FRMWRT_SGL_R);
        if (r > 0 && checkCRC(buf, r) && (buf[4] & 0x3F) != 0) {
            allOff = false;
            { char _b[128]; snprintf(_b, sizeof(_b), "[BQ] B%d CB_SW_STAT=0x%02X (FETs aun activos)\n", b, buf[4] & 0x3F); Serial.print(_b); }
        }
    }
    { char _b[128]; snprintf(_b, sizeof(_b), "[BQ] stopAllBalancing: FETs %s\n", allOff ? "apagados OK" : "pendientes"); Serial.print(_b); }

    writeReg(0, CB_CONFIG, 0x0A, 1, FRMWRT_ALL_NR);  // restaurar para próximo ciclo
    _balHwRunning = false;
}

void BQ79606::setCBConfig(uint8_t val)
{
    // ⚠ SIN USO: nadie llama a setCBConfig(). El mapa de bits que asumía el
    // comentario original (CB_LOOP=bit3, CB_ERR_STOP=bit2) NO existe. Datasheet
    // §8.6.1.215: B7=DUTY_UNIT, B6:B3=DUTY, B2=FLTSTOP, B1:B0=SEQ. Con la lógica
    // de abajo, `(val&0xF0)|0x0C` forzaría FLTSTOP=1 y SEQ=00 (solo impares),
    // que NO es lo pretendido. No tocar/llamar sin revisar y probar; la config
    // real de balanceo la ponen startBalancingTimers() y _initDevices() (0x0A).
    val = (val & 0xF0) | 0x0C;
    writeReg(0, CB_CONFIG, val, 1, FRMWRT_ALL_NR);
}

bool BQ79606::isBalancing(uint8_t board)
{
    if (board >= TOTALBOARDS) return false;

    byte buf[1 + 6];
    memset(buf, 0, sizeof(buf));

    int res = readReg(board, CB_SW_STAT, buf, 1, 0, FRMWRT_SGL_R);
    if (res <= 0) return false;
    if (!checkCRC(buf, res)) return false;

    return (buf[4] & 0x3F) != 0;
}

bool BQ79606::isCellBalancing(uint8_t board, uint8_t cell)
{
    return (getBalancingMask(board) & (1 << cell)) != 0;
}

uint8_t BQ79606::getBalancingMask(uint8_t board)
{
    if (board >= TOTALBOARDS) return 0;
    byte buf[7] = {0};
    int res = readReg(board, CB_SW_STAT, buf, 1, 0, FRMWRT_SGL_R);
    if (res <= 0 || !checkCRC(buf, res)) return 0;
    return buf[4] & 0x3F;
}

bool BQ79606::isBalancingDone(uint8_t board)
{
    if (board >= TOTALBOARDS) return false;

    byte buf[7] = {0};

    // Leer la máscara de celdas que tienen timer asignado (CB_SW_STAT o CB_CELLx_CTRL)
    // y compararla con CB_DONE para saber si TODAS terminaron.
    //
    // Estrategia: leer CB_DONE y CB_SW_STAT.
    // - CB_SW_STAT = 0 significa que ningún FET está activo (todas terminaron o no había).
    // - CB_DONE bits 0-5 = celdas que completaron su timer.
    //
    // Consideramos "done" cuando CB_SW_STAT=0 Y CB_DONE!=0
    // (alguna celda terminó y no hay ninguna activa todavía).
    // Esto evita detectar done a los 2min cuando solo terminan las odds.

    // Leer CB_DONE
    int r1 = readReg(board, CB_DONE, buf, 1, 0, FRMWRT_SGL_R);
    if (r1 <= 0 || !checkCRC(buf, r1)) return false;
    uint8_t cbDone = buf[4] & 0x3F;
    if (cbDone == 0) return false;  // ninguna celda ha terminado aún

    // Leer CB_SW_STAT — si algún FET sigue activo, el ciclo no ha terminado
    memset(buf, 0, sizeof(buf));
    int r2 = readReg(board, CB_SW_STAT, buf, 1, 0, FRMWRT_SGL_R);
    if (r2 <= 0 || !checkCRC(buf, r2)) return false;
    uint8_t swStat = buf[4] & 0x3F;

    // Done = alguna celda terminó Y ningún FET sigue activo
    return (cbDone != 0) && (swStat == 0);
}

void BQ79606::_updateVoltageStats()
{
    _vMin = 99.0f;
    _vMax = 0.0f;
    for (uint8_t b = 0; b < TOTALBOARDS; b++) {
        // Los boards impares solo tienen 5 celdas reales — usar CELLS_FOR_BOARD(b).
        // La celda 6 (índice 5) siempre mide ~0V — excluirla para no corromper Vmin.
        uint8_t nCells = CELLS_FOR_BOARD(b);
        for (uint8_t c = 0; c < nCells; c++) {
            if (_voltages[b][c] < _vMin) _vMin = _voltages[b][c];
            if (_voltages[b][c] > _vMax) _vMax = _voltages[b][c];
        }
    }
}

void BQ79606::_updateTempStats()
{
    _tMin =  999.0f;
    _tMax = -999.0f;
    _ntcOpenCount = 0;
    _ntcOpenBoard = -1;
    _ntcOpenCell  = -1;
    for (uint8_t b = 0; b < TOTALBOARDS; b++) {
        for (uint8_t c = 0; c < NTCS_PER_BOARD[b % 2]; c++) {
            float t = _temps[b][c];
            // NTC configurado pero abierto/inválido: NO entra en min/max
            // (evitaría un falso UT/OT con un número basura) y se contabiliza
            // para que evalSafetyTemperature() dispare el fallo correcto.
            if (t <= BQ_NTC_TMIN_C || t >= BQ_NTC_TMAX_C) {
                if (_ntcOpenCount == 0) {
                    _ntcOpenBoard = (int8_t)b;
                    _ntcOpenCell  = (int8_t)c;
                }
                _ntcOpenCount++;
                continue;
            }
            if (t < _tMin) _tMin = t;
            if (t > _tMax) _tMax = t;
        }
    }
}

// ============================================================
//  COMUNICACIÓN PÚBLICA (debug / balanceo)
// ============================================================

int BQ79606::writeReg(byte id, uint16_t addr, uint64_t data, byte len, byte type)
{
    int res = 0;
    memset(_bBuf, 0, sizeof(_bBuf));

    // Desempaquetar data en _bBuf (big-endian)
    for (int i = len - 1; i >= 0; i--) {
        _bBuf[i] = data & 0xFF;
        data >>= 8;
    }
    res = _writeFrame(id, addr, _bBuf, len, type);
    return res;
}

int BQ79606::_readRegImpl(byte id, uint16_t addr, byte* buf, size_t bufSize,
                          byte len, uint32_t timeout, byte type)
{
    const uint32_t DEFAULT_TIMEOUT_MS = 10;
    int expectedLen = 0;

    if      (type == FRMWRT_SGL_R) expectedLen = len + 6;
    else if (type == FRMWRT_STK_R) expectedLen = (len + 6) * (TOTALBOARDS - 1);
    else if (type == FRMWRT_ALL_R) expectedLen = (len + 6) * TOTALBOARDS;
    else return 0;

    // Validar contra el buffer DESTINO real (no contra _pFrame): se escribe en
    // buf, así que un STK_R/ALL_R con buf pequeño desbordaría si no se vigila aquí.
    if (expectedLen > (int)bufSize) {
        Serial.println(F("[BQ] readReg: respuesta excede el buffer destino."));
        return -2;
    }

    if (_readFrameReq(id, addr, len, type) == 0) {
        Serial.println(F("[BQ] readReg: ReadFrameReq > 128 bytes."));
        return -3;
    }

    _uart.setTimeout(timeout == 0 ? DEFAULT_TIMEOUT_MS : timeout);
    memset(buf, 0, expectedLen);

    int bytesRead = _uart.readBytes(buf, expectedLen);
    if (bytesRead != expectedLen) return -1;

    return bytesRead;
}

bool BQ79606::checkCRC(uint8_t* data, uint16_t len)
{
    if (len < 2) return false;
    uint16_t received = ((uint16_t)data[len - 1] << 8) | data[len - 2];
    uint16_t calc     = _crc16(data, len - 2);
    return received == calc;
}

// ============================================================
//  DEBUG
// ============================================================

void BQ79606::printRegisters(uint8_t deviceID)
{
    const uint16_t START = 0x0200;
    const uint8_t  LEN   = 81;
    uint8_t buf[LEN + 6] = {0};

    int n = readReg(deviceID, START, buf, LEN, 0, FRMWRT_SGL_R);
    if (n <= 0) {
        { char _b[128]; snprintf(_b, sizeof(_b), "[BQ] printRegisters: error %d\n", n); Serial.print(_b); }
        return;
    }
    if (!checkCRC(buf, n)) {
        Serial.println(F("[BQ] printRegisters: CRC incorrecto."));
        return;
    }
    Serial.println(F("Addr   | Hex"));
    Serial.println(F("-------+-----"));
    for (int i = 0; i < LEN; i++) {
        { char _b[128]; snprintf(_b, sizeof(_b), "0x%04X | 0x%02X\n", START + i, buf[4 + i]); Serial.print(_b); }
    }
}

void BQ79606::printRegister(uint8_t deviceID, uint16_t addr)
{
    uint8_t buf[7] = {0};
    int n = readReg(deviceID, addr, buf, 1, 0, FRMWRT_SGL_R);
    if (n < 7) {
        { char _b[128]; snprintf(_b, sizeof(_b), "[BQ] printRegister 0x%04X: error %d\n", addr, n); Serial.print(_b); }
        return;
    }
    if (!checkCRC(buf, n)) {
        Serial.println(F("[BQ] printRegister: CRC incorrecto."));
        return;
    }
    { char _b[128]; snprintf(_b, sizeof(_b), "0x%04X = 0x%02X\n", addr, buf[4]); Serial.print(_b); }
}

// ============================================================
//  PROTOCOLO PRIVADO
// ============================================================

void BQ79606::_wakeUp()
{
    digitalWrite(BQ_DPIN(_cfg.pinWake), LOW);
    delayMicroseconds(300);
    digitalWrite(BQ_DPIN(_cfg.pinWake), HIGH);
    delay(12 * TOTALBOARDS);
}

void BQ79606::_commClear()
{
    _uart.end();
    pinMode(BQ_DPIN(_cfg.pinTx), OUTPUT);
    digitalWrite(BQ_DPIN(_cfg.pinTx), LOW);
    delayMicroseconds(17UL * (1000000UL / _cfg.baudrate));
}

void BQ79606::_commSleepToWake()
{
    _uart.end();
    pinMode(BQ_DPIN(_cfg.pinTx), OUTPUT);
    digitalWrite(BQ_DPIN(_cfg.pinTx), LOW);
    delayMicroseconds(260);
    _uart.begin(_cfg.baudrate);
    delay(50);
}

void BQ79606::_commReset(int baud)
{
    _uart.end();
    pinMode(BQ_DPIN(_cfg.pinTx), OUTPUT);
    digitalWrite(BQ_DPIN(_cfg.pinTx), LOW);
    delayMicroseconds(500);
    digitalWrite(BQ_DPIN(_cfg.pinTx), HIGH);

    _uart.begin(250000);
    delay(10);

    const int delayUs = 10000;
    uint16_t commCtrlVal = 0x3C3C; // default 1M

    if      (baud == 1000000) commCtrlVal = 0x3C3C;
    else if (baud ==  500000) commCtrlVal = 0x383C;
    else if (baud ==  250000) commCtrlVal = 0x343C;
    else if (baud ==  125000) commCtrlVal = 0x303C;
    else {
        Serial.println(F("[BQ] Baudrate inválido, usando 1M."));
        baud = 1000000;
    }

    writeReg(0, COMM_CTRL, commCtrlVal, 2, FRMWRT_ALL_NR);
    delayMicroseconds(delayUs);
    // STM32duino: begin() NO reconfigura el baud si la UART ya esta iniciada
    // (a diferencia de ESP32). Forzar end() antes del nuevo begin() para que
    // el host pase de 250000 al baud objetivo y NO quede desincronizado con
    // la cadena (sintoma: TODOS los boards ERROR comm, incluido el base).
    _uart.end();
    _uart.begin(baud);
    delayMicroseconds(100);
}

bool BQ79606::_autoAddress()
{
    byte resp[(MAXBYTES + 6) * TOTALBOARDS];
    memset(resp, 0, sizeof(resp));

    writeReg(0, ECC_TEST, 0x00, 1, FRMWRT_ALL_NR); delay(5);
    writeReg(0, CONFIG,   0x00, 1, FRMWRT_ALL_NR); delay(5);
    writeReg(0, CONTROL1, 0x01, 1, FRMWRT_ALL_NR); delay(5);

    for (_nCurrentBoard = 0; _nCurrentBoard < TOTALBOARDS; _nCurrentBoard++) {
        writeReg(_nCurrentBoard, DEVADD_USR, _nCurrentBoard, 1, FRMWRT_ALL_NR);
        delayMicroseconds(1000);
    }

    writeReg(0,              CONFIG, 0x02, 1, FRMWRT_ALL_NR);
    writeReg(0,              CONFIG, 0x00, 1, FRMWRT_SGL_NR);
    writeReg(TOTALBOARDS - 1,CONFIG, 0x03, 1, FRMWRT_SGL_NR);

    readReg(TOTALBOARDS - 1, ECC_TEST, resp, 1, 0, FRMWRT_ALL_R);

    // Verificar que cada board responde con su dirección
    bool ok = true;
    Serial.println(F("\n--- Verificacion AutoAddress ---"));
    for (_nCurrentBoard = 0; _nCurrentBoard < TOTALBOARDS; _nCurrentBoard++) {
        memset(resp, 0, sizeof(resp));
        int res = readReg(_nCurrentBoard, DEVADD_USR, resp, 1, 0, FRMWRT_SGL_R);

        if (res <= 0) {
            Serial.print(F("  Board ")); Serial.print(_nCurrentBoard);
            Serial.println(F(" -> ERROR comm"));
            ok = false;
            delay(10);
            continue;
        }

        if (!checkCRC(resp, res)) {
            Serial.print(F("  Board ")); Serial.print(_nCurrentBoard);
            Serial.println(F(" -> ERROR CRC"));
            ok = false;
            delay(10);
            continue;
        }

        uint8_t received = resp[4];
        Serial.print(F("  Board ")); Serial.print(_nCurrentBoard);
        Serial.print(F(" -> esperado ")); Serial.print(_nCurrentBoard);
        Serial.print(F(", recibido ")); Serial.println(received);
        if (received != _nCurrentBoard) ok = false;
        delay(10);
    }
    Serial.println(F("--------------------------------"));
    return ok;
}

void BQ79606::_initDevices()
{
    delay(1);
    writeReg(0, COMM_TO,    0x00, 1, FRMWRT_ALL_NR);
    writeReg(0, TX_HOLD_OFF,0x00, 1, FRMWRT_ALL_NR);

    // Enmascarar todos los fallos de bajo nivel
    writeReg(0, GPIO_FLT_MSK,        0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, UV_FLT_MSK,          0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, OV_FLT_MSK,          0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, UT_FLT_MSK,          0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, OT_FLT_MSK,          0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, TONE_FLT_MSK,        0x07, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_UART_FLT_MSK,   0x07, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_UART_RC_FLT_MSK,0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_UART_RR_FLT_MSK,0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_UART_TR_FLT_MSK,0x03, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_FLT_MSK,   0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_RC_FLT_MSK,0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_RR_FLT_MSK,0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_TR_FLT_MSK,0x03, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_FLT_MSK,   0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_RC_FLT_MSK,0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_RR_FLT_MSK,0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_TR_FLT_MSK,0x03, 1, FRMWRT_ALL_NR);
    writeReg(0, OTP_FLT_MSK,         0x07, 1, FRMWRT_ALL_NR);
    writeReg(0, RAIL_FLT_MSK,        0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, SYSFLT1_FLT_MSK,     0x7F, 1, FRMWRT_ALL_NR);
    writeReg(0, SYSFLT2_FLT_MSK,     0xFF, 1, FRMWRT_ALL_NR);
    writeReg(0, SYSFLT3_FLT_MSK,     0x7F, 1, FRMWRT_ALL_NR);
    writeReg(0, OVUV_BIST_FLT_MSK,   0x03, 1, FRMWRT_ALL_NR);
    writeReg(0, OTUT_BIST_FLT_MSK,   0xFF, 1, FRMWRT_ALL_NR);

    // ADC de celdas
    writeReg(0, CELL_ADC_CTRL,  0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, OVUV_CTRL,      0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, UV_THRESH,      0x53, 1, FRMWRT_ALL_NR); // 2.8V
    writeReg(0, OV_THRESH,      0x5B, 1, FRMWRT_ALL_NR); // 4.3V
    writeReg(0, GPIO_ADC_CONF,  0x00, 1, FRMWRT_ALL_NR);

    for (_nCurrentBoard = 0; _nCurrentBoard < TOTALBOARDS; _nCurrentBoard++)
        writeReg(_nCurrentBoard, ADC_DELAY, 0x00, 1, FRMWRT_SGL_NR);

    // AUX_ADC_CONF: GPAI_NUM bits controlan cuántos GPIOs secuencia el AUX ADC.
    // 0x18 = 0b00011000 prueba GPAI_NUM[1:0]=11 en bits [4:3] → máximo canales.
    writeReg(0, AUX_ADC_CONF,   0x18, 1, FRMWRT_ALL_NR);
    writeReg(0, CELL_ADC_CONF1, 0x67, 1, FRMWRT_ALL_NR);
    writeReg(0, CELL_ADC_CONF2, 0x00, 1, FRMWRT_ALL_NR);

    // ── Configuración de GPIOs y comparadores OTUT por board ────────────────
    // Cada board tiene un número distinto de NTCs conectados (NTCS_PER_BOARD).
    // Los GPIOs sin NTC se fuerzan como salida digital (0x00) para que el pin
    // no flote — un pin analógico flotante lee ~0V, que el comparador OTUT
    // interpreta como sobretemperatura y bloquea BAL_GO.
    // OTUT_CTRL habilita el comparador solo en los GPIOs con NTC real.
    for (uint8_t b = 0; b < TOTALBOARDS; b++) {
        uint8_t ntcs    = NTCS_PER_BOARD[b % 2];        // GPIOs con NTC: 0..ntcs-1
        uint8_t otutMsk = (1u << ntcs) - 1u;        // máscara de bits 0..(ntcs-1)
        for (uint8_t g = 0; g < 6; g++) {
            uint8_t cfg = (g < ntcs) ? 0x20 : 0x00; // 0x20=analógico, 0x00=digital
            writeReg(b, GPIO1_CONF + g, cfg, 1, FRMWRT_SGL_NR);
        }
        writeReg(b, OTUT_CTRL,   otutMsk, 1, FRMWRT_SGL_NR);
        writeReg(b, OTUT_THRESH, 0x9B,    1, FRMWRT_SGL_NR); // OT~60°C, UT~-20°C
    }

    writeReg(0, CONTROL2,       0x10, 1, FRMWRT_ALL_NR); // TSREF ON
    delay(2);

    // Lecturas de diagnóstico iniciales
    for (_nCurrentBoard = 0; _nCurrentBoard < TOTALBOARDS; _nCurrentBoard++) {
        readReg(_nCurrentBoard, PARTID,    _bFrame, 1, 0, FRMWRT_SGL_R); delayMicroseconds(500);
        readReg(_nCurrentBoard, DEV_STAT,  _bFrame, 1, 0, FRMWRT_SGL_R); delayMicroseconds(500);
        readReg(_nCurrentBoard, LOOP_STAT, _bFrame, 1, 0, FRMWRT_SGL_R); delayMicroseconds(500);
        readReg(_nCurrentBoard, FAULT_SUM, _bFrame, 1, 0, FRMWRT_SGL_R); delayMicroseconds(500);
    }

    // Actualizar CRC de cliente
    // buf[4]=CUST_CRC_RSLTH (byte alto), buf[5]=CUST_CRC_RSLTL (byte bajo)
    for (_nCurrentBoard = 0; _nCurrentBoard < TOTALBOARDS; _nCurrentBoard++) {
        readReg(_nCurrentBoard, CUST_CRC_RSLTH, _bFrame, 2, 0, FRMWRT_SGL_R);
        delay(1);
        writeReg(_nCurrentBoard, CUST_CRCH, _bFrame[4], 1, FRMWRT_SGL_NR);  // byte alto
        writeReg(_nCurrentBoard, CUST_CRCL, _bFrame[5], 1, FRMWRT_SGL_NR);  // byte bajo
    }

    // AUX_ADC_CTRL: configurar por board según número de NTCs conectados.
    // Board par  (GPIO1-6): CTRL1=0xF0, CTRL2=0x03
    // Board impar (GPIO1-3): CTRL1=0x70, CTRL2=0x00
    // Enviar broadcast con el valor par, luego corregir los impares.
    writeReg(0, AUX_ADC_CTRL1, 0xF0, 1, FRMWRT_ALL_NR);
    writeReg(0, AUX_ADC_CTRL2, 0x03, 1, FRMWRT_ALL_NR);
    writeReg(0, AUX_ADC_CTRL3, 0x00, 1, FRMWRT_ALL_NR);
    for (uint8_t b = 1; b < TOTALBOARDS; b += 2) {
        // Boards impares: solo GPIO1-3 (CTRL1=0x70, CTRL2=0x00)
        writeReg(b, AUX_ADC_CTRL1, 0x70, 1, FRMWRT_SGL_NR);
        writeReg(b, AUX_ADC_CTRL2, 0x00, 1, FRMWRT_SGL_NR);
    }
    delayMicroseconds(100);

    for (_nCurrentBoard = 0; _nCurrentBoard < TOTALBOARDS; _nCurrentBoard++) {
        readReg(_nCurrentBoard, CB_SW_STAT, _bFrame, 1, 0, FRMWRT_SGL_R);
        delayMicroseconds(500);
    }

    // CB_CONFIG = 0x0A = 0b0000_1010 (decodificado del datasheet §8.6.1.215):
    //   B7 DUTY_UNIT=0 (minutos) | B6:B3 DUTY=1 -> 1x2 = 2 min por fase
    //   B2 FLTSTOP=0 (sigue balanceando en fallo, salvo thermal shutdown)
    //   B1:B0 SEQ=0b10 -> impares, luego pares
    // (El antiguo comentario "[7:6]=01(60s)..." usaba un mapa de bits erróneo.)
    writeReg(0, CB_CONFIG, 0x0A, 1, FRMWRT_ALL_NR);

    // Timers a 0 por seguridad — ninguna celda debe balancear hasta que
    // startBalancing() lo indique explícitamente.
    for (uint8_t c = 0; c < 6; c++)
        writeReg(0, CB_CELL1_CTRL + c, 0x00, 1, FRMWRT_ALL_NR);

    delay(2);
}

// ============================================================
//  UTILIDADES PRIVADAS
// ============================================================

/**
 * Construye una trama completa según el protocolo BQ79606 y la envía por UART.
 *
 * Estructura de la trama enviada:
 *   [INIT_BYTE] [DEV_ADDR?] [REG_H] [REG_L] [DATA...] [CRC_L] [CRC_H]
 *
 *   - INIT_BYTE: 0x80 | type | (len-1 si es escritura)
 *   - DEV_ADDR: solo presente en FRMWRT_SGL_R y FRMWRT_SGL_NR
 *   - CRC: calculado sobre todos los bytes anteriores (sin incluirse a sí mismo)
 *          y enviado en little-endian (byte bajo primero)
 */
int BQ79606::_writeFrame(byte id, uint16_t addr, byte* data, byte len, byte type)
{
    int pktLen = 0;
    uint8_t* p = _pFrame;
    memset(_pFrame, 0x7F, sizeof(_pFrame));

    *p++ = 0x80 | type | ((type & 0x10) ? len - 1 : 0);
    if (type == FRMWRT_SGL_R || type == FRMWRT_SGL_NR)
        *p++ = id & 0xFF;

    *p++ = (addr >> 8) & 0xFF;
    *p++ = addr & 0xFF;

    while (len--) *p++ = *data++;

    pktLen = p - _pFrame;
    uint16_t crc = _crc16(_pFrame, pktLen);
    *p++ = crc & 0xFF;
    *p++ = (crc >> 8) & 0xFF;
    pktLen += 2;

    _uart.write(_pFrame, pktLen);
    return pktLen;
}

/**
 * Envía una petición de lectura al IC especificado.
 *
 * Antes de enviar, limpia el buffer UART para descartar ecos residuales
 * de tramas anteriores. Esto es crítico en un bus half-duplex donde el
 * MCU recibe su propio eco.
 *
 * El byte de datos enviado es (bytesToReturn - 1): el BQ interpreta este
 * valor como "número de bytes adicionales a devolver tras el primero".
 */
int BQ79606::_readFrameReq(byte id, uint16_t addr, byte bytesToReturn, byte type)
{
    // Limpiar buffer antes de enviar para descartar ecos residuales
    while (_uart.available()) _uart.read();

    _bReturn = bytesToReturn - 1;
    if (_bReturn <= 127)
        return _writeFrame(id, addr, &_bReturn, 1, type);
    return 0;
}

/**
 * Calcula el CRC-16 ITU-T sobre un buffer de datos.
 *
 * Algoritmo: XOR del byte de entrada con el byte bajo del CRC acumulado,
 * luego lookup en la tabla para obtener el nuevo CRC.
 *
 * @param buf  Puntero al inicio de los datos.
 * @param len  Número de bytes sobre los que calcular el CRC.
 * @return     Valor CRC de 16 bits.
 */
uint16_t BQ79606::_crc16(byte* buf, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i] & 0x00FF;
        crc = crc16_table[crc & 0xFF] ^ (crc >> 8);
    }
    return crc;
}

/**
 * Convierte un valor raw del ADC del BQ79606 a voltios.
 *
 * El ADC usa formato de complemento a 2 de 16 bits.
 * La resolución es 190.73 µV/LSB (multiplier = 0.00019073).
 *
 * Fórmula: V = -(~raw + 1) × multiplier
 *
 * El signo negativo es necesario porque el hardware del BQ invierte el signo
 * en la representación interna para voltajes positivos.
 */
float BQ79606::_complement(uint16_t raw, float multiplier)
{
    return -1.0f * (float)((int16_t)(~raw + 1)) * multiplier;
}

/**
 * Convierte el voltaje de salida de un NTC a temperatura en grados Celsius.
 *
 * El NTC está en un divisor de tensión alimentado por TSREF (2.5V).
 * La resistencia del NTC se calcula a partir del voltaje medido en el GPIO,
 * y luego se aplica una curva polinómica de tercer orden calibrada para
 * el NTC específico del PCB.
 *
 * Curva: T = a·R³ + b·R² + c·R + d  (R en kΩ)
 *   a = -7.388512707e-02
 *   b =  1.987401122e+00
 *   c = -1.975941021e+01
 *   d =  9.894631155e+01
 *
 * ⚠ Si se cambia el NTC del PCB, hay que recalibrar estos coeficientes.
 */
float BQ79606::_voltToTemp(float v)
{
    // Blindaje: con NTC abierto/lectura inválida el divisor puede acercarse a 0
    // (v≈2.38V) y R explotar a ±1e9 → temperatura no física que falsearía la
    // seguridad. Detectarlo y devolver el centinela de "NTC abierto".
    float den = 200000.0f * (2.5f - v) - 9760.0f * v;
    if (den < 1.0f && den > -1.0f) return BQ_NTC_OPEN_C;   // divisor ~0
    float R = (1.952e9f * v) / den;
    if (R <= 0.0f) return BQ_NTC_OPEN_C;                    // resistencia no física
    float Rk = R / 1000.0f;
    return -7.388512707e-02f * Rk * Rk * Rk
           + 1.987401122e+00f * Rk * Rk
           - 1.975941021e+01f * Rk
           + 9.894631155e+01f;
}