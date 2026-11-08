@echo off
REM ============================================================================
REM  Registro completo de una carga (firmware charger). Doble clic y dejalo abierto.
REM  1. Arrancalo ANTES de dar corriente (con 15-20 min de reposo del pack).
REM  2. Carga normal (7 A) hasta ~4,05 V.
REM  3. Para la carga y dejalo 30 min mas: el reposo final da la capacidad.
REM  4. Ctrl+C para terminar. Luego analizar_carga.bat.
REM  Cierra el monitor de PlatformIO antes (el puerto COM no se comparte).
REM  La 1a vez instala pyserial solo.
REM ============================================================================
cd /d "%~dp0"
chcp 65001 >nul
where py >nul 2>nul && (set "PY=py") || (set "PY=python")
%PY% -c "import serial" 2>nul || %PY% -m pip install pyserial --quiet --disable-pip-version-check
%PY% registrar_carga.py %*
echo.
pause
