@echo off
REM ============================================================================
REM  Paralelos y NTC de todo el pack: CSV del AiM -> tabla por segundo.
REM  Doble clic -> eliges el CSV exportado de RaceStudio -> resumen en pantalla y
REM  tabla en modulos\<sesion>_pack.csv. Necesita los canales BMS Pk* (IDs
REM  393-396) en el log, con "Stepped Values". No necesita instalar nada.
REM  Validar el AiM en banco: comando P por serie al BMS y arrastrar el CSV a
REM  este .bat con --patron (o: py aim_a_modulos.py sesion.csv --patron).
REM ============================================================================
cd /d "%~dp0"
chcp 65001 >nul
where py >nul 2>nul && (set "PY=py") || (set "PY=python")
%PY% aim_a_modulos.py %*
echo.
pause
