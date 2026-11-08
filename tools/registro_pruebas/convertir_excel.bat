@echo off
REM ============================================================================
REM  Bateria de una sesion del AiM -> Excel visual (modulos y paralelos).
REM  Doble clic -> eliges el CSV exportado de RaceStudio -> modulos\<sesion>_baterias.xlsx
REM  con graficas, mapas de color y tiras en el tiempo. Necesita los canales BMS Pk*
REM  (IDs 393-396). Funciona aunque falte "Stepped Values" (rescata lo que vale).
REM  No necesita instalar nada.
REM ============================================================================
cd /d "%~dp0"
chcp 65001 >nul
where py >nul 2>nul && (set "PY=py") || (set "PY=python")
%PY% aim_a_excel.py %*
echo.
pause
