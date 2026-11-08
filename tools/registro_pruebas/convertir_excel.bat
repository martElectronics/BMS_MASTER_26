@echo off
REM ============================================================================
REM  Bateria de una sesion del AiM -> Excel visual (modulos y paralelos).
REM  Doble clic -> eliges el CSV exportado de RaceStudio (o el .xrk) -> modulos\<sesion>_baterias.xlsx
REM  con graficas, mapas de color y tiras en el tiempo. Necesita los canales BMS Pk*
REM  (IDs 393-396). Mejor el .xrk (C:\AIM_SPORT\RaceStudio3\user\data): trae todas
REM  las tramas sin interpolar; para leerlo: py -m pip install libxrk.
REM  Con el CSV exportado tambien funciona (rescata lo que vale).
REM ============================================================================
cd /d "%~dp0"
chcp 65001 >nul
where py >nul 2>nul && (set "PY=py") || (set "PY=python")
%PY% aim_a_excel.py %*
echo.
pause
