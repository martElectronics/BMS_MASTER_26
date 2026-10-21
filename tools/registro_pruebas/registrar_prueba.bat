@echo off
REM ============================================================================
REM  Registrar una prueba: CSV del AiM -> fila de la hoja "Testing 26/27".
REM  Doble clic -> eliges el CSV exportado de RaceStudio -> escribes el codigo de
REM  la prueba (Hoja 2) -> la fila queda en el portapapeles para pegar en Hoja 1.
REM  No necesita instalar nada (solo Python).
REM ============================================================================
cd /d "%~dp0"
chcp 65001 >nul
where py >nul 2>nul && (set "PY=py") || (set "PY=python")
%PY% aim_a_fila.py %*
echo.
pause
