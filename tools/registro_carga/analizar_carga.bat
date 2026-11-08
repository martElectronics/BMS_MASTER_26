@echo off
REM ============================================================================
REM  Capacidad de cada paralelo y del pack a partir de una carga registrada.
REM  Doble clic -> eliges el CSV de registros\ -> resultados\<carga>.xlsx
REM  No necesita instalar nada.
REM ============================================================================
cd /d "%~dp0"
chcp 65001 >nul
where py >nul 2>nul && (set "PY=py") || (set "PY=python")
%PY% analizar_carga.py %*
echo.
pause
