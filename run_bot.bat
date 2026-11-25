@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "PY=%~dp0.venv\Scripts\python.exe"
if exist "%PY%" goto :run
echo Run setup_python.bat first.
pause
exit /b 1

:run
rem Local practice flag: my_car.py saves memory.json and logs\ (off on the contest server)
set "SSAFY_LOCAL=1"
set "BOT=%~1"
if "%BOT%"=="" set "BOT=my_car.py"
title SSAFY RACE bot - %BOT%
echo [%BOT%] The simulator must already be open. Stop with Ctrl+C
echo.
"%PY%" "%BOT%"
echo.
echo Bot finished (exit code %ERRORLEVEL%)
pause
