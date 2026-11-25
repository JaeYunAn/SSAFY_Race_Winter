@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "PY=%~dp0.venv\Scripts\python.exe"
if exist "%PY%" goto :run
echo Run setup_python.bat first.
pause
exit /b 1
:run
echo Checking my_car.py without the simulator (real drive_controller.py + fake physics, 1-3 min)
"%PY%" offline_test\run_all.py
pause
