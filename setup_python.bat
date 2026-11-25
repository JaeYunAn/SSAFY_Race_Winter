@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title SSAFY RACE - Python 3.7 setup
echo ================= SSAFY RACE Python setup =================
echo  Creates a private Python 3.7 environment (.venv) in this folder,
echo  the same version as the contest server.
echo  Your other Python (3.12 etc.) and PATH are NOT changed.
echo ===========================================================

rem --- 1. SSAFY template files (DrivingInterface, examples) from this PC ---
if exist "DrivingInterface\drive_controller.py" goto :have_di
echo.
echo Copying DrivingInterface from the SSAFY template on this PC...
set "SRC="
for /f "delims=" %%F in ('where /r "%~dp0.." drive_controller.py 2^>nul ^| findstr /i /c:"Bot_Python"') do if not defined SRC set "SRC=%%~dpF"
if not defined SRC for /f "delims=" %%F in ('where /r "%~dp0..\.." drive_controller.py 2^>nul ^| findstr /i /c:"Bot_Python"') do if not defined SRC set "SRC=%%~dpF"
if not defined SRC goto :no_template
xcopy /E /I /Y /Q "%SRC%." "DrivingInterface" >nul
for %%F in ("%SRC%..") do set "BOTDIR=%%~fF"
for %%F in ("%SRC%..\..") do set "TPLDIR=%%~fF"
if not exist "my_car_template.py" if exist "%BOTDIR%\my_car.py" copy /Y "%BOTDIR%\my_car.py" "my_car_template.py" >nul
if not exist "my_car_basic.py" if exist "%TPLDIR%\1_Basic\my_car.py" copy /Y "%TPLDIR%\1_Basic\my_car.py" "my_car_basic.py" >nul
echo   from: %SRC%
if not exist "DrivingInterface\drive_controller.py" goto :no_template
:have_di

rem --- 2. Python 3.7 ---
set "PY37="
for /f "usebackq delims=" %%P in (`py -3.7 -c "import sys; print(sys.executable)" 2^>nul`) do set "PY37=%%P"
if defined PY37 if not exist "%PY37%" set "PY37="
if not defined PY37 if exist "%LOCALAPPDATA%\Programs\Python\Python37\python.exe" set "PY37=%LOCALAPPDATA%\Programs\Python\Python37\python.exe"
if not defined PY37 if exist "%ProgramFiles%\Python37\python.exe" set "PY37=%ProgramFiles%\Python37\python.exe"
if defined PY37 goto :have_py

echo.
echo Python 3.7 is not installed.
choice /c YN /m "Download Python 3.7.9 from python.org and install it for this user only"
if errorlevel 2 goto :manual
set "INST=%TEMP%\python-3.7.9-amd64.exe"
echo Downloading (about 25 MB)...
curl.exe -L --fail -o "%INST%" "https://www.python.org/ftp/python/3.7.9/python-3.7.9-amd64.exe"
if errorlevel 1 powershell -NoProfile -Command "[Net.ServicePointManager]::SecurityProtocol='Tls12'; Invoke-WebRequest -Uri 'https://www.python.org/ftp/python/3.7.9/python-3.7.9-amd64.exe' -OutFile '%INST%'"
if not exist "%INST%" goto :manual
echo Installing (1-2 minutes). PATH is not changed...
"%INST%" /quiet InstallAllUsers=0 PrependPath=0 Include_launcher=1 InstallLauncherAllUsers=0 Include_test=0 Include_doc=0 Include_tcltk=0
set "PY37=%LOCALAPPDATA%\Programs\Python\Python37\python.exe"
if not exist "%PY37%" goto :manual

:have_py
echo.
echo Using Python: %PY37%
if exist ".venv\Scripts\python.exe" goto :venv_ok
echo Creating .venv ...
"%PY37%" -m venv .venv
if not exist ".venv\Scripts\python.exe" goto :venv_fail

:venv_ok
".venv\Scripts\python.exe" -m pip install --upgrade pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto :pip_fail
".venv\Scripts\python.exe" -c "import sys, numpy, msgpackrpc, msgpack; from win32com.shell import shell; print('OK: Python', sys.version.split()[0], '/ numpy', numpy.__version__)"
if errorlevel 1 goto :pip_fail
echo.
echo ===== Ready =====
echo  1. ..\start_simulator.bat   start the simulator
echo  2. run_bot.bat              run the bot (my_car.py)
pause
exit /b 0

:manual
echo.
echo Automatic install failed. Download "Windows x86-64 executable installer" from the page below,
echo install it, then run this file again.
echo https://www.python.org/downloads/release/python-379/
start "" "https://www.python.org/downloads/release/python-379/"
pause
exit /b 1

:no_template
echo.
echo [ERROR] SSAFY template not found (MyCar_...\Template_Python\Bot_Python\DrivingInterface).
echo Put this workspace folder inside the folder that contains the unzipped MyCar_xxxx folder, then run again.
pause
exit /b 1

:venv_fail
echo Could not create .venv. Please capture the messages above.
pause
exit /b 1

:pip_fail
echo Package install or check failed. Please capture the messages above.
pause
exit /b 1
