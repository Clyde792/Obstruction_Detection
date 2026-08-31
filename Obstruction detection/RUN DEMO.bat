@echo off
REM ============================================================
REM  Walkway Guard - one-click demo launcher.
REM
REM  Written for operators who have never used a command prompt:
REM  every failure below must print a plain-English instruction
REM  and PAUSE, never a Python traceback and never a window that
REM  vanishes before it can be read.
REM
REM  Uses goto-based flow rather than if/else blocks on purpose --
REM  setting a variable inside a parenthesised block needs delayed
REM  expansion to read back correctly, which is a classic silent
REM  bug in .bat files.
REM ============================================================
setlocal
title Walkway Guard - Demo
cd /d "%~dp0"
color 0F
cls

echo.
echo   ============================================================
echo                        W A L K W A Y   G U A R D
echo   ============================================================
echo.

if not exist ".venv\Scripts\python.exe" goto no_program

echo   Looking for the controller board...
echo.

set BOARD=
for /f "usebackq delims=" %%i in (`.venv\Scripts\python.exe -c "import serial.tools.list_ports as lp; ports=[p.device for p in lp.comports() if (p.vid,p.pid)==(0x303A,0x1001)]; print(ports[0] if ports else '')" 2^>nul`) do set BOARD=%%i

if "%BOARD%"=="" goto no_board
goto found_board


:no_board
echo   [!]  No controller board found.
echo.
echo        The lamps will NOT light up without it.
echo.
echo        Try this:
echo          1. Plug the board into the laptop with a USB cable.
echo          2. If it is already plugged in, try a DIFFERENT cable.
echo             Many charging cables carry power but no data, and
echo             the board looks dead when you use one.
echo          3. Close this window and double-click RUN DEMO again.
echo.
choice /c YN /n /m "        Continue anyway with the screen only?  [Y/N] "
if errorlevel 2 goto quit
echo.
set PORTARG=
goto launch


:found_board
echo   [OK] Board found on %BOARD%
echo.
set PORTARG=--port %BOARD%
goto launch


:launch
REM Work out this laptop's address on the local network so the operator can
REM read it straight off the screen. Opening a UDP socket toward a public
REM address picks the interface Windows would actually route through, without
REM sending any traffic -- more reliable than parsing ipconfig, which lists
REM virtual adapters (VPN, WSL, VirtualBox) that are not the one in use.
set LANIP=
for /f "usebackq delims=" %%i in (`.venv\Scripts\python.exe -c "import socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.settimeout(0.4);s.connect(('8.8.8.8',80));print(s.getsockname()[0]);s.close()" 2^>nul`) do set LANIP=%%i

echo   Starting up - please wait.
echo   Your web browser will open by itself.
echo.
echo   ------------------------------------------------------------
echo    TO WATCH ON A PHONE OR TABLET
echo.
if "%LANIP%"=="" goto no_ip
echo      Connect it to the SAME Wi-Fi as this laptop,
echo      then open this address in its web browser:
echo.
echo            http://%LANIP%:8000
echo.
goto after_ip

:no_ip
echo      This laptop is not on a network right now, so other
echo      devices cannot connect. Join a Wi-Fi network and
echo      restart to enable this.
echo.

:after_ip
echo   ------------------------------------------------------------
echo    KEEP THIS BLACK WINDOW OPEN.
echo    Closing it stops the demo and turns the camera off.
echo    To stop: close this window, or press Ctrl and C.
echo   ------------------------------------------------------------
echo.

REM Open the browser after a delay, from a separate minimised shell,
REM so the dashboard itself can keep the foreground window.
start "" /min cmd /c "timeout /t 14 /nobreak >nul & start "" http://127.0.0.1:8000"

.venv\Scripts\python.exe dashboard.py %PORTARG% --bind 0.0.0.0 --lock-exposure --fast

echo.
echo   ------------------------------------------------------------
echo    The demo has stopped.
echo   ------------------------------------------------------------
echo.
echo   If it closed on its own without you pressing Ctrl+C, the most
echo   likely cause is the camera being used by another app - close
echo   Zoom, Teams or Camera, then double-click RUN DEMO again.
echo.
pause
goto quit


:no_program
echo   [X]  This laptop has not been set up yet.
echo.
echo        The program files ^(.venv folder^) are missing. This is
echo        normal on a laptop that has just downloaded the project
echo        for the first time - they are too large to be included.
echo.
echo        Someone technical needs to run the one-time setup:
echo          open a Command Prompt in this folder and run
echo            python -m venv .venv
echo            .venv\Scripts\python.exe -m pip install ultralytics opencv-python pyserial platformio
echo.
echo        ^(If this laptop HAS been set up before, then this
echo         RUN DEMO file has been moved out of its folder -
echo         it must sit inside the "Obstruction detection" folder.^)
echo.
pause
goto quit


:quit
endlocal
