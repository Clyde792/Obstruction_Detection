@echo off
REM One uninterrupted capture session. Do NOT tilt the laptop screen at any
REM point after the calibration step -- clicking and key presses are fine,
REM but moving the lid re-aims the camera and invalidates everything.
setlocal
set PY=.venv\Scripts\python.exe

echo.
echo ============================================================
echo  STEP 1 of 5 - CALIBRATE
echo  Click 4 corners of LANE A, press ENTER.
echo  Then 4 corners of LANE B, press ENTER.
echo  Use the trackpad. Do not tilt the screen.
echo ============================================================
%PY% lane_detect.py --calibrate --lock-exposure
if errorlevel 1 goto :fail

echo.
echo ============================================================
echo  STEP 2 of 5 - BASELINE
echo  Clear both lanes. Hands out of frame.
echo ============================================================
pause
%PY% lane_detect.py --baseline --lock-exposure
if errorlevel 1 goto :fail

echo.
echo ============================================================
echo  STEP 3 of 5 - RECORD "empty"  (20s)
echo  Lanes stay empty. Hands out of frame the whole time.
echo ============================================================
pause
%PY% replay.py record empty --seconds 20 --lock-exposure --overwrite --note "control, lanes empty"

echo.
echo ============================================================
echo  STEP 4 of 5 - RECORD "object-A"  (20s)
echo  Place the object in LANE A now, then hands out of frame.
echo ============================================================
pause
%PY% replay.py record object-A --seconds 20 --lock-exposure --overwrite --note "object sitting in lane A"

echo.
echo ============================================================
echo  STEP 5 of 5 - RECORD "passthrough"  (20s)
echo  Remove the object first. Once recording starts, sweep your
echo  hand through LANE A a few times, leaving nothing behind.
echo ============================================================
pause
%PY% replay.py record passthrough --seconds 20 --lock-exposure --overwrite --note "hand through lane A, nothing left"

echo.
echo ============================================================
echo  DONE. You can move the screen again now.
echo ============================================================
goto :eof

:fail
echo.
echo Step failed -- fix the error above and re-run this script.
