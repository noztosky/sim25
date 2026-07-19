@echo off
setlocal
set "ROOT=%~dp0"

echo ================================================================
echo   EFT Z30 - GUI + SIL_App launcher
echo ----------------------------------------------------------------
echo   NOTE: the simulator must be running first.
echo         Start it with:  sblocks.bat 1000   (in d:\xlab\sim25)
echo ================================================================
echo.

REM --- kill any leftover SIL_App so it can bind UDP 5005 / SHM cleanly ---
taskkill /F /IM SIL_App.exe >nul 2>&1

REM --- optional args: run_gui.bat [hz] [profile]   (default: 1000 z30) ---
set "HZ=%~1"
if "%HZ%"=="" set "HZ=1000"
set "PROFILE=%~2"
if "%PROFILE%"=="" set "PROFILE=z30"

echo Starting SIL_App (%HZ% %PROFILE%) in a new window...
start "SIL_App (Z30)" /D "%ROOT%build" "%ROOT%build\SIL_App.exe" %HZ% %PROFILE%

REM --- give SIL_App a moment to connect before the GUI sends commands ---
timeout /t 2 /nobreak >nul

echo Starting GUI...
cd /d "%ROOT%"
python apps\sendcmd_gui.py

echo.
echo GUI closed. (SIL_App is still running in its own window - close it or Ctrl+C there.)
endlocal
