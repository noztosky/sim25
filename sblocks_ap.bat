@echo off
setlocal
REM ===== One-stop ArduPilot mode launcher =====
REM  1) swaps in settings_ardupilot.json (ArduCopter vehicle, EFTZ30 frame, lockstep)
REM  2) starts the Blocks sim (kills any previous UE instance first - RPC port!)
REM  3) starts ArduPilot SITL in WSL (watchdog window - auto-restarts on crash)
REM  Restore SIL_App mode:  git restore settings.json Unreal\Environments\Blocks\settings.json
REM                          then sblocks.bat 1000
set "ROOT=%~dp0"

if not exist "%ROOT%settings_ardupilot.json" (
  echo FAIL: settings_ardupilot.json not found
  exit /b 1
)

echo [1/3] Applying ArduPilot settings (VehicleType=ArduCopter, Model=EFTZ30)...
copy /y "%ROOT%settings_ardupilot.json" "%ROOT%settings.json" >nul
copy /y "%ROOT%settings_ardupilot.json" "%ROOT%Unreal\Environments\Blocks\settings.json" >nul

echo [2/3] Starting Blocks sim FIRST (UE must bind RPC 41451 before WSL comes up:
echo        WSL mirrored networking can silently seize 41451 - Linux ephemeral range)...
taskkill /F /IM UE4Editor.exe >nul 2>&1
taskkill /F /IM SIL_App.exe >nul 2>&1
REM give the old UE instance time to fully release its ports (bind error otherwise)
timeout /t 8 /nobreak >nul
REM -settings pins the config regardless of the process working directory
REM (otherwise AirSim can silently fall back to Documents\AirSim\settings.json)
start "" "C:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe" "%ROOT%Unreal\Environments\Blocks\Blocks.uproject" -game -windowed -resx=1280 -resy=720 -log -settings="%ROOT%settings.json"

echo [3/3] Waiting 75s for UE to bind RPC, then starting ArduPilot SITL (WSL watchdog)...
timeout /t 75 /nobreak >nul
wsl -e bash -c "pkill -f ap_sitl_watchdog.sh 2>/dev/null; pkill -f bin/arducopter 2>/dev/null; exit 0" >nul 2>&1
start "ArduPilot SITL (Z30)" wsl -e bash /mnt/d/xlab/sim25/x_memory2/build/ap_sitl_watchdog.sh

echo.
echo ================================================================
echo  Sim loading (~40-60s). SITL auto-restarts if it crashes.
echo  NOTE: with lockstep the sim looks FROZEN until SITL connects.
echo.
echo  Ground station:  Mission Planner - TCP 127.0.0.1 : 5760
echo                   (2nd GCS: 5762, 3rd: 5763)
echo  SITL logs (.BIN): \\wsl$\Ubuntu\home\nodes\ardupilot\logs\
echo  Close the "ArduPilot SITL (Z30)" window to stop SITL.
echo ================================================================
endlocal
