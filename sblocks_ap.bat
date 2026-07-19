@echo off
setlocal enabledelayedexpansion
REM ===== One-stop ArduPilot mode launcher (state-driven, self-healing) =====
REM  1) swaps in settings_ardupilot.json (ArduCopter vehicle, lockstep)
REM  2) starts the Blocks sim and WAITS until its RPC port 41451 is actually bound
REM  3) starts ArduPilot SITL (WSL watchdog window - auto-restarts on crash)
REM  4) verifies the sensor link; if the UE<->SITL UDP deadlock happens,
REM     bounces arducopter automatically (up to 3 times)
REM  Restore SIL_App mode:  git restore settings.json Unreal\Environments\Blocks\settings.json
REM                          then sblocks.bat 1000
REM  NOTE: uses "ping -n" for sleeping because "timeout /t" dies without a console stdin.
set "ROOT=%~dp0"

if not exist "%ROOT%settings_ardupilot.json" (
  echo FAIL: settings_ardupilot.json not found
  exit /b 1
)

echo [1/4] Applying ArduPilot settings...
copy /y "%ROOT%settings_ardupilot.json" "%ROOT%settings.json" >nul
copy /y "%ROOT%settings_ardupilot.json" "%ROOT%Unreal\Environments\Blocks\settings.json" >nul

echo [2/4] Killing previous instances (UE + SITL)...
taskkill /F /IM UE4Editor.exe >nul 2>&1
taskkill /F /IM SIL_App.exe >nul 2>&1
wsl -e bash -c "pkill -f ap_sitl_watchdog.sh 2>/dev/null; pkill -9 -x arducopter 2>/dev/null; > ~/sitl_watchdog.log; exit 0" >nul 2>&1
ping 127.0.0.1 -n 9 >nul

echo        Starting Blocks sim (UE must bind RPC 41451 before SITL:
echo        WSL mirrored networking can silently seize 41451 otherwise)...
start "" "C:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe" "%ROOT%Unreal\Environments\Blocks\Blocks.uproject" -game -windowed -resx=1280 -resy=720 -log -settings="%ROOT%settings.json"

echo [3/4] Waiting for UE RPC port 41451 (up to ~3 min)...
set /a RPC_TRIES=0
:wait_rpc
ping 127.0.0.1 -n 4 >nul
netstat -ano | findstr ":41451" | findstr "LISTENING" >nul
if not errorlevel 1 goto rpc_up
set /a RPC_TRIES+=1
if !RPC_TRIES! lss 60 goto wait_rpc
echo FAIL: UE never bound 41451. Check the UE window / Blocks.log.
exit /b 1
:rpc_up
echo        RPC up. Giving the level + vehicle ~20 s to spawn...
ping 127.0.0.1 -n 21 >nul

echo [4/4] Starting ArduPilot SITL (watchdog window)...
start "ArduPilot SITL (K20)" wsl -e bash /mnt/d/xlab/sim25/x_memory2/build/ap_sitl_watchdog.sh

REM ---- link verification, 3-state (checked every 15 s, up to ~4 min):
REM        quiet log (>12 s no writes)            -> linked, done
REM        fresh writes ending in "No sensor..."  -> UDP deadlock -> bounce SITL
REM        fresh writes of anything else          -> still booting, keep waiting
REM      (a booting SITL prints for 20-30 s; killing it on a timer would
REM       murder healthy boots forever - only bounce on the deadlock signature)
set /a BOUNCES=0
set /a CHECKS=0
:linkcheck
ping 127.0.0.1 -n 16 >nul
set /a CHECKS+=1
if !CHECKS! gtr 16 (
  echo WARN: link not confirmed after ~4 min. See ~/sitl_watchdog.log in WSL.
  goto done
)
wsl -e bash -c "age=$(( $(date +%%s) - $(stat -c %%Y ~/sitl_watchdog.log 2>/dev/null || echo 0) )); if [ $age -gt 12 ]; then exit 0; elif tail -1 ~/sitl_watchdog.log 2>/dev/null | grep -q 'No sensor'; then exit 1; else exit 2; fi" >nul 2>&1
if errorlevel 2 goto linkcheck
if errorlevel 1 goto bounce
goto linked
:bounce
set /a BOUNCES+=1
if !BOUNCES! gtr 3 (
  echo WARN: still deadlocked after 3 bounces. See ~/sitl_watchdog.log in WSL.
  goto done
)
echo        UDP deadlock detected - bouncing SITL (attempt !BOUNCES!/3)...
wsl -e bash -c "pkill -9 -x arducopter" >nul 2>&1
goto linkcheck
:linked
echo        Sensor link OK - lockstep running.

:done
echo.
echo ================================================================
echo  Ground station:  Mission Planner - TCP 127.0.0.1 : 5760
echo                   (2nd GCS: 5762, 3rd: 5763)
echo  SITL logs (.BIN): \\wsl$\Ubuntu\home\nodes\ardupilot\logs\
echo  Close the "ArduPilot SITL (K20)" window to stop SITL.
echo  If the sim ever FREEZES: wsl pkill -9 -x arducopter   (auto-restarts)
echo ================================================================
endlocal
