@echo off
setlocal enabledelayedexpansion
REM ===== STEP 2: ArduPilot SITL — SYNCHRONOUS: waits here until linked =====
REM Run AFTER the drone is visible in UE (1_UE.bat).
REM Starts (or bounces) SITL, then blocks until the sensor link is confirmed.
REM Auto-bounces on the UDP deadlock signature. Exits when the sim is flyable.

wsl -e bash -c "pgrep -f ap_sitl_watchdog.sh >/dev/null" >nul 2>&1
if not errorlevel 1 (
  echo Watchdog already running - bouncing arducopter...
  wsl -e bash -c "> ~/sitl_watchdog.log; pkill -9 -x arducopter" >nul 2>&1
) else (
  echo Starting SITL watchdog...
  wsl -e bash -c "> ~/sitl_watchdog.log; exit 0" >nul 2>&1
  start "ArduPilot SITL (K20)" wsl -e bash /mnt/d/xlab/sim25/x_memory2/build/ap_sitl_watchdog.sh
)

echo Waiting for boot + sensor link (normally 30-110 s)...
REM 3-state check every 15 s, up to ~5 min:
REM   log quiet ^>12 s                    -> linked, done
REM   fresh writes ending in "No sensor" -> UDP deadlock -> bounce (max 3)
REM   fresh writes of anything else      -> still booting, keep waiting
set /a BOUNCES=0
set /a CHECKS=0
:linkcheck
ping 127.0.0.1 -n 16 >nul
set /a CHECKS+=1
echo   ...check !CHECKS! (elapsed ~%TIME%)
if !CHECKS! gtr 20 (
  echo WARN: link not confirmed after ~5 min. See ~/sitl_watchdog.log in WSL.
  goto done
)
wsl -e bash -c "age=$(( $(date +%%s) - $(stat -c %%Y ~/sitl_watchdog.log 2>/dev/null || echo 0) )); if [ $age -gt 12 ]; then exit 0; elif tail -1 ~/sitl_watchdog.log 2>/dev/null | grep -q 'No sensor'; then exit 1; else exit 2; fi" >nul 2>&1
if errorlevel 2 goto linkcheck
if errorlevel 1 goto bounce
goto linked
:bounce
set /a BOUNCES+=1
if !BOUNCES! gtr 3 (
  echo WARN: still deadlocked after 3 bounces. Is UE actually running?
  goto done
)
echo   UDP deadlock detected - bouncing SITL (attempt !BOUNCES!/3)...
wsl -e bash -c "pkill -9 -x arducopter" >nul 2>&1
goto linkcheck

:linked
echo.
echo  ================================================
echo   LINKED - sim is running. Connect now:
echo     GcsX / Mission Planner : UDP 14550
echo     SBUS joystick          : 3_JOYSTICK.bat
echo  ================================================

:done
endlocal
pause
