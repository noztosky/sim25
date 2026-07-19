@echo off
REM ===== Sim frozen? This fixes the UE-SITL UDP deadlock in one shot. =====
REM Kills arducopter; the watchdog auto-restarts it in ~3 s and it relinks.
REM (Parameters are safe - they live in the WSL eeprom.)
echo Bouncing arducopter...
wsl -e bash -c "pkill -9 -x arducopter" >nul 2>&1
echo Done. Give it ~30-80 s to boot and relink.
echo If STILL frozen after that: UE itself crashed (GPU hang) - rerun 1_UE.bat
echo then 2_SITL.bat.
