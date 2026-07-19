@echo off
REM ===== STEP 1: UE sim (run FIRST, wait until the drone is visible) =====
set "ROOT=D:\xlab\sim25"
copy /y "%ROOT%\settings_ardupilot.json" "%ROOT%\settings.json" >nul
copy /y "%ROOT%\settings_ardupilot.json" "%ROOT%\Unreal\Environments\Blocks\settings.json" >nul
taskkill /F /IM UE4Editor.exe >nul 2>&1
taskkill /F /IM SIL_App.exe >nul 2>&1
start "" "C:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe" "%ROOT%\Unreal\Environments\Blocks\Blocks.uproject" -game -windowed -resx=1280 -resy=720 -log -settings="%ROOT%\settings.json"
echo.
echo  UE starting... wait until the level + drone are VISIBLE (~40-60 s),
echo  then run 2_SITL.bat
echo  (with lockstep the screen stays FROZEN until SITL connects - normal)
