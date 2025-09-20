@echo off
setlocal

REM ===== AirSim UE build (Blocks) =====
set "ROOT_DIR=%~dp0"
set "PROJECT_DIR=%ROOT_DIR%Unreal\Environments\Blocks"
set "UPROJECT=%PROJECT_DIR%\Blocks.uproject"
set "PLUGIN_XAIR=%ROOT_DIR%Unreal\Plugins\AirSim\Source\XAir"
set "BLOCKS_PLUGIN_DIR=%PROJECT_DIR%\Plugins\AirSim"

echo ===== copy pluugins ======
robocopy "d:\open\airsim\Unreal\Plugins\AirSim\Source\Vehicles\Multirotor" "d:\open\airsim\Unreal\Environments\Blocks\Plugins\AirSim\Source\Vehicles\Multirotor" /E /COPY:DAT /R:0 /W:0 /NFL /NDL /NP /NJH /NJS

echo ===== AirSim UE build (Blocks) =====

REM 1) Kill processes
echo Killing processes...
taskkill /f /im UE4Editor.exe >nul 2>&1
taskkill /f /im UBT.exe >nul 2>&1
taskkill /f /im MSBuild.exe >nul 2>&1

REM 1a) If Blocks plugin path is NOT a junction, sync sources; otherwise skip
for /f "tokens=*" %%A in ('powershell -NoProfile -Command "(Get-Item '%BLOCKS_PLUGIN_DIR%').Attributes -band [IO.FileAttributes]::ReparsePoint"') do set IS_JUNCTION=%%A
if /I "%IS_JUNCTION%"=="1" (
    echo Detected junction at %BLOCKS_PLUGIN_DIR% - skipping source sync
) else (
    if exist "%PROJECT_DIR%\update_from_git.bat" (
        echo Sync AirSim plugin source -> Blocks project ...
        call "%PROJECT_DIR%\update_from_git.bat" "%ROOT_DIR%"
    ) else (
        echo WARNING: update_from_git.bat not found in %PROJECT_DIR%
    )
)

REM 1b) Force rebuild: remove previous plugin binaries/intermediate in Blocks project
if exist "%BLOCKS_PLUGIN_DIR%\Binaries" rmdir /s /q "%BLOCKS_PLUGIN_DIR%\Binaries"
if exist "%BLOCKS_PLUGIN_DIR%\Intermediate" rmdir /s /q "%BLOCKS_PLUGIN_DIR%\Intermediate"

REM 1c) Sync settings.json to Blocks project
echo Sync settings.json -> Blocks project ...
if exist "%ROOT_DIR%Unreal\Environments\Blocks\settings.json" copy "%ROOT_DIR%Unreal\Environments\Blocks\settings.json" "%PROJECT_DIR%\settings.json" /Y >nul
if exist "%ROOT_DIR%settings.json" copy "%ROOT_DIR%settings.json" "%PROJECT_DIR%\settings.json" /Y >nul
echo OK: settings.json synced

REM 2) Check Blocks.uproject
if not exist "%UPROJECT%" (
  echo FAIL: Blocks.uproject not found: %UPROJECT%
  exit /b 1
)

REM 3) Hard-coded UE4.27 path
set "ENGINE_ROOT=C:\Program Files\Epic Games\UE_4.27"
set "BUILD_BAT=%ENGINE_ROOT%\Engine\Build\BatchFiles\Build.bat"
if not exist "%BUILD_BAT%" (
  echo FAIL: Build.bat not found at %BUILD_BAT%
  echo Please check UE4.27 installation path
  exit /b 1
)

REM 4) Build BlocksEditor (quiet)
echo Build: BlocksEditor ^(Development^|Win64^)
call "%BUILD_BAT%" "BlocksEditor" Win64 Development -project="%UPROJECT%" -NoHotReload -NoUBTMakefiles -progress -Quiet
if errorlevel 1 (
  echo FAIL: Build BlocksEditor
  exit /b 1
)

REM Optional: package game target (non-blocking)
REM call "%BUILD_BAT%" "Blocks" Win64 Development -project="%UPROJECT%" -NoHotReload -progress -Quiet >nul 2>&1

echo DONE: Blocks.uproject build complete.
echo xlabSim actor has been added to the simulation.
exit /b 0
