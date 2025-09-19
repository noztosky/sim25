@echo off
setlocal

REM ===== Clean UE (Blocks) =====
set "ROOT_DIR=%~dp0"
set "PROJECT_DIR=%ROOT_DIR%Unreal\Environments\Blocks"
set "UPROJECT=%PROJECT_DIR%\Blocks.uproject"

echo ===== Clean UE (Blocks) =====

REM Check if project exists
if not exist "%UPROJECT%" (
  echo FAIL: Blocks.uproject not found: %UPROJECT%
  exit /b 1
)

REM Clean intermediate files
echo Cleaning intermediate files...
if exist "%PROJECT_DIR%\Binaries" rmdir /s /q "%PROJECT_DIR%\Binaries" >nul 2>&1
if exist "%PROJECT_DIR%\Intermediate" rmdir /s /q "%PROJECT_DIR%\Intermediate" >nul 2>&1
if exist "%PROJECT_DIR%\Saved" rmdir /s /q "%PROJECT_DIR%\Saved" >nul 2>&1

REM Clean plugin intermediate files
set "PLUGIN_DIR=%PROJECT_DIR%\Plugins\AirSim"
if exist "%PLUGIN_DIR%\Binaries" rmdir /s /q "%PLUGIN_DIR%\Binaries" >nul 2>&1
if exist "%PLUGIN_DIR%\Intermediate" rmdir /s /q "%PLUGIN_DIR%\Intermediate" >nul 2>&1

echo OK: Cleaned caches and outputs.

REM Regenerate project files
echo Regenerating project files...
set "ENGINE_ROOT=C:\Program Files\Epic Games\UE_4.27"
set "UBT=%ENGINE_ROOT%\Engine\Binaries\DotNET\UnrealBuildTool.exe"
if exist "%UBT%" (
  "%UBT%" -projectfiles -project="%UPROJECT%" -game -rocket -progress >nul 2>&1
  echo OK: Regenerated project files.
) else (
  echo WARNING: UnrealBuildTool not found, skipping project file regeneration
)

echo DONE: Clean complete.
exit /b 0
