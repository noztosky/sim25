@echo off
setlocal

REM ===== Run UE (Blocks) =====
set "ROOT_DIR=%~dp0"
set "PROJECT_DIR=%ROOT_DIR%Unreal\Environments\Blocks"
set "UPROJECT=%PROJECT_DIR%\Blocks.uproject"

echo ===== Run UE (Blocks) =====

REM Check if project exists
if not exist "%UPROJECT%" (
  echo FAIL: Blocks.uproject not found: %UPROJECT%
  exit /b 1
)

REM Check if editor exists
set "ENGINE_ROOT=C:\Program Files\Epic Games\UE_4.27"
set "EDITOR=%ENGINE_ROOT%\Engine\Binaries\Win64\UE4Editor.exe"
if not exist "%EDITOR%" (
  echo FAIL: UE4Editor not found at %EDITOR%
  echo Please check UE4.27 installation path
  exit /b 1
)

REM Launch UE4 Editor with Blocks project
echo Launching UE4 Editor with Blocks project...
echo Project: %UPROJECT%
echo Editor: %EDITOR%

start "" "%EDITOR%" "%UPROJECT%"

echo DONE: UE4 Editor launched.
echo xlabSim actor should be visible in the simulation.
exit /b 0
