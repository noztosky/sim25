@echo off
setlocal enabledelayedexpansion

REM Build test_speed.cpp (SHM speed test) with MSVC
REM Usage: build_test.bat [Debug|Release]

set CONFIG=%1
if "%CONFIG%"=="" set CONFIG=Release

set RUNTIME_FLAG=/MD
set OPT_FLAGS=/O2
if /I "%CONFIG%"=="Debug" (
  set RUNTIME_FLAG=/MDd
  set OPT_FLAGS=/Od /Zi
)

if not exist build mkdir build
pushd build

cl /nologo /utf-8 /EHsc /std:c++17 %RUNTIME_FLAG% %OPT_FLAGS% ..\test_speed.cpp /Fe:..\test_speed.exe winmm.lib
if errorlevel 1 (
  echo Build failed.
  popd
  exit /b 1
)

echo.
echo Build succeeded: %~dp0test_speed.exe
popd
exit /b 0


