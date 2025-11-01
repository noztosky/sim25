@echo off
setlocal enabledelayedexpansion

REM Build rpc_client.cpp using MSVC, linking against AirLib and rpclib
REM Usage: build.bat [Debug|Release]

set CONFIG=%1
if "%CONFIG%"=="" set CONFIG=Release

REM Resolve repo root (two levels up from this script's directory)
set AIRSIM_ROOT=%~dp0..\..\

REM Configure runtime/link options to match AirLib (/MD in Release, /MDd in Debug)
set RUNTIME_FLAG=/MD
set OPT_FLAGS=/O2
set LINK_OPTS=
if /I "%CONFIG%"=="Debug" (
  set RUNTIME_FLAG=/MDd
  set OPT_FLAGS=/Od /Zi
) else (
  set OPT_FLAGS=/O2 /GL
  set LINK_OPTS=/LTCG
)

set INCLUDES=/I "%AIRSIM_ROOT%AirLib\include" ^
  /I "%AIRSIM_ROOT%AirLib\deps\eigen3" ^
  /I "%AIRSIM_ROOT%MavLinkCom\include" ^
  /I "%AIRSIM_ROOT%MavLinkCom\common_utils" ^
  /I "%AIRSIM_ROOT%external\rpclib\rpclib-2.3.0\include" ^
  /I "%AIRSIM_ROOT%external\rpclib\include"

set LIBS=/link /LIBPATH:"%AIRSIM_ROOT%AirLib\lib\x64\%CONFIG%" AirLib.lib ^
  /LIBPATH:"%AIRSIM_ROOT%external\rpclib\rpclib-2.3.0\build\%CONFIG%" rpc.lib ^
  Ws2_32.lib %LINK_OPTS%

set OUT=%~dp0rpc_client.exe

if not exist build mkdir build
pushd build

cl /nologo /utf-8 /EHsc /std:c++17 %RUNTIME_FLAG% %OPT_FLAGS% %INCLUDES% ..\rpc_client.cpp /Fe:..\rpc_client.exe %LIBS%
if errorlevel 1 (
  echo Build failed.
  popd
  exit /b 1
)

echo.
echo Build succeeded: %OUT%
popd
exit /b 0
