@echo off
if not exist build mkdir build

:: Get absolute path to this script's directory
set ROOT_DIR=%~dp0
:: Resolve AirLib path (parent of current dir + AirLib)
pushd %ROOT_DIR%..\AirLib
set AIRLIB_PATH=%CD%
popd

echo AirLib Path: %AIRLIB_PATH%

set AIRLIB_INC=%AIRLIB_PATH%\include
set AIRLIB_LIB=%AIRLIB_PATH%\lib\x64\Release
set RPC_LIB=%AIRLIB_PATH%\deps\rpclib\lib\x64\Release
set EIGEN_INC=%AIRLIB_PATH%\deps\eigen3
set MAVLINK_INC=%AIRLIB_PATH%\deps\MavLinkCom\include
set MAVLINK_LIB=%AIRLIB_PATH%\deps\MavLinkCom\lib\x64\Release

if not exist "%AIRLIB_INC%" (
    echo ERROR: AirLib include directory not found at %AIRLIB_INC%
    exit /b 1
)

:: Compiler Flags
:: /EHsc for exceptions, /MD for multithreaded DLL runtime
set CFLAGS=/EHsc /std:c++17 /MD /DNOMINMAX /I. /I"%AIRLIB_INC%" /I"%AIRLIB_PATH%\deps\rpclib\include" /I"%MAVLINK_INC%" /I"%EIGEN_INC%"

:: Main Build
echo Building SIL_App...
echo CFLAGS=%CFLAGS%
cl %CFLAGS% apps\SIL_App.cpp "%AIRLIB_LIB%\AirLib.lib" "%RPC_LIB%\rpc.lib" "%MAVLINK_LIB%\MavLinkCom.lib" Advapi32.lib ws2_32.lib User32.lib winmm.lib /Fe:build\SIL_App.exe /link /NODEFAULTLIB:LIBCMT

if %errorlevel% neq 0 exit /b %errorlevel%
echo Build Success!
