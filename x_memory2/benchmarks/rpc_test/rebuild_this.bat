@echo off
if not exist build mkdir build

:: Resolve AirLib path (relative to the original location for libs)
set AIRLIB_PATH=D:\open\airsim\AirLib
set AIRLIB_INC=%AIRLIB_PATH%\include
set AIRLIB_LIB=%AIRLIB_PATH%\lib\x64\Release
set RPC_LIB=%AIRLIB_PATH%\deps\rpclib\lib\x64\Release
set EIGEN_INC=%AIRLIB_PATH%\deps\eigen3
set MAVLINK_INC=%AIRLIB_PATH%\deps\MavLinkCom\include
set MAVLINK_LIB=%AIRLIB_PATH%\deps\MavLinkCom\lib\x64\Release

:: Compiler Flags (Point to LOCAL src folder)
set CFLAGS=/EHsc /std:c++17 /MD /DNOMINMAX /I./src /I"%AIRLIB_INC%" /I"%AIRLIB_PATH%\deps\rpclib\include" /I"%MAVLINK_INC%" /I"%EIGEN_INC%"

echo Building SIL_App from BACKUP src...
cl %CFLAGS% src\apps\SIL_App.cpp "%AIRLIB_LIB%\AirLib.lib" "%RPC_LIB%\rpc.lib" "%MAVLINK_LIB%\MavLinkCom.lib" Advapi32.lib ws2_32.lib User32.lib winmm.lib /Fe:build\SIL_App_RPC.exe /link /NODEFAULTLIB:LIBCMT

if %errorlevel% neq 0 (
    echo Build Failed!
) else (
    echo Build Success: build\SIL_App_RPC.exe
)
pause
