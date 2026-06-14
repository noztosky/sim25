@echo off
setlocal
set ROOT_DIR=%~dp0
cd /d %ROOT_DIR%

for /d %%i in ("%ProgramFiles%\Microsoft Visual Studio\2022\*") do (
    if exist "%%i\Common7\Tools\VsDevCmd.bat" (
        set "VSDEVCMD=%%i\Common7\Tools\VsDevCmd.bat"
        goto :found
    )
)
echo [error] VsDevCmd.bat not found.
exit /b 1

:found
call "%VSDEVCMD%" -arch=x64 -host_arch=x64

pushd %ROOT_DIR%..\AirLib
set AIRLIB_PATH=%CD%
popd

set CFLAGS=/EHsc /std:c++17 /MD /DNOMINMAX /I. /I"%AIRLIB_PATH%\include" /I"%AIRLIB_PATH%\deps\rpclib\include" /I"%AIRLIB_PATH%\deps\MavLinkCom\include" /I"%AIRLIB_PATH%\deps\eigen3"

cl %CFLAGS% apps\GetPos.cpp "%AIRLIB_PATH%\lib\x64\Release\AirLib.lib" "%AIRLIB_PATH%\deps\rpclib\lib\x64\Release\rpc.lib" "%AIRLIB_PATH%\deps\MavLinkCom\lib\x64\Release\MavLinkCom.lib" Advapi32.lib ws2_32.lib User32.lib /Fe:build\GetPos.exe /link /NODEFAULTLIB:LIBCMT
exit /b %errorlevel%
