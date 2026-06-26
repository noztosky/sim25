@echo off
:: Builds the standalone scheduler/timing test tools (no AirLib, no SHM).
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
if not exist build mkdir build

set CFLAGS=/nologo /EHsc /std:c++17 /O2 /DNOMINMAX /I.

for %%F in (tick_test tick_test2 wait_test client_speed jitter_bench jitter_bench_shm) do (
    echo Building %%F...
    cl %CFLAGS% apps\%%F.cpp /Fe:build\%%F.exe /Fo:build\%%F.obj >nul
    if errorlevel 1 (
        echo   FAILED: %%F
        exit /b 1
    )
)
echo All tests built into build\
