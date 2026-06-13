@echo off
setlocal

rem Check 64-bit Program Files first
for /d %%i in ("%ProgramFiles%\Microsoft Visual Studio\2022\*") do (
    if exist "%%i\Common7\Tools\VsDevCmd.bat" (
        set "VSDEVCMD=%%i\Common7\Tools\VsDevCmd.bat"
        goto :found
    )
)
for /d %%i in ("%ProgramFiles%\Microsoft Visual Studio\2019\*") do (
    if exist "%%i\Common7\Tools\VsDevCmd.bat" (
        set "VSDEVCMD=%%i\Common7\Tools\VsDevCmd.bat"
        goto :found
    )
)

rem Check 32-bit Program Files
for /d %%i in ("%ProgramFiles(x86)%\Microsoft Visual Studio\2022\*") do (
    if exist "%%i\Common7\Tools\VsDevCmd.bat" (
        set "VSDEVCMD=%%i\Common7\Tools\VsDevCmd.bat"
        goto :found
    )
)
for /d %%i in ("%ProgramFiles(x86)%\Microsoft Visual Studio\2019\*") do (
    if exist "%%i\Common7\Tools\VsDevCmd.bat" (
        set "VSDEVCMD=%%i\Common7\Tools\VsDevCmd.bat"
        goto :found
    )
)

echo [error] VsDevCmd.bat not found in standard paths.
exit /b 1

:found
echo Found Developer Environment: %VSDEVCMD%
call "%VSDEVCMD%" -arch=x64 -host_arch=x64
call build.bat
exit /b %errorlevel%
