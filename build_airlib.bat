@echo off
setlocal

if "%1"=="--no-log-wrap" goto :Main

set "LOG_DIR=%~dp0logs\build"
if not exist "%LOG_DIR%" mkdir "%LOG_DIR%"

set "TIMESTAMP=%date:~0,4%%date:~5,2%%date:~8,2%_%time:~0,2%%time:~3,2%%time:~6,2%"
set "TIMESTAMP=%TIMESTAMP: =0%"
set "LOG_FILE=%LOG_DIR%\build_airlib_%TIMESTAMP%.log"

echo Logging build_airlib to %LOG_FILE%
powershell -NoProfile -Command "& '%~f0' --no-log-wrap 2>&1 | Tee-Object -FilePath '%LOG_FILE%'"
exit /b %errorlevel%

:Main
shift

echo Building AirLib Release x64...
msbuild "%~dp0AirLib\AirLib.vcxproj" /property:Configuration=Release /property:Platform=x64
