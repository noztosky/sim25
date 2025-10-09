@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1

where cl.exe >nul 2>&1
if errorlevel 1 (
  echo msvc: NOT FOUND
  exit /b 1
)

del /q *.exe

cl.exe /nologo /EHsc airsim_server.cpp > build_server.log 2>&1
set SERVER_RC=%ERRORLEVEL%

cl.exe /nologo /EHsc airsim_client.cpp > build_client.log 2>&1
set CLIENT_RC=%ERRORLEVEL%

if %SERVER_RC%==0 (
  echo server: SUCCESS
  del /q build_server.log >nul 2>&1
) else (
  echo server: FAILED (see build_server.log)
)

if %CLIENT_RC%==0 (
  echo client: SUCCESS
  del /q build_client.log >nul 2>&1
) else (
  echo client: FAILED (see build_client.log)
)

if exist server.exe del /q server.exe >nul 2>&1
if exist client.exe del /q client.exe >nul 2>&1
if exist airsim_server.exe move /y airsim_server.exe server.exe >nul 2>&1
if exist airsim_client.exe move /y airsim_client.exe client.exe >nul 2>&1

del /q *.obj

exit /b 0
