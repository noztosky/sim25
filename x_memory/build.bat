@echo off
setlocal ENABLEDELAYEDEXPANSION
pushd %~dp0
set TARGETS="airsim_server.cpp|airsim_server.exe|server" "airsim_client.cpp|airsim_client.exe|client" "flight_client.cpp|flight.exe|flight" "xsim_server.cpp|xsim_server.exe|xsim_server" "xsim_client.cpp|xsim_client.exe|xsim_client"

where cl.exe >nul 2>&1 || call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
where cl.exe >nul 2>&1 || (echo msvc: NOT FOUND & popd & endlocal & exit /b 1)

for %%E in (server.exe client.exe flight.exe airsim_server.exe airsim_client.exe) do if exist %%E del /q %%E >nul 2>&1
del /q *.obj >nul 2>&1

for %%X in (%TARGETS%) do (
  for /f "tokens=1-3 delims=^|" %%a in ("%%~X") do (
    set TAG=%%c
    cl /nologo /O2 /MD /EHsc /std:c++17 "%%a" /Fe:"%%b" /I. /link /INCREMENTAL:NO > "build_!TAG!.log" 2>&1
    if errorlevel 1 (
      echo !TAG!: FAILED (see build_!TAG!.log)
    )
    if not errorlevel 1 (
      if /I "!TAG!"=="flight" (
        echo flight: success
      ) else (
        echo !TAG!: SUCCESS
      )
      del /q "build_!TAG!.log" >nul 2>&1
    )
  )
)

for %%Z in (server client) do (
  if exist airsim_%%Z.exe (
    if exist %%Z.exe del /q %%Z.exe >nul 2>&1
    move /y airsim_%%Z.exe %%Z.exe >nul 2>&1
  )
)

del /q *.obj >nul 2>&1
popd
endlocal
exit /b 0
