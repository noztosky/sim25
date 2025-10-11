@echo off
setlocal

REM Build test_client.exe only

pushd %~dp0

where cl >nul 2>&1
if errorlevel 1 (
  echo [error] cl not found. Please run from "x64 Native Tools Command Prompt for VS 2022".
  popd & endlocal & exit /b 1
)

set OUT=test_client.exe
if exist %OUT% del /q %OUT% >nul 2>&1

echo Building %OUT% ...
cl /nologo /utf-8 /EHsc /std:c++17 test_client.cpp winmm.lib /Fe:%OUT% > build_test.log 2>&1

if exist %OUT% (
  echo test: SUCCESS
  echo log: build_test.log
) else (
  echo test: FAIL
  echo See build_test.log for details.
)

popd
endlocal


