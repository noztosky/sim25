@echo off
setlocal

REM Build xsim_client.exe only (standalone)
REM Usage: run from Developer Command Prompt so cl/link are available

pushd %~dp0

REM Ensure MSVC build tools are available
where cl >nul 2>&1
if errorlevel 1 (
  echo [info] cl not found in PATH. Trying to initialize VS 2022 environment...
  set "CAND1=%ProgramFiles(x86)%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
  set "CAND2=%ProgramFiles(x86)%\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat"
  set "CAND3=%ProgramFiles(x86)%\Microsoft Visual Studio\2022\Enterprise\Common7\Tools\VsDevCmd.bat"
  if exist "%CAND1%" (
    call "%CAND1%" -arch=x64 -host_arch=x64
    where cl >nul 2>&1 && goto :env_ready
  )
  if exist "%CAND2%" (
    call "%CAND2%" -arch=x64 -host_arch=x64
    where cl >nul 2>&1 && goto :env_ready
  )
  if exist "%CAND3%" (
    call "%CAND3%" -arch=x64 -host_arch=x64
    where cl >nul 2>&1 && goto :env_ready
  )
  echo [error] Visual Studio 개발자 명령 프롬프트 환경을 찾지 못했습니다.
  echo        "x64 Native Tools Command Prompt for VS 2022"에서 b.bat을 실행하세요.
  popd
  endlocal
  exit /b 1
)
:env_ready

set SRC=xsim_client.cpp
set OUT=xsim_client.exe

if exist %OUT% del /q %OUT% >nul 2>&1

echo Building %OUT% ...
cl /nologo /utf-8 /EHsc /std:c++17 "%SRC%" winmm.lib /Fe:%OUT% > build_client.log 2>&1

if exist %OUT% (
  echo client: SUCCESS
  echo log: build_client.log
) else (
  echo client: FAIL
  echo See build_client.log for details.
)

popd
endlocal


