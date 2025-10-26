@echo off
echo ========================================
echo  Python AirSim Client
echo ========================================
echo.

REM Python 버전 확인
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python이 설치되지 않았습니다!
    echo         https://www.python.org/downloads/ 에서 설치하세요.
    pause
    exit /b 1
)

echo [INFO] Python이 설치되어 있습니다.
echo.

REM 서버 실행 확인
echo [CHECK] AirSim 서버가 실행 중인지 확인하세요...
echo         만약 실행되지 않았다면, 먼저 xsim_server.exe를 실행하세요!
echo.
timeout /t 3 >nul

REM 간단한 버전과 완전한 버전 선택
echo 어떤 클라이언트를 실행하시겠습니까?
echo   [1] 간단한 버전 (python_simple_client.py) - 추천
echo   [2] 완전한 버전 (python_client.py) - 통계/로깅 포함
echo.
choice /c 12 /n /m "선택 (1 또는 2): "

if errorlevel 2 goto full_client
if errorlevel 1 goto simple_client

:simple_client
echo.
echo [RUN] 간단한 클라이언트 실행 중...
echo.
python python_simple_client.py
goto end

:full_client
echo.
echo [RUN] 완전한 클라이언트 실행 중...
echo.
python python_client.py
goto end

:end
echo.
echo [DONE] 클라이언트 종료
pause

