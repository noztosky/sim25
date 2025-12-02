@echo off
setlocal
REM UnrealVersionSelector 경로를 레지스트리에서 찾지 말고, 설치 위치를 직접 사용
REM 필요 시 아래 경로를 사용 중인 Unreal 설치에 맞게 수정
set "UE_VS=C:\Program Files (x86)\Epic Games\Launcher\Engine\Binaries\Win64\UnrealVersionSelector.exe"

if not exist "%UE_VS%" (
    echo [ERROR] UnrealVersionSelector.exe not found at:
    echo         %UE_VS%
    echo 경로를 GenerateProjectFiles.bat 안에서 UE_VS 변수로 수정해 주세요.
    exit /b 1
)

for %%f in (*.uproject) do (
    echo Generating files for %%f
    "%UE_VS%" /projectfiles "%cd%\%%f"
)

