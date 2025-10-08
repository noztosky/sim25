@echo off
REM Thin wrapper: delegate to original script under Unreal\Environments\Blocks\package.bat
setlocal
set ROOT_DIR=%~dp0
set BLOCKS_DIR=%ROOT_DIR%Unreal\Environments\Blocks

IF NOT EXIST "%BLOCKS_DIR%\package.bat" (
    echo "Blocks package script not found: %BLOCKS_DIR%\package.bat"
    echo "Expected repo layout under %ROOT_DIR%"
    exit /b 1
)

pushd "%BLOCKS_DIR%"
call package.bat %*
set EXITCODE=%ERRORLEVEL%
popd
exit /b %EXITCODE%
