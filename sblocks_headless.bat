@echo off
setlocal

REM sblocks_headless.bat <hz> [clocktype] — same as sblocks.bat but NO rendering (-nullrhi)
REM Used for fair RTF comparison vs PX4 headless (isolates render overhead).

set "ENGINE_ROOT=C:\Program Files\Epic Games\UE_4.27"
set "UPROJECT=%~dp0Unreal\Environments\Blocks\Blocks.uproject"
set "SETTINGS_JSON=%~dp0settings.json"
set "PROJECT_SETTINGS_JSON=%~dp0Unreal\Environments\Blocks\settings.json"

if "%1"=="" goto :RunSimulation

set "CLOCK_PARAM=%2"
set /a HZ=%1
set /a PERIOD=1000000000 / HZ
echo Dynamic config: Setting PhysicsLoopPeriod to %1 Hz (%PERIOD% ns)...

powershell -NoProfile -Command "$p = '%SETTINGS_JSON%'; if (Test-Path $p) { $c = Get-Content $p -Raw -ErrorAction SilentlyContinue; $json = $null; if (![string]::IsNullOrEmpty($c)) { $json = try { $c | ConvertFrom-Json -ErrorAction Stop } catch { $null } }; if ($null -eq $json) { $json = [pscustomobject]@{} }; $json | Add-Member -NotePropertyName 'PhysicsLoopPeriod' -NotePropertyValue ([long]%PERIOD%) -Force; $json | ConvertTo-Json -Depth 100 | Out-File $p -Encoding utf8 }"
powershell -NoProfile -Command "$p = '%PROJECT_SETTINGS_JSON%'; if (Test-Path $p) { $c = Get-Content $p -Raw -ErrorAction SilentlyContinue; $json = $null; if (![string]::IsNullOrEmpty($c)) { $json = try { $c | ConvertFrom-Json -ErrorAction Stop } catch { $null } }; if ($null -eq $json) { $json = [pscustomobject]@{} }; $json | Add-Member -NotePropertyName 'PhysicsLoopPeriod' -NotePropertyValue ([long]%PERIOD%) -Force; $json | ConvertTo-Json -Depth 100 | Out-File $p -Encoding utf8 }"

:RunSimulation
echo Starting Blocks simulation HEADLESS (-nullrhi, no rendering)...
start "" "%ENGINE_ROOT%\Engine\Binaries\Win64\UE4Editor.exe" "%UPROJECT%" -game -nullrhi -nosound -log
