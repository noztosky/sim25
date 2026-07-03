@echo off
setlocal

set "ENGINE_ROOT=C:\Program Files\Epic Games\UE_4.27"
set "UPROJECT=%~dp0Unreal\Environments\Blocks\Blocks.uproject"
set "SETTINGS_JSON=%~dp0settings.json"
set "PROJECT_SETTINGS_JSON=%~dp0Unreal\Environments\Blocks\settings.json"

if "%1"=="" goto :RunSimulation

set "CLOCK_PARAM=%2"
set /a HZ=%1
set /a PERIOD=1000000000 / HZ
echo Dynamic config: Setting PhysicsLoopPeriod to %1 Hz (%PERIOD% ns)...
if not "%CLOCK_PARAM%"=="" echo Dynamic config: Setting ClockType to %CLOCK_PARAM%...

powershell -NoProfile -Command "$p = '%SETTINGS_JSON%'; if (Test-Path $p) { $c = Get-Content $p -Raw -ErrorAction SilentlyContinue; $json = $null; if (![string]::IsNullOrEmpty($c)) { $json = try { $c | ConvertFrom-Json -ErrorAction Stop } catch { $null } }; if ($null -eq $json) { $json = [pscustomobject]@{} }; $json | Add-Member -NotePropertyName 'PhysicsLoopPeriod' -NotePropertyValue ([long]%PERIOD%) -Force; $cParam = '%CLOCK_PARAM%'; if ($cParam -ne '') { $cVal = if ($cParam -eq 'true' -or $cParam -eq 'SteppableClock') { 'SteppableClock' } else { if ($cParam -eq 'false' -or $cParam -eq 'ScalableClock') { 'ScalableClock' } else { $cParam } }; $json | Add-Member -NotePropertyName 'ClockType' -NotePropertyValue $cVal -Force }; $json | ConvertTo-Json -Depth 100 | Out-File $p -Encoding utf8 }"
powershell -NoProfile -Command "$p = '%PROJECT_SETTINGS_JSON%'; if (Test-Path $p) { $c = Get-Content $p -Raw -ErrorAction SilentlyContinue; $json = $null; if (![string]::IsNullOrEmpty($c)) { $json = try { $c | ConvertFrom-Json -ErrorAction Stop } catch { $null } }; if ($null -eq $json) { $json = [pscustomobject]@{} }; $json | Add-Member -NotePropertyName 'PhysicsLoopPeriod' -NotePropertyValue ([long]%PERIOD%) -Force; $cParam = '%CLOCK_PARAM%'; if ($cParam -ne '') { $cVal = if ($cParam -eq 'true' -or $cParam -eq 'SteppableClock') { 'SteppableClock' } else { if ($cParam -eq 'false' -or $cParam -eq 'ScalableClock') { 'ScalableClock' } else { $cParam } }; $json | Add-Member -NotePropertyName 'ClockType' -NotePropertyValue $cVal -Force }; $json | ConvertTo-Json -Depth 100 | Out-File $p -Encoding utf8 }"

:RunSimulation
echo Starting Blocks simulation in Standalone mode (1280x720 Windowed)...
start "" "%ENGINE_ROOT%\Engine\Binaries\Win64\UE4Editor.exe" "%UPROJECT%" -game -windowed -resx=1280 -resy=720 -log
