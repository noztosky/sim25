@echo off
setlocal

echo Building AirLib Release x64...
msbuild "%~dp0AirLib\AirLib.vcxproj" /property:Configuration=Release /property:Platform=x64
