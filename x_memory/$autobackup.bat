@echo off

rmdir "build-INTOGCS2-Desktop_Qt_5_15_2_MSVC2019_64bit-Debug" /s /q
rmdir "build-INTOGCS-Desktop_Qt_5_15_2_MSVC2019_64bit-Debug" /s /q
rmdir "build-INTOGCS-Desktop_Qt_5_15_2_MSVC2019_64bit-Profile" /s /q

for %%I in (.) do set dname=%%~nxI

set d=%date:~0%
set year=%d:~0,4%
set month=%d:~5,2%
set day=%d:~8,2%

set hh=%time:~0,2%
set mm=%time:~3,2%
set ss=%time:~6,2%

set CURPATH=%cd%

del "*.sdf" /s /q
rd ".\ipch" /s /q
rd ".\Visual Micro" /s /q
rd ".\debug" /s /q
rd ".\__vm" /s /q

call 7z a "%intosky_backup_path%\%year%%month%%day%(%hh%%mm%%ss%) %dname%(%pcname%).7z" "%CURPATH%\*" -m0=lzma2 -mx=9 -aoa
