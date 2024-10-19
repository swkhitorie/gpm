@echo off

SET EXE_NAME=bootloader_px4usb

SET MDK_PATH=C:\Keil_v5

SET FROMELF=%MDK_PATH%\ARM\ARMCLANG\bin\fromelf.exe

SET YYYY=%DATE:~0,4%
SET MM=%DATE:~5,2%
SET DD=%DATE:~8,2%
SET HH=%TIME:~0,2%
SET MI=%TIME:~3,2%
SET SS=%TIME:~6,2%
SET BUILD_TIME=%YYYY%-%MM%-%DD%_%HH%-%MI%-%SS%
echo build time:%BUILD_TIME%


%FROMELF% --bin -o .\output\%EXE_NAME%.bin .\output\%EXE_NAME%.axf

COPY output\%EXE_NAME%.bin ..\..\bin\%EXE_NAME%_%BUILD_TIME%.bin

exit