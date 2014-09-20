@echo off
setlocal

REM CLASSIFICATION : UNCLASSIFIED

REM set library path and data path
set PATH=..\win;%PATH%
set MSPCCS_DATA=..\..\data

REM run the test driver
.\testCoordinateConversionSample.exe
goto done

:usage
echo Usage:
echo    testCoordinateConversionSample.bat
goto done

:done
endlocal
@echo on
