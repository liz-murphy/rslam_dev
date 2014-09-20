@echo off
REM CLASSIFICATION: UNCLASSIFIED
setlocal

REM set library path and data path
set PATH=.;..\..\CCS/win_64;C:\Program Files\Java\jdk1.6.0_24\bin;%PATH%
set MSPCCS_DATA=..\..\data
REM run the test driver
java -jar MSPCCS_SpreadsheetTester.jar
goto done

:done
endlocal
@echo on
