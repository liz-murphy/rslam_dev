@echo off
REM CLASSIFICATION: UNCLASSIFIED
setlocal

REM set library path and data path
set PATH=.;..\..\CCS/win;C:\Program Files\Java\jdk1.5.0_18\bin;%PATH%
set MSPCCS_DATA=..\..\data
REM run the test driver
java -jar MSPCCS_SpreadsheetTester.jar
goto done

:done
endlocal
@echo on
