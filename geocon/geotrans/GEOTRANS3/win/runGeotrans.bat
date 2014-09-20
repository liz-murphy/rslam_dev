@echo off
REM CLASSIFICATION: UNCLASSIFIED

@set MSPCCS_DATA=..\..\data
@set path=%path%;..\..\CCS\win;.
@javaw -Xss1024k -jar MSPCCS.jar
