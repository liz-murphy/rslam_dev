@echo off
REM CLASSIFICATION: UNCLASSIFIED

@set MSPCCS_DATA=..\..\data
@set path=%path%;..\..\CCS\win_64;.
@javaw -Xss1024k -jar MSPCCS.jar
