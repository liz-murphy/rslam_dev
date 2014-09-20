set CURRENT_BUILD=C:\MSP
set PATH=%CURRENT_BUILD%\lib;C:\Program Files\Java\jre6\bin;%PATH%
set GEOID_DATA=%CURRENT_BUILD%\data
set MSPCCS_DATA=%CURRENT_BUILD%\data
set CCS_DATA=%CURRENT_BUILD%\Test_Drivers\inputs
set RESULTS_DIR=%CURRENT_BUILD%\Test_Drivers\outputs
del /Q %RESULTS_DIR%\screen\*
del /Q %RESULTS_DIR%\output\*

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\cartesianCombos.txt %RESULTS_DIR%\output\cartesianCombos.txt 1 > %RESULTS_DIR%\screen\cartesianCombos.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\combHorizVertTrans.txt %RESULTS_DIR%\output\combHorizVertTrans.txt 1 > %RESULTS_DIR%\screen\combHorizVertTrans.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\conversionCombos.txt %RESULTS_DIR%\output\conversionCombos.txt 1 > %RESULTS_DIR%\screen\conversionCombos.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\GDCtoGCC.txt %RESULTS_DIR%\output\GDCtoGCC.txt 1 > %RESULTS_DIR%\screen\GDCtoGCC.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\geoToMgrs_CD.txt %RESULTS_DIR%\output\geoToMgrs_CD.txt 1 > %RESULTS_DIR%\screen\geoToMgrs_CD.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\geoToMgrs_less0.6.txt %RESULTS_DIR%\output\geoToMgrs_less0.6.txt 1 > %RESULTS_DIR%\screen\geoToMgrs_less0.6.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\geoToMgrs_psn_we.txt %RESULTS_DIR%\output\geoToMgrs_psn_we.txt 1 > %RESULTS_DIR%\screen\geoToMgrs_psn_we.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\geoToMgrs_pss_we.txt %RESULTS_DIR%\output\geoToMgrs_pss_we.txt 1 > %RESULTS_DIR%\screen\geoToMgrs_pss_we.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\geoToMgrs_WE.txt %RESULTS_DIR%\output\geoToMgrs_WE.txt 1 > %RESULTS_DIR%\screen\geoToMgrs_WE.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\grids.txt %RESULTS_DIR%\output\grids.txt 1 > %RESULTS_DIR%\screen\grids.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\horizDT-360DegFormat.txt %RESULTS_DIR%\output\horizDT-360DegFormat.txt 1 > %RESULTS_DIR%\screen\horizDT-360DegFormat.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\horizDT-DDFormat.txt %RESULTS_DIR%\output\horizDT-DDFormat.txt 1 > %RESULTS_DIR%\screen\horizDT-DDFormat.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\localHorizDT-DDFormat.txt %RESULTS_DIR%\output\localHorizDT-DDFormat.txt 1 > %RESULTS_DIR%\screen\localHorizDT-DDFormat.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToGeo_CD.txt %RESULTS_DIR%\output\mgrsToGeo_CD.txt 1 > %RESULTS_DIR%\screen\mgrsToGeo_CD.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToGeo_less0.6.txt %RESULTS_DIR%\output\mgrsToGeo_less0.6.txt 1 > %RESULTS_DIR%\screen\mgrsToGeo_less0.6.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToGeo_psn_we.txt %RESULTS_DIR%\output\mgrsToGeo_psn_we.txt 1 > %RESULTS_DIR%\screen\mgrsToGeo_psn_we.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToGeo_pss_we.txt %RESULTS_DIR%\output\mgrsToGeo_pss_we.txt 1 > %RESULTS_DIR%\screen\mgrsToGeo_pss_we.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToGeo_WE.txt %RESULTS_DIR%\output\mgrsToGeo_WE.txt 1 > %RESULTS_DIR%\screen\mgrsToGeo_WE.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToUps_psn_we.txt %RESULTS_DIR%\output\mgrsToUps_psn_we.txt 1 > %RESULTS_DIR%\screen\mgrsToUps_psn_we.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToUps_pss_we.txt %RESULTS_DIR%\output\mgrsToUps_pss_we.txt 1 > %RESULTS_DIR%\screen\mgrsToUps_pss_we.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToUtm_CD.txt %RESULTS_DIR%\output\mgrsToUtm_CD.txt 1 > %RESULTS_DIR%\screen\mgrsToUtm_CD.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToUtm_less0.6.txt %RESULTS_DIR%\output\mgrsToUtm_less0.6.txt 1 > %RESULTS_DIR%\screen\mgrsToUtm_less0.6.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\mgrsToUtm_WE.txt %RESULTS_DIR%\output\mgrsToUtm_WE.txt 1 > %RESULTS_DIR%\screen\mgrsToUtm_WE.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\projections.txt %RESULTS_DIR%\output\projections.txt 1 > %RESULTS_DIR%\screen\projections.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\upsToMgrs_psn_we.txt %RESULTS_DIR%\output\upsToMgrs_psn_we.txt 1 > %RESULTS_DIR%\screen\upsToMgrs_psn_we.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\upsToMgrs_pss_we.txt %RESULTS_DIR%\output\upsToMgrs_pss_we.txt 1 > %RESULTS_DIR%\screen\upsToMgrs_pss_we.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\usng_grids.txt %RESULTS_DIR%\output\usng_grids.txt 1 > %RESULTS_DIR%\screen\usng_grids.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\utmToMgrs_CD.txt %RESULTS_DIR%\output\utmToMgrs_CD.txt 1 > %RESULTS_DIR%\screen\utmToMgrs_CD.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\utmToMgrs_less0.6.txt %RESULTS_DIR%\output\utmToMgrs_less0.6.txt 1 > %RESULTS_DIR%\screen\utmToMgrs_less0.6.txt

%CURRENT_BUILD%\Test_Drivers\MSPCCS_SpreadsheetTester\win\MSPCCS_SpreadsheetTester.exe %CCS_DATA%\utmToMgrs_WE.txt %RESULTS_DIR%\output\utmToMgrs_WE.txt 1 > %RESULTS_DIR%\screen\utmToMgrs_WE.txt

