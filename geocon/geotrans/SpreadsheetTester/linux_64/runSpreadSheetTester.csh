#! /bin/csh
# CLASSIFICATION: UNCLASSIFIED

setenv LD_LIBRARY_PATH .:../../CCS/linux_64
setenv MSPCCS_DATA ../../data
setenv JAVA_HOME /usr/jdk1.6.0_24

${JAVA_HOME}/bin/java -jar MSPCCS_SpreadsheetTester.jar
