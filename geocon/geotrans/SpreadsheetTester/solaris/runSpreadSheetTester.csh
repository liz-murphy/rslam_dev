#! /bin/csh
# CLASSIFICATION: UNCLASSIFIED

setenv LD_LIBRARY_PATH .:../../CCS/solaris
setenv MSPCCS_DATA ../../data
setenv JAVA_HOME /usr/jdk1.5.0_18

${JAVA_HOME}/bin/java -jar MSPCCS_SpreadsheetTester.jar
