#! /bin/csh

# CLASSIFICATION : UNCLASSIFIED

# set the library path for solaris
setenv LD_LIBRARY_PATH	../solaris

# set the library path for linux
#setenv LD_LIBRARY_PATH	../linux

setenv MSPCCS_DATA	../../data

./testCoordinateConversionSample
