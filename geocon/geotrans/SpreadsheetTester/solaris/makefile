####
# Compiler declarations
####

#Sun SPARCworks
cc = /opt/SUNWspro/bin/cc
CC = /opt/SUNWspro/bin/CC
COMPILER_BOOLS = -DCOMPILER_HAS_NO_BOOLEAN
COMPILER_ISSUES = $(COMPILER_BOOLS) $(COMPILER_CASTS) -DSOLARIS

#Gnu
#cc = gcc
#CC = g++
#COMPILER_ISSUES = $(COMPILER_CASTS)

####
# Support declarations
####

DEBUG = -g

CFLAGS = $(DEBUG)
CCFLAGS = $(CFLAGS) $(COMPILER_ISSUES)

####
# Directories
####

CCSERVICEDIR = ../../CCS/src
SRCDIR = ../src

####
# Libraries
####

LIBMSP_CCS = \
	../../CCS/solaris/libMSPCoordinateConversionService.so \
	../../CCS/solaris/libMSPdtcc.so \




####
# Sources
####


SRCS = \
	SpreadsheetTester.cpp \
	MSPCCS_SpreadsheetTester.cpp
		
####
# Includes
####

INCLUDES = \
        -I$(CCSERVICEDIR)/dtcc \
        -I$(CCSERVICEDIR)/CoordinateConversion \
        -I$(CCSERVICEDIR)/dtcc/CoordinateSystemParameters \
        -I$(CCSERVICEDIR)/dtcc/CoordinateSystems \
        -I$(CCSERVICEDIR)/dtcc/CoordinateTuples \
        -I$(CCSERVICEDIR)/dtcc/Enumerations \
        -I$(CCSERVICEDIR)/dtcc/Exception \
        -I$(SRCDIR)
	
####
# Suffix rules
####

.SUFFIXES: .cpp .o

.cpp.o:
	$(CC) -c $(CCFLAGS) $(INCLUDES) $<
	
####
# Targets
####

#MSPCCS_SpreadsheetTester
MSPCCS_SpreadsheetTester: $(SRCS:.cpp=.o)
	$(CC) -o MSPCCS_SpreadsheetTester $(INCLUDES) $(SRCS:.cpp=.o) $(LIBMSP_CCS) 
	
#Misc MSPCCS_SpreadsheetTester application sources
$(SRCS):
	ln -s $(SRCDIR)/$@ .
	
clean:
	$(RM) *.c *.cpp *.o $(BINARY)


