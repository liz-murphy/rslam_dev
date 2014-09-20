// CLASSIFICATION: UNCLASSIFIED

// MSPCCS_SpreadsheetTester.cpp : Defines the entry point for the console application.
//

#include "ctype.h"
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "SpreadsheetTester.h"
#include "CoordinateConversionException.h"

using namespace std;


int main(int argc, char* argv[])
{
	if ( argc == 4 )
	{
		cout << "Source File: '" << argv[1] << "'" << endl;
		cout << "Output File: '" << argv[2] << "'" << endl;
		cout << "Display Warnings: '" << argv[3] << "'" << endl << endl;
    
    try
    {
      SpreadsheetTester* spreadsheetTester = new SpreadsheetTester( argv[1], argv[2], atoi( argv[3] ) != 0 );
      delete spreadsheetTester;
      spreadsheetTester = 0;
    }
    catch( CoordinateConversionException e )
    {
      cout << "The file could not be tested:" << e.getMessage() << endl;
    }
///    if(!Set_File_Names(argv[1], argv[2], atoi( argv[3] ) != 0))
///      cout << "The file could not be tested." << endl;
	}
	else
		cout << "Usage: mspccs_tester <input file> <output file> <display warnings 1 or 0>" << endl;

  return 0;
}



// CLASSIFICATION: UNCLASSIFIED
