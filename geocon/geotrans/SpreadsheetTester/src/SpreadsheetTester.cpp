// CLASSIFICATION: UNCLASSIFIED
/*

File
----
Coord_Test.c

Authors
-------
Amy Comstock

Description
-----------
This file reads a row from a spreadsheet containing a mspccs test
case. The row is used to set the input & output datums, input & output
projections, input & output parameters and input coordinates. The test 
case is converted and the row is written to a file. The new row will 
contain the converted values. A row consists of the following columns:
test case id, input datum code or ellipse name (Airy, WGS_84, WGS_72, Clarke 1880,
Clarke 1866, GRS_80, International, Bessel 1841, Australian National, South American 1969 only),
input projection, output datum code or ellipse name, output projection, input 
zone or hemisphere, output zone or hemisphere, input latitude(or easting, string, X), 
input longitude (or northing, Y), input height or Z, output latitude(or easting, string, X),
output longitude (or northing, Y), output height or Z, calculated 
latitude(or easting, string, X), calculated longitude (or northing, Y),
calculated height or Z, calculated zone or hemisphere, input 1st std. parallel or scale factor, 
input 2nd std. parallel, input central meridian, input origin latitude,
input false easting, input false northing, input longitude 1, input latitude 1, 
input longitude 2, input latitude 2, output 1st std.parallel or scale factor, output 2nd std. 
parallel, output central meridian, output origin latitude, output false easting, 
output false northing, output longitude 1, output latitude 1, output longitude 2, output 
latitude 2, results.

History
-------
7-12-2000
01-06-2009 - updated to work with thread safe msp ccs
06-16-2010 - S. Gillis, BAEts26660, spreadsheet tester has incorrect datum string
02-25-2010 - K. Lam, BAEts26267, add EGM 2008 support
 
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "SpreadsheetTester.h"
#include "CoordinateConversionService.h"
#include "CoordinateSystemParameters.h"
#include "EquidistantCylindricalParameters.h"
#include "GeodeticParameters.h"
#include "LocalCartesianParameters.h"
#include "MercatorStandardParallelParameters.h"
#include "MercatorScaleFactorParameters.h"
#include "MapProjection3Parameters.h"
#include "MapProjection4Parameters.h"
#include "MapProjection5Parameters.h"
#include "MapProjection6Parameters.h"
#include "NeysParameters.h"
#include "ObliqueMercatorParameters.h"
#include "PolarStereographicStandardParallelParameters.h"
#include "PolarStereographicScaleFactorParameters.h"
#include "UTMParameters.h"
#include "Accuracy.h"
#include "BNGCoordinates.h"
#include "CartesianCoordinates.h"
#include "GARSCoordinates.h"
#include "GeodeticCoordinates.h"
#include "GEOREFCoordinates.h"
#include "MapProjectionCoordinates.h"
#include "MGRSorUSNGCoordinates.h"
#include "UPSCoordinates.h"
#include "UTMCoordinates.h"
#include "CoordinateConversionException.h"
#include "ErrorMessages.h"


using namespace MSP::CCS;


#define BOOLEAN long
#define TRUE 1
#define FALSE 0
#define COORDINATES_ERROR 2

#define PI                    3.14159265358979323e0  /* PI  */
#define TWO_PI                2.0 * PI                 
#define PI_OVER_180           PI / 180.0
#define _180_OVER_PI          180.0 / PI
#define PI_DEGREES            180  /* PI in degrees */
#define TWO_PI_DEGREES        2.0 * PI_DEGREES  /* 2 * PI in degrees */
#define LETTER_I               8   /* ARRAY INDEX FOR LETTER I               */
#define LETTER_O              14   /* ARRAY INDEX FOR LETTER O               */


/* Convert a string to uppercase - if there is a space, convert it to an underscore(_) */
void To_UpperCase (char* string)
{
  int length = strlen(string);
  int index = 0;
  
  while (index < length)
  {

    if (isalpha(string[index]))
      string[index] = (char)toupper(string[index]);
    else if ((string[index - 1] != ',') && (string[index] == ' '))
      string[index] = '_';
    else
      string[index] = string[index];
    index++;
  }
}


long Break_MGRS_Str (char* MGRS,
                     long* Zone,
                     long Letters[3],
                     double* Easting,
                     double* Northing)
/* Break down an MGRS string into its component parts */
{ 
  long error_code = FALSE;
  long i = 0;
  long j = 0;
  long num_digits;
  long num_letters;

  while (MGRS[i] == ' ')
    i++;  /* skip any leading blanks */
  j = i;
  while (isdigit(MGRS[i]))
    i++;
  num_digits = i - j;
  if (num_digits <= 2)
    if (num_digits > 0)
    {
      char zone_string[3];
      /* get zone */
      strncpy (zone_string, MGRS+j, 2);
      zone_string[2] = 0;
      sscanf (zone_string, "%ld", Zone);  
      if ((*Zone < 1) || (*Zone > 60))
        error_code = TRUE;
    }
    else
      *Zone = 0;
  else
    error_code = TRUE;
  j = i;

  while (isalpha(MGRS[i]))
    i++;
  num_letters = i - j;
  if (num_letters == 3)
  {
    /* get letters */
    Letters[0] = (toupper(MGRS[j]) - (long)'A');
    if ((Letters[0] == LETTER_I) || (Letters[0] == LETTER_O))
      error_code = TRUE;
    Letters[1] = (toupper(MGRS[j+1]) - (long)'A');
    if ((Letters[1] == LETTER_I) || (Letters[1] == LETTER_O))
      error_code = TRUE;
    Letters[2] = (toupper(MGRS[j+2]) - (long)'A');
    if ((Letters[2] == LETTER_I) || (Letters[2] == LETTER_O))
      error_code = TRUE;
  }
  else
    error_code |= TRUE;
  j = i;
  while (isdigit(MGRS[i]))
    i++;
  num_digits = i - j;
  if ((num_digits <= 10) && (num_digits%2 == 0))
  {
    long n;
    char east_string[6];
    char north_string[6];
    long east;
    long north;
    double multiplier;
    /* get easting & northing */
    n = num_digits/2;
   /// *Precision = n;
    if (n > 0)
    {
      strncpy (east_string, MGRS+j, n);
      east_string[n] = 0;
      sscanf (east_string, "%ld", &east);
      strncpy (north_string, MGRS+j+n, n);
      north_string[n] = 0;
      sscanf (north_string, "%ld", &north);
      multiplier = pow (10.0, 5.0 - n);
      *Easting = east * multiplier;
      *Northing = north * multiplier;
    }
    else
    {
      *Easting = 0.0;
      *Northing = 0.0;
    }
  }
  else
    error_code = TRUE;

  return error_code;
} 


long Break_GEOREF_Str(char* GEOREF,
                      long Letters[4],
                      double* Easting,
                      double* Northing)
/* Break down a GEOREF string into its component parts */
{ 
  long error_code = FALSE;
  long i = 0;
  long j = 0;
  long num_digits;
  long num_letters;

  while (GEOREF[i] == ' ')
    i++;  /* skip any leading blanks */
  j = i;
//  while (isdigit(MGRS[i]))
 //   i++;
 // num_digits = i - j;
 /* if (num_digits <= 2)
    if (num_digits > 0)
    {
      char zone_string[3];
      /* get zone */
 /*     strncpy (zone_string, MGRS+j, 2);
      zone_string[2] = 0;
      sscanf (zone_string, "%ld", Zone);  
      if ((*Zone < 1) || (*Zone > 60))
        error_code = TRUE;
    }
    else
      *Zone = 0;
  else
    error_code = TRUE;
  j = i;*/

  while (isalpha(GEOREF[i]))
    i++;
  num_letters = i - j;
  if (num_letters == 4)
  {
    /* get letters */
    Letters[0] = (toupper(GEOREF[j]) - (long)'A');
    if ((Letters[0] == LETTER_I) || (Letters[0] == LETTER_O))
      error_code = TRUE;
    Letters[1] = (toupper(GEOREF[j+1]) - (long)'A');
    if ((Letters[1] == LETTER_I) || (Letters[1] == LETTER_O))
      error_code = TRUE;
    Letters[2] = (toupper(GEOREF[j+2]) - (long)'A');
    if ((Letters[2] == LETTER_I) || (Letters[2] == LETTER_O))
      error_code = TRUE;
    Letters[3] = (toupper(GEOREF[j+3]) - (long)'A');
    if ((Letters[3] == LETTER_I) || (Letters[3] == LETTER_O))
      error_code = TRUE;
  }
  else
    error_code |= TRUE;
  j = i;
  while (isdigit(GEOREF[i]))
    i++;
  num_digits = i - j;
  if ((num_digits <= 10) && (num_digits%2 == 0))
  {
    long n;
    char east_string[6];
    char north_string[6];
    long east;
    long north;
    double multiplier;
    /* get easting & northing */
    n = num_digits/2;
   /// *Precision = n;
    if (n > 0)
    {
      strncpy (east_string, GEOREF+j, n);
      east_string[n] = 0;
      sscanf (east_string, "%ld", &east);
      strncpy (north_string, GEOREF+j+n, n);
      north_string[n] = 0;
      sscanf (north_string, "%ld", &north);
      multiplier = pow (10.0, 5.0 - n);
      *Easting = east * multiplier;
      *Northing = north * multiplier;
    }
    else
    {
      *Easting = 0.0;
      *Northing = 0.0;
    }
  }
  else
    error_code = TRUE;

  return error_code;
} 


BOOLEAN Compare_MGRS_Result(char* expected_str, char* results_str)
{
  long expected_zone, results_zone;
  long expected_letters[3], results_letters[3];
  double expected_easting, results_easting;
  double expected_northing, results_northing;
  int expected_length = strlen(expected_str);
  int results_length = strlen(results_str);

  if (expected_length != results_length)
    return FALSE;
  else
  {
    if (!Break_MGRS_Str(expected_str, &expected_zone, expected_letters, &expected_easting, &expected_northing))
    {
      if (!Break_MGRS_Str(results_str, &results_zone, results_letters, &results_easting, &results_northing))
      {
        if (expected_zone != results_zone)
          return FALSE;

        if (expected_letters[0] != results_letters[0] || 
            expected_letters[1] != results_letters[1] || 
            expected_letters[2] != results_letters[2])
          return FALSE;

        if (fabs(expected_easting - results_easting) > 1.0e0)
          return FALSE;

        if (fabs(expected_northing - results_northing) > 1.0e0)
          return FALSE;
      }
    }
  }

  return TRUE;
}


BOOLEAN Compare_GEOREF_Result(char* expected_str, char* results_str)
{
  long expected_letters[4], results_letters[4];
  double expected_easting, results_easting;
  double expected_northing, results_northing;
  int expected_length = strlen(expected_str);
  int results_length = strlen(results_str);

  if (expected_length != results_length)
    return FALSE;
  else
  {
    if (!Break_GEOREF_Str(expected_str, expected_letters, &expected_easting, &expected_northing))
    {
      if (!Break_GEOREF_Str(results_str, results_letters, &results_easting, &results_northing))
      {
        if (expected_letters[0] != results_letters[0] || 
            expected_letters[1] != results_letters[1] || 
            expected_letters[2] != results_letters[2] || 
            expected_letters[3] != results_letters[3])
          return FALSE;

        if (fabs(expected_easting - results_easting) > 1.0e0)
          return FALSE;

        if (fabs(expected_northing - results_northing) > 1.0e0)
          return FALSE;
      }
    }
  }

  return TRUE;
}


/* Open the input file for reading, output file for writing and create a file */
/* to contain error messages */
SpreadsheetTester::SpreadsheetTester( const char* input_file_name, const char* output_file_name, bool _showWarnings ):
  input_file( 0 ),
  output_file( 0 ),
  error_count( 0 ),
  warning_different_count( 0 ),
  warning_equivalent_count( 0 ),
  successful_different_count( 0 ),
  successful_equivalent_count( 0 ),
  coordinateConversionService( 0 ),
  input_coord1( 0.0 ),
  input_coord2( 0.0 ),
  input_coord3( 0.0 ),
  output_coord1( 0.0 ),
  output_coord2( 0.0 ),
  output_coord3( 0.0 ),
  input_hemisphere( '\0' ),
  output_hemisphere( '\0' ),
  id( NULL ),
  input_coord1_str( NULL ),
  output_coord1_str( NULL ),
  input_datum( NULL ),
  output_datum( NULL ),
  input_projection( NULL ),
  output_projection( NULL ),
  data( NULL ),
  input_height( 0 ),
  output_height( 0 ),
  zone( 0 ),
  showWarnings( true )

{
  /*  Thread_Mutex geoid_96_mutex;
    long mutex_error = Threads_Create_Mutex( &geoid_96_mutex, "initializeEGM96Geoid_Mutex" );
    if( !mutex_error )
      mutex_error = Threads_Lock_Mutex( geoid_96_mutex );*/

  showWarnings = _showWarnings;
  testEGM2008  = false;

  char* envEGM2008 = getenv( "EGM2008_TEST" );
  if (envEGM2008)
  {
      if ( strcmp( envEGM2008, "YES" ) == 0 )
         testEGM2008 = true;
  }

  if( Open_Test_File( SourceOrTarget::source, input_file_name, &input_file ) )
  {
    if( Open_Test_File( SourceOrTarget::target, output_file_name, &output_file ) )
    {
      Convert_Test_File();

      Close_Test_File( &input_file );
      Close_Test_File( &output_file );
    }
    else
    {
      printf( "The file, %s, could not be opened.\n", output_file_name );
      Close_Test_File( &input_file );
    }
  }
  else
  {
    printf( "The file, %s, could not be opened.\n", input_file_name );
  }
/*    if( !mutex_error )
      mutex_error = Threads_Unlock_Mutex( geoid_96_mutex );
    if( !mutex_error )
      Threads_Destroy_Mutex( geoid_96_mutex );*/
}


SpreadsheetTester::~SpreadsheetTester()
{
  if( coordinateConversionService )
  {
    delete coordinateConversionService;
    coordinateConversionService = 0;
  }

  deleteCoordinateSystem( SourceOrTarget::source );
  deleteCoordinateSystem( SourceOrTarget::target );

  if (id != NULL)
  {
      free(id);
      id = NULL;
  }
  if (input_coord1_str != NULL)
  {
      free(input_coord1_str);
      input_coord1_str = NULL;
  }
  if (output_coord1_str != NULL)
  {
      free(output_coord1_str);
      output_coord1_str = NULL;
  }
  if (input_datum != NULL)
  {
      free(input_datum);
      input_datum = NULL;
  }
  if (output_datum != NULL)
  {
      free(output_datum);
      output_datum = NULL;
  }
  if (input_projection != NULL)
  {
      free(input_projection);
      input_projection = NULL;
  }
  if (output_projection != NULL)
  {
      free(output_projection);
      output_projection = NULL;
  }
  if (data != NULL)
  {
      free(data);
      data = NULL;
  }
}


/* Open a file for reading or writing */
bool SpreadsheetTester::Open_Test_File( SourceOrTarget::Enum direction, const char *filename, FILE **file)
{
  char *control_String = "r";
  FILE *local_File = NULL;

  if (direction == SourceOrTarget::source)
    control_String = "r";
  else if (direction == SourceOrTarget::target)
    control_String = "w";
  local_File = fopen(filename, control_String);

  if (local_File)
  {
    *file = local_File;
    return TRUE;
  }
  else
  {
    *file = NULL;
    return FALSE;
  }
}


/* Close a file */
void SpreadsheetTester::Close_Test_File(FILE **file)
{ 
  if (*file != NULL)
  {
    fclose(*file);
    *file = NULL;
  }
  else
    printf("file is null, cannont close\n");
} 


/* Convert a test case file */
void SpreadsheetTester::Convert_Test_File()
{ 
  int total_test_cases = 0;

  Read_Header();

  while (!feof(input_file))
  {
    initCoordinateSystemState( SourceOrTarget::source );
    initCoordinateSystemState( SourceOrTarget::target );

    long error_code = 0;

    total_test_cases++;
///if(total_test_cases == 494)
///int f = 3;

    error_code = Read_File();

    if (error_code == TRUE)
    {
      try
      {
        MSP::CCS::CoordinateConversionService* tempCoordinateConversionService = new MSP::CCS::CoordinateConversionService( coordinateSystemState[SourceOrTarget::source].datumCode, getCoordinateSystem( SourceOrTarget::source ), 
                                     coordinateSystemState[SourceOrTarget::target].datumCode, getCoordinateSystem( SourceOrTarget::target ) );
      
        if( coordinateConversionService )
        {
          delete coordinateConversionService;
          coordinateConversionService = 0;
        }
      
        coordinateConversionService = tempCoordinateConversionService;

        convertSourceToTarget();

        Write_To_File( FALSE );
      }
      catch( CoordinateConversionException e )
      {
        error_count++;
        printf("Test Case Id: %s - Conversion Error: %s\n", id, e.getMessage());
        Write_To_File(FALSE);
      }

      deleteCoordinateSystem( SourceOrTarget::source );
      deleteCoordinateSystem( SourceOrTarget::target );
    }
    else
    {
      if((error_code & COORDINATES_ERROR) != 0)
        Write_To_File(FALSE);
      else
        Write_To_File(TRUE);
    }

    if (id != NULL)
    {
        free(id);
        id = NULL;
    }
    if (input_coord1_str != NULL)
    {
        free(input_coord1_str);
        input_coord1_str = NULL;
    }
    if (output_coord1_str != NULL)
    {
        free(output_coord1_str);
        output_coord1_str = NULL;
    }
    if (input_datum != NULL)
    {
        free(input_datum);
        input_datum = NULL;
    }
    if (output_datum != NULL)
    {
        free(output_datum);
        output_datum = NULL;
    }
    if (input_projection != NULL)
    {
        free(input_projection);
        input_projection = NULL;
    }
    if (output_projection != NULL)
    {
        free(output_projection);
        output_projection = NULL;
    }
    if (data != NULL)
    {
        free(data);
        data = NULL;
    }
  }

  if( coordinateConversionService )
  {
    delete coordinateConversionService;
    coordinateConversionService = 0;
  }

  printf("\n\nResults: \n\n");
  printf("\tSuccessful\n");
  printf("\t\tEquivalent Results\t%d \n", successful_equivalent_count);
  printf("\t\tDifferent Results\t%d \n", successful_different_count);
  printf("\tWarnings\n");
  printf("\t\tEquivalent Results\t%d \n", warning_equivalent_count);
  printf("\t\tDifferent Results\t%d \n", warning_different_count);
  printf("\tErrors (No Results)\t\t%d \n", error_count);
  printf("\tTotal Test Cases\t\t%d \n", total_test_cases);

}


void SpreadsheetTester::initCoordinateSystemState( const SourceOrTarget::Enum direction )
{
/*
 *  The function initCoordinateSystemState initializes coordinateSystemState.
 *
 *  direction  : Indicates whether the coordinate system is to be used for
 *               source or target                                      (input)
 */

 /* Thread_Mutex mutex;
  long mutex_error = Threads_Create_Mutex( &mutex, "initCoordinateSystem_Mutex" );
  if( !mutex_error )
    mutex_error = Threads_Lock_Mutex( mutex );*/

///  coordinateSystemState[direction].datumIndex = 0;
  coordinateSystemState[direction].coordinateType = CoordinateType::geodetic;
///  coordinateSystemState[direction].coordinateSystem = 0;

  coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
  coordinateSystemState[direction].parameters.mapProjection3Parameters = 0;
  coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
  coordinateSystemState[direction].parameters.mapProjection5Parameters = 0;
  coordinateSystemState[direction].parameters.mapProjection6Parameters = 0;
  coordinateSystemState[direction].parameters.equidistantCylindricalParameters = 0;
  coordinateSystemState[direction].parameters.geodeticParameters = 0;
  coordinateSystemState[direction].parameters.localCartesianParameters = 0;
  coordinateSystemState[direction].parameters.mercatorStandardParallelParameters = 0;
  coordinateSystemState[direction].parameters.mercatorScaleFactorParameters = 0;
  coordinateSystemState[direction].parameters.neysParameters = 0;
  coordinateSystemState[direction].parameters.obliqueMercatorParameters = 0;
  coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters = 0;
  coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters = 0;
  coordinateSystemState[direction].parameters.utmParameters = 0;

  coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
  coordinateSystemState[direction].coordinates.bngCoordinates = 0;
  coordinateSystemState[direction].coordinates.cartesianCoordinates = 0;
  coordinateSystemState[direction].coordinates.geodeticCoordinates = 0;
  coordinateSystemState[direction].coordinates.georefCoordinates = 0;
  coordinateSystemState[direction].coordinates.garsCoordinates = 0;
  coordinateSystemState[direction].coordinates.mgrsOrUSNGCoordinates = 0;
  coordinateSystemState[direction].coordinates.upsCoordinates = 0;
  coordinateSystemState[direction].coordinates.utmCoordinates = 0;

/*  if( !mutex_error )
    mutex_error = Threads_Unlock_Mutex( mutex );
  if( !mutex_error )
    Threads_Destroy_Mutex( mutex );*/
}


void SpreadsheetTester::deleteCoordinateSystem( const SourceOrTarget::Enum direction )
{
/*
 *  The function deleteCoordinateSystem frees memory of coordinateSystemState.
 *
 *  direction  : Indicates whether the coordinate system is to be used for
 *               source or target                                      (input)
 */

  switch( coordinateSystemState[direction].coordinateType )
  {
    case CoordinateType::albersEqualAreaConic:
      if( coordinateSystemState[direction].parameters.mapProjection6Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection6Parameters;
        coordinateSystemState[direction].parameters.mapProjection6Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::lambertConformalConic2Parallels:
      if( coordinateSystemState[direction].parameters.mapProjection6Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection6Parameters;
        coordinateSystemState[direction].parameters.mapProjection6Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::azimuthalEquidistant:
      if( coordinateSystemState[direction].parameters.mapProjection4Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection4Parameters;
        coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::bonne:
      if( coordinateSystemState[direction].parameters.mapProjection4Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection4Parameters;
        coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::cassini:
      if( coordinateSystemState[direction].parameters.mapProjection4Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection4Parameters;
        coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::cylindricalEqualArea:
      if( coordinateSystemState[direction].parameters.mapProjection4Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection4Parameters;
        coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::gnomonic:
      if( coordinateSystemState[direction].parameters.mapProjection4Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection4Parameters;
        coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::orthographic:
      if( coordinateSystemState[direction].parameters.mapProjection4Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection4Parameters;
        coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::polyconic:
      if( coordinateSystemState[direction].parameters.mapProjection4Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection4Parameters;
        coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::stereographic:
      if( coordinateSystemState[direction].parameters.mapProjection4Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection4Parameters;
        coordinateSystemState[direction].parameters.mapProjection4Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::britishNationalGrid:
      if( coordinateSystemState[direction].parameters.coordinateSystemParameters )
      {
        delete coordinateSystemState[direction].parameters.coordinateSystemParameters;
        coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.bngCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.bngCoordinates;
        coordinateSystemState[direction].coordinates.bngCoordinates = 0;
      }
      break;
    case CoordinateType::eckert4:
      if( coordinateSystemState[direction].parameters.mapProjection3Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection3Parameters;
        coordinateSystemState[direction].parameters.mapProjection3Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::eckert6:
      if( coordinateSystemState[direction].parameters.mapProjection3Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection3Parameters;
        coordinateSystemState[direction].parameters.mapProjection3Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::millerCylindrical:
      if( coordinateSystemState[direction].parameters.mapProjection3Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection3Parameters;
        coordinateSystemState[direction].parameters.mapProjection3Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::mollweide:
      if( coordinateSystemState[direction].parameters.mapProjection3Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection3Parameters;
        coordinateSystemState[direction].parameters.mapProjection3Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::sinusoidal:
      if( coordinateSystemState[direction].parameters.mapProjection3Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection3Parameters;
        coordinateSystemState[direction].parameters.mapProjection3Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::vanDerGrinten:
      if( coordinateSystemState[direction].parameters.mapProjection3Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection3Parameters;
        coordinateSystemState[direction].parameters.mapProjection3Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::equidistantCylindrical:
      if( coordinateSystemState[direction].parameters.equidistantCylindricalParameters )
      {
        delete coordinateSystemState[direction].parameters.equidistantCylindricalParameters;
        coordinateSystemState[direction].parameters.equidistantCylindricalParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::geocentric:
      if( coordinateSystemState[direction].parameters.coordinateSystemParameters )
      {
        delete coordinateSystemState[direction].parameters.coordinateSystemParameters;
        coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.cartesianCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.cartesianCoordinates;
        coordinateSystemState[direction].coordinates.cartesianCoordinates = 0;
      }
      break;
    case CoordinateType::geodetic:
      if( coordinateSystemState[direction].parameters.geodeticParameters )
      {
        delete coordinateSystemState[direction].parameters.geodeticParameters;
        coordinateSystemState[direction].parameters.geodeticParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.geodeticCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.geodeticCoordinates;
        coordinateSystemState[direction].coordinates.geodeticCoordinates = 0;
      }
      break;
    case CoordinateType::georef:
      if( coordinateSystemState[direction].parameters.coordinateSystemParameters )
      {
        delete coordinateSystemState[direction].parameters.coordinateSystemParameters;
        coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.georefCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.georefCoordinates;
        coordinateSystemState[direction].coordinates.georefCoordinates = 0;
      }
      break;
    case CoordinateType::globalAreaReferenceSystem:
      if( coordinateSystemState[direction].parameters.coordinateSystemParameters )
      {
        delete coordinateSystemState[direction].parameters.coordinateSystemParameters;
        coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.garsCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.garsCoordinates;
        coordinateSystemState[direction].coordinates.garsCoordinates = 0;
      }
      break;
    case CoordinateType::lambertConformalConic1Parallel:
      if( coordinateSystemState[direction].parameters.mapProjection5Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection5Parameters;
        coordinateSystemState[direction].parameters.mapProjection5Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::transverseCylindricalEqualArea:
      if( coordinateSystemState[direction].parameters.mapProjection5Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection5Parameters;
        coordinateSystemState[direction].parameters.mapProjection5Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::transverseMercator:
      if( coordinateSystemState[direction].parameters.mapProjection5Parameters )
      {
        delete coordinateSystemState[direction].parameters.mapProjection5Parameters;
        coordinateSystemState[direction].parameters.mapProjection5Parameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::localCartesian:
      if( coordinateSystemState[direction].parameters.localCartesianParameters )
      {
        delete coordinateSystemState[direction].parameters.localCartesianParameters;
        coordinateSystemState[direction].parameters.localCartesianParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.cartesianCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.cartesianCoordinates;
        coordinateSystemState[direction].coordinates.cartesianCoordinates = 0;
      }
      break;
    case CoordinateType::mercatorStandardParallel:
      if( coordinateSystemState[direction].parameters.mercatorStandardParallelParameters )
      {
        delete coordinateSystemState[direction].parameters.mercatorStandardParallelParameters;
        coordinateSystemState[direction].parameters.mercatorStandardParallelParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::mercatorScaleFactor:
      if( coordinateSystemState[direction].parameters.mercatorScaleFactorParameters )
      {
        delete coordinateSystemState[direction].parameters.mercatorScaleFactorParameters;
        coordinateSystemState[direction].parameters.mercatorScaleFactorParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::militaryGridReferenceSystem:
      if( coordinateSystemState[direction].parameters.coordinateSystemParameters )
      {
        delete coordinateSystemState[direction].parameters.coordinateSystemParameters;
        coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mgrsOrUSNGCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mgrsOrUSNGCoordinates;
        coordinateSystemState[direction].coordinates.mgrsOrUSNGCoordinates = 0;
      }
      break;
    case CoordinateType::usNationalGrid:
      if( coordinateSystemState[direction].parameters.coordinateSystemParameters )
      {
        delete coordinateSystemState[direction].parameters.coordinateSystemParameters;
        coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mgrsOrUSNGCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mgrsOrUSNGCoordinates;
        coordinateSystemState[direction].coordinates.mgrsOrUSNGCoordinates = 0;
      }
      break;
    case CoordinateType::newZealandMapGrid:
      if( coordinateSystemState[direction].parameters.coordinateSystemParameters )
      {
        delete coordinateSystemState[direction].parameters.coordinateSystemParameters;
        coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::neys:
      if( coordinateSystemState[direction].parameters.neysParameters )
      {
        delete coordinateSystemState[direction].parameters.neysParameters;
        coordinateSystemState[direction].parameters.neysParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::obliqueMercator:
      if( coordinateSystemState[direction].parameters.obliqueMercatorParameters )
      {
        delete coordinateSystemState[direction].parameters.obliqueMercatorParameters;
        coordinateSystemState[direction].parameters.obliqueMercatorParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::polarStereographicStandardParallel:
      if( coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters )
      {
        delete coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters;
        coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::polarStereographicScaleFactor:
      if( coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters )
      {
        delete coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters;
        coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.mapProjectionCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.mapProjectionCoordinates;
        coordinateSystemState[direction].coordinates.mapProjectionCoordinates = 0;
      }
      break;
    case CoordinateType::universalPolarStereographic:
      if( coordinateSystemState[direction].parameters.coordinateSystemParameters )
      {
        delete coordinateSystemState[direction].parameters.coordinateSystemParameters;
        coordinateSystemState[direction].parameters.coordinateSystemParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.upsCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.upsCoordinates;
        coordinateSystemState[direction].coordinates.upsCoordinates = 0;
      }
      break;
    case CoordinateType::universalTransverseMercator:
      if( coordinateSystemState[direction].parameters.utmParameters )
      {
        delete coordinateSystemState[direction].parameters.utmParameters;
        coordinateSystemState[direction].parameters.utmParameters = 0;
      }
      if( coordinateSystemState[direction].coordinates.utmCoordinates )
      {
        delete coordinateSystemState[direction].coordinates.utmCoordinates;
        coordinateSystemState[direction].coordinates.utmCoordinates = 0;
      }
      break;
    default:
      break;
  }
}


/* Read header from input file and write it to output file */
void SpreadsheetTester::Read_Header(void)
{
  char header[692];

  fscanf(input_file, " %247[^\n]", header);
  fprintf(output_file, "%s\n", header);
  fscanf(input_file, " %691[^\n]", header);
  fprintf(output_file, "\t\t\t\t\t%s\n\n", header);
}


/* Read a row from the file, set input & output projections and datums */
long SpreadsheetTester::Read_File(void)
{
  char buf[200] = "";
  int next_character = '\0';  
  char temp_input_projection[COORD_SYS_NAME_LENGTH] = "";
  char temp_output_projection[COORD_SYS_NAME_LENGTH] = "";
  int buf_length = 0;

  input_coord1 = 0.0;
  input_coord2 = 0.0;
  input_coord3 = 0.0;
  output_coord1 = 0.0;
  output_coord2 = 0.0;
  output_coord3 = 0.0;
  input_height = 0;
  output_height = 0;

  fscanf(input_file, " %s", buf);
  id = (char *)malloc((strlen(buf) + 1) * sizeof(char));
  strcpy(id, buf);
  buf[0] = '\0';

  fscanf(input_file, " %[^\t\n]", buf);
  input_datum = (char *)malloc((strlen(buf) + 1) * sizeof(char));
  strcpy(input_datum, buf);
  buf[0] = '\0';

  fscanf(input_file, " %[^\t\n]", buf);
  input_projection = (char *)malloc((strlen(buf) + 1) * sizeof(char));
  strcpy(input_projection, buf);
  buf[0] = '\0';

  fscanf(input_file, " %[^\t\n]", buf);
  output_datum = (char *)malloc((strlen(buf) + 1) * sizeof(char));
  strcpy(output_datum, buf);
  buf[0] = '\0';

  fscanf(input_file, " %[^\t\n]", buf);
  output_projection = (char *)malloc((strlen(buf) + 1) * sizeof(char));
  strcpy(output_projection, buf);
  buf[0] = '\0';

  strcpy(temp_input_projection, input_projection);
  strcpy(temp_output_projection, output_projection);

  
  strcpy(coordinateSystemState[SourceOrTarget::source].datumCode, input_datum);
  To_UpperCase(coordinateSystemState[SourceOrTarget::source].datumCode);

  strcpy(coordinateSystemState[SourceOrTarget::target].datumCode, output_datum);
  To_UpperCase(coordinateSystemState[SourceOrTarget::target].datumCode);

  To_UpperCase(temp_input_projection);
  To_UpperCase(temp_output_projection);

  if (Set_Projection(SourceOrTarget::source, temp_input_projection))
  {
    if (Set_Projection(SourceOrTarget::target, temp_output_projection)) 
    {
      CoordinateType::Enum input_projection_type = coordinateSystemState[SourceOrTarget::source].coordinateType;
      CoordinateType::Enum output_projection_type = coordinateSystemState[SourceOrTarget::target].coordinateType;

      if ((input_projection_type == CoordinateType::universalPolarStereographic) || 
          (input_projection_type == CoordinateType::polarStereographicScaleFactor))
      {
        fscanf(input_file, " %c ", &input_hemisphere);
        input_hemisphere = (char)toupper(input_hemisphere);
      }
      else if (input_projection_type == CoordinateType::universalTransverseMercator)
      {
        fscanf(input_file, " %34[^\t\n]", buf);
        zone = atoi(buf);
          
        if (zone < 0)
          input_hemisphere = 'S';
        else
          input_hemisphere = 'N';
      }

      if ((output_projection_type == CoordinateType::universalPolarStereographic) ||
          (output_projection_type == CoordinateType::polarStereographicScaleFactor))
      {
        fscanf(input_file, " %c ", &output_hemisphere);
        output_hemisphere = (char)toupper(output_hemisphere);
      }
      else if (output_projection_type == CoordinateType::universalTransverseMercator)
      {
        fscanf(input_file, " %34[^\t\n]", buf);
        zone = atoi(buf);
        if (zone < 0)
          output_hemisphere = 'S';
        else
          output_hemisphere = 'N';
      }

      if ((input_projection_type == CoordinateType::militaryGridReferenceSystem) || 
          (input_projection_type == CoordinateType::usNationalGrid) || (input_projection_type == CoordinateType::georef) || (input_projection_type == CoordinateType::britishNationalGrid))
      {
        // BNG string contains a space
        fscanf(input_file, " %34[^\t\n]", buf);
        input_coord1_str = (char *)malloc((strlen(buf) + 1) * sizeof(char));
        strcpy(input_coord1_str, buf);
      }
      else
      {
        fscanf(input_file, " %s", buf);
        input_coord1 = atof(buf);

        fscanf(input_file, " %s", buf);
        input_coord2 = atof(buf);

        fgetc(input_file);
        next_character = fgetc(input_file);
        ungetc(next_character, input_file);
        if (next_character != '\t')
        {
          input_height = 1;
          fscanf(input_file, " %s", buf);
          input_coord3 = atof(buf);
        }
      }

      if ((output_projection_type == CoordinateType::militaryGridReferenceSystem) || (output_projection_type == CoordinateType::usNationalGrid) || (output_projection_type == CoordinateType::georef) || (output_projection_type == CoordinateType::britishNationalGrid))
      {
        fscanf(input_file, " %34[^\t\n]", buf);
        buf_length = strlen(buf);
        output_coord1_str = (char *)malloc((buf_length + 1) * sizeof(char));
        strcpy(output_coord1_str, buf);
        if (output_coord1_str[buf_length-1] == ' ')
          output_coord1_str[buf_length-1] = '\0';

        // read calculated coord string
        fscanf(input_file, " %34[^\t\n]", buf);
      }
      else
      {
        // Read output coords
        fscanf(input_file, " %s", buf);
        output_coord1 = atof(buf);

        fscanf(input_file, " %s", buf);
        output_coord2 = atof(buf);

        fgetc(input_file);
        next_character = fgetc(input_file);
        ungetc(next_character, input_file);
        if (next_character != '\t')
        {
          output_height = 1;
          fscanf(input_file, "%34[^\t\n]", buf);
          output_coord3 = atof(buf);
        }

        // read calculated coords
        // coord 1
        fscanf(input_file, " %s", buf);

        // coord 2
        fscanf(input_file, " %s", buf);
        next_character = fgetc(input_file);
        if (next_character == '\t')
        {
          next_character = fgetc(input_file);
          ungetc(next_character, input_file);
          if ((next_character != '\t') && (next_character != '\n'))
          {
            // coord 3
            fscanf(input_file, "%s", buf);
          }
        }
        else
          ungetc(next_character, input_file);
      }

      if ((output_projection_type == CoordinateType::universalTransverseMercator) || (output_projection_type == CoordinateType::universalPolarStereographic))
      {
        // read calculated zone or hemisphere
        fscanf(input_file, " %34[^\t\n]", buf);
      }

      Read_Parameters( SourceOrTarget::source, input_projection_type );

      Read_Parameters( SourceOrTarget::target, output_projection_type );

      // Read coordinates of input projection 
      switch (input_projection_type)
      {
        case CoordinateType::albersEqualAreaConic:
        case CoordinateType::azimuthalEquidistant:
        case CoordinateType::bonne:
        case CoordinateType::cassini:
        case CoordinateType::cylindricalEqualArea:
        case CoordinateType::eckert4:
        case CoordinateType::eckert6:
        case CoordinateType::equidistantCylindrical:
        case CoordinateType::gnomonic:
        case CoordinateType::lambertConformalConic1Parallel:
        case CoordinateType::lambertConformalConic2Parallels:
        case CoordinateType::mercatorStandardParallel:
        case CoordinateType::mercatorScaleFactor:
        case CoordinateType::millerCylindrical:
        case CoordinateType::mollweide:
        case CoordinateType::neys:
        case CoordinateType::newZealandMapGrid:
        case CoordinateType::obliqueMercator:
        case CoordinateType::orthographic:
        case CoordinateType::polarStereographicStandardParallel:
        case CoordinateType::polarStereographicScaleFactor:
        case CoordinateType::polyconic:
        case CoordinateType::sinusoidal:
        case CoordinateType::stereographic:
        case CoordinateType::transverseCylindricalEqualArea:
        case CoordinateType::transverseMercator:
        case CoordinateType::vanDerGrinten:
        {
          coordinateSystemState[SourceOrTarget::source].coordinates.mapProjectionCoordinates = new MSP::CCS::MapProjectionCoordinates( input_projection_type );
          coordinateSystemState[SourceOrTarget::source].coordinates.mapProjectionCoordinates->set( input_coord1, input_coord2 );
          break;
        }
        case CoordinateType::britishNationalGrid:
        {
          coordinateSystemState[SourceOrTarget::source].coordinates.bngCoordinates = new MSP::CCS::BNGCoordinates( input_projection_type );
          coordinateSystemState[SourceOrTarget::source].coordinates.bngCoordinates->set( input_coord1_str );
          break;
        }
        case CoordinateType::geocentric:
        case CoordinateType::localCartesian:
        {
          coordinateSystemState[SourceOrTarget::source].coordinates.cartesianCoordinates = new MSP::CCS::CartesianCoordinates( input_projection_type );
          coordinateSystemState[SourceOrTarget::source].coordinates.cartesianCoordinates->set( input_coord1, input_coord2, input_coord3 );
          break;
        }
        case CoordinateType::geodetic:
        {
          double longitude = input_coord2 * PI_OVER_180;
          double latitude = input_coord1 * PI_OVER_180;

          if( longitude < -PI || longitude > 2 * PI )
          {
            error_count++;
            printf("Test Case ID: %s - Error: Source Geodetic Coordinates:  Longitude out of range \n", id);
            return COORDINATES_ERROR;
          }
         if( latitude < -PI/2 || latitude > PI/2 )
          {
            error_count++;
            printf("Test Case ID: %s - Error: Source Geodetic Coordinates:  Latitude out of range \n", id);
            return COORDINATES_ERROR;
          }

          coordinateSystemState[SourceOrTarget::source].coordinates.geodeticCoordinates = new MSP::CCS::GeodeticCoordinates( input_projection_type );
          coordinateSystemState[SourceOrTarget::source].coordinates.geodeticCoordinates->set( input_coord2 * PI_OVER_180, input_coord1 * PI_OVER_180, input_coord3 );
          break;
        }
        case CoordinateType::georef:
        {
          coordinateSystemState[SourceOrTarget::source].coordinates.georefCoordinates = new MSP::CCS::GEOREFCoordinates( input_projection_type );
          coordinateSystemState[SourceOrTarget::source].coordinates.georefCoordinates->set( input_coord1_str);
          break;
        }
        case CoordinateType::militaryGridReferenceSystem:
        case CoordinateType::usNationalGrid:
        {
          coordinateSystemState[SourceOrTarget::source].coordinates.mgrsOrUSNGCoordinates = new MSP::CCS::MGRSorUSNGCoordinates( input_projection_type );
          coordinateSystemState[SourceOrTarget::source].coordinates.mgrsOrUSNGCoordinates->set( input_coord1_str);
          break;
        }
        case CoordinateType::universalTransverseMercator:
        {
          coordinateSystemState[SourceOrTarget::source].coordinates.utmCoordinates = new MSP::CCS::UTMCoordinates( input_projection_type );
          coordinateSystemState[SourceOrTarget::source].coordinates.utmCoordinates->set( abs(zone), input_hemisphere, input_coord1, input_coord2 );
          break;
        }
        case CoordinateType::universalPolarStereographic:
        {
          coordinateSystemState[SourceOrTarget::source].coordinates.upsCoordinates = new MSP::CCS::UPSCoordinates( input_projection_type );
          coordinateSystemState[SourceOrTarget::source].coordinates.upsCoordinates->set( input_hemisphere, input_coord1, input_coord2 );
          break;
        }
        default:
        {
          break;
        }
      }

      // Results string
      next_character = fgetc(input_file);
      ungetc(next_character, input_file);
      if (next_character != '\n')
        fscanf(input_file, "%45[^\n]", buf);

      fscanf(input_file, " ");

    }
    else
    {
      error_count++;
      printf("Test Case ID: %s - Error: Invalid Output Projection: %s\n", id, output_projection);
      fgetc(input_file);
      fscanf(input_file, "%199[^\n]", buf);
      data = (char *)malloc((strlen(buf) + 1) * sizeof(char));
      strcpy(data, buf);
      fscanf(input_file, " ");
      return FALSE;
    }
  }
  else
  {
    error_count++;
    printf("Test Case ID: %s - Error: Invalid Input Projection: %s\n", id, input_projection);
    fgetc(input_file);
    fscanf(input_file, "%199[^\n]", buf);
    data = (char *)malloc((strlen(buf) + 1) * sizeof(char));
    strcpy(data, buf);
    fscanf(input_file, " ");
    return FALSE;
  }

  return TRUE;
}


/* Set input and output coordinate systems */
bool SpreadsheetTester::Set_Projection(SourceOrTarget::Enum direction, const char* projection_str/*, CoordinateType::Enum* projection_type*/)
{
  if (strstr("ALBERS_EQUAL_AREA_CONIC", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::albersEqualAreaConic;
  }
  else if (strstr("AZIMUTHAL_EQUIDISTANT_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::azimuthalEquidistant;
  }
  else if (strstr("BRITISH_NATIONAL_GRID_(BNG)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::britishNationalGrid;
  }
  else if (strstr("BONNE", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::bonne;
  }
  else if (strstr("CASSINI", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::cassini;
  }
  else if (strstr("CYLINDRICAL_EQUAL_AREA", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::cylindricalEqualArea;
  }
  else if (strstr("TRANSVERSE_CYLINDRICAL_EQUAL_AREA", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::transverseCylindricalEqualArea;
  }
  else if (strstr("ECKERT_IV_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::eckert4;
  }
  else if (strstr("ECKERT_VI_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::eckert6;
  }
  else if (strstr("EQUIDISTANT_CYLINDRICAL_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::equidistantCylindrical;
  }
  else if (strstr("GEOCENTRIC", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::geocentric;
  }
  else if (strstr("GEOREF", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::georef;
  }
  else if (strstr("GNOMONIC_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::gnomonic;
  }
  else if((strcmp("LAMBERT_CONFORMAL_CONIC", projection_str) == 0) ||
           strstr("LAMBERT_CONFORMAL_CONIC_(2_PARALLEL)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::lambertConformalConic2Parallels;
  }
  else if (strstr("LAMBERT_CONFORMAL_CONIC_(1_PARALLEL)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::lambertConformalConic1Parallel;
  }
  else if (strstr("LOCAL_CARTESIAN", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::localCartesian;
  }
  else if ((strstr("MERCATOR", projection_str)) ||
           (strstr("MERCATOR_(STANDARD_PARALLEL)", projection_str)))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::mercatorStandardParallel;
  }
  else if (strstr("MERCATOR_(SCALE_FACTOR)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::mercatorScaleFactor;
  }
  else if (strstr("OBLIQUE_MERCATOR", projection_str)) 
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::obliqueMercator;
  }
  else if (strstr("TRANSVERSE_MERCATOR", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::transverseMercator;
  }
  else if (strstr("UNIVERSAL_TRANSVERSE_MERCATOR_(UTM)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::universalTransverseMercator;
  }
  else if (strstr("MILITARY_GRID_REFERENCE_SYSTEM_(MGRS)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::militaryGridReferenceSystem;
  }
  else if (strstr("UNITED_STATES_NATIONAL_GRID_(USNG)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::usNationalGrid;
  }
  else if (strstr("MILLER_CYLINDRICAL_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::millerCylindrical;
  }
  else if (strstr("MOLLWEIDE_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::mollweide;
  }
  else if ((strstr("NEY'S_(MODIFIED_LAMBERT_CONFORMAL_CONIC)", projection_str)) || 
           (strstr("NEYS_(MODIFIED_LAMBERT_CONFORMAL_CONIC)", projection_str)))  
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::neys;
  }
  else if (strstr("NEW_ZEALAND_MAP_GRID_(NZMG)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::newZealandMapGrid;
  }
  else if (strstr("ORTHOGRAPHIC_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::orthographic;
  }
  else if (strstr("STEREOGRAPHIC_(S)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::stereographic;
  }
  else if ((strstr("POLAR_STEREOGRAPHIC", projection_str)) ||
           (strstr("POLAR_STEREOGRAPHIC_(STANDARD_PARALLEL)", projection_str)))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::polarStereographicStandardParallel;
  }
  else if (strstr("POLAR_STEREOGRAPHIC_(SCALE_FACTOR)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::polarStereographicScaleFactor;
  }
  else if (strstr("UNIVERSAL_POLAR_STEREOGRAPHIC_(UPS)", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::universalPolarStereographic;
  }
  else if (strstr("POLYCONIC", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::polyconic;
  }
  else if (strstr("SINUSOIDAL", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::sinusoidal;
  }
  else if (strstr("VAN_DER_GRINTEN", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::vanDerGrinten;
  }
  else if (strstr("GEODETIC", projection_str))
  {
    coordinateSystemState[direction].coordinateType = CoordinateType::geodetic;
  }
  else
   return FALSE;

  return TRUE;
}


/* Read the input and output projection parameters */
void SpreadsheetTester::Read_Parameters( SourceOrTarget::Enum direction, CoordinateType::Enum projection_type )
{
  char buf[28] = "";

  /* Read parameters of projection */
  switch(projection_type)
  {
    case CoordinateType::albersEqualAreaConic:
    case CoordinateType::lambertConformalConic2Parallels:
    {
      coordinateSystemState[direction].parameters.mapProjection6Parameters = new MSP::CCS::MapProjection6Parameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection6Parameters->setStandardParallel1( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection6Parameters->setStandardParallel2( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection6Parameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection6Parameters->setOriginLatitude( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection6Parameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection6Parameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::neys:
    {
      coordinateSystemState[direction].parameters.neysParameters = new MSP::CCS::NeysParameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.neysParameters->setStandardParallel1( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.neysParameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.neysParameters->setOriginLatitude( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.neysParameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.neysParameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::lambertConformalConic1Parallel:
    case CoordinateType::transverseCylindricalEqualArea:
    case CoordinateType::transverseMercator:
    {
      coordinateSystemState[direction].parameters.mapProjection5Parameters = new MSP::CCS::MapProjection5Parameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection5Parameters->setScaleFactor( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection5Parameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection5Parameters->setOriginLatitude( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection5Parameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection5Parameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::obliqueMercator:
    {
      coordinateSystemState[direction].parameters.obliqueMercatorParameters = new MSP::CCS::ObliqueMercatorParameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.obliqueMercatorParameters->setScaleFactor( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.obliqueMercatorParameters->setOriginLatitude( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.obliqueMercatorParameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.obliqueMercatorParameters->setFalseNorthing( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.obliqueMercatorParameters->setLongitude1( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.obliqueMercatorParameters->setLatitude1( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.obliqueMercatorParameters->setLongitude2( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.obliqueMercatorParameters->setLatitude2( atof(buf) * PI_OVER_180 );
      break;
    }
    case CoordinateType::eckert4:
    case CoordinateType::eckert6:
    case CoordinateType::millerCylindrical:
    case CoordinateType::mollweide:
    case CoordinateType::sinusoidal:
    case CoordinateType::vanDerGrinten:
    {
      coordinateSystemState[direction].parameters.mapProjection3Parameters = new MSP::CCS::MapProjection3Parameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection3Parameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection3Parameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection3Parameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::azimuthalEquidistant:
    case CoordinateType::bonne:
    case CoordinateType::cassini:
    case CoordinateType::cylindricalEqualArea:
    case CoordinateType::gnomonic:
    case CoordinateType::orthographic:
    case CoordinateType::polyconic:
    case CoordinateType::stereographic:
    {
      coordinateSystemState[direction].parameters.mapProjection4Parameters = new MSP::CCS::MapProjection4Parameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection4Parameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection4Parameters->setOriginLatitude( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection4Parameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mapProjection4Parameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::equidistantCylindrical:
    {
      coordinateSystemState[direction].parameters.equidistantCylindricalParameters = new MSP::CCS::EquidistantCylindricalParameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.equidistantCylindricalParameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.equidistantCylindricalParameters->setStandardParallel( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.equidistantCylindricalParameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.equidistantCylindricalParameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::mercatorStandardParallel:
    {
      coordinateSystemState[direction].parameters.mercatorStandardParallelParameters = new MSP::CCS::MercatorStandardParallelParameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mercatorStandardParallelParameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mercatorStandardParallelParameters->setStandardParallel( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mercatorStandardParallelParameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mercatorStandardParallelParameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::mercatorScaleFactor:
    {
      coordinateSystemState[direction].parameters.mercatorScaleFactorParameters = new MSP::CCS::MercatorScaleFactorParameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mercatorScaleFactorParameters->setScaleFactor( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mercatorScaleFactorParameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mercatorScaleFactorParameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.mercatorScaleFactorParameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::polarStereographicStandardParallel:
    {
      coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters = new MSP::CCS::PolarStereographicStandardParallelParameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters->setStandardParallel( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters->setFalseNorthing( atof(buf) );
      break;
    }
    case CoordinateType::polarStereographicScaleFactor:
    {
      coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters = new MSP::CCS::PolarStereographicScaleFactorParameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters->setScaleFactor( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters->setCentralMeridian( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters->setFalseEasting( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters->setFalseNorthing( atof(buf) );
      if( direction == SourceOrTarget::source )
        coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters->setHemisphere( input_hemisphere );
      else
        coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters->setHemisphere( output_hemisphere );
      break;
    }
    case CoordinateType::geodetic:
    {
      coordinateSystemState[direction].parameters.geodeticParameters = new MSP::CCS::GeodeticParameters( projection_type );
      // set height to EGM2008 if environment variable is set
      if (testEGM2008)
         coordinateSystemState[direction].parameters.geodeticParameters->setHeightType( HeightType::EGM2008TwoPtFiveMinBicubicSpline );
      else
         coordinateSystemState[direction].parameters.geodeticParameters->setHeightType( HeightType::ellipsoidHeight );
      break;
    }
    case CoordinateType::localCartesian:
    {
      coordinateSystemState[direction].parameters.localCartesianParameters = new MSP::CCS::LocalCartesianParameters( projection_type );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.localCartesianParameters->setLongitude( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.localCartesianParameters->setLatitude( atof(buf) * PI_OVER_180 );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.localCartesianParameters->setHeight( atof(buf) );
      fscanf(input_file, " %s", buf);
      coordinateSystemState[direction].parameters.localCartesianParameters->setOrientation( atof(buf) * PI_OVER_180 );
      break;
    }
    case CoordinateType::universalTransverseMercator:
    {
      coordinateSystemState[direction].parameters.utmParameters = new MSP::CCS::UTMParameters( projection_type );
      coordinateSystemState[direction].parameters.utmParameters->set( abs(zone), 1 );
      break;
    }
    default:
    {
      coordinateSystemState[direction].parameters.coordinateSystemParameters = new MSP::CCS::CoordinateSystemParameters( projection_type );
      break;
    }
  }

///  return errorStatus;
}


MSP::CCS::CoordinateSystemParameters* SpreadsheetTester::getCoordinateSystem( const SourceOrTarget::Enum direction )
{
/*
 *  The function getCoordinateSystem returns the current coordinate system
 *  type.
 *
 *  direction  : Indicates whether the coordinate system is to be used for
 *               source or target                                       (input)
 *  parameters : Identifies current coordinate system type             (output)
 */

  switch( coordinateSystemState[direction].coordinateType )
  {
    case CoordinateType::albersEqualAreaConic:
    case CoordinateType::lambertConformalConic2Parallels:
      return coordinateSystemState[direction].parameters.mapProjection6Parameters;
    case CoordinateType::azimuthalEquidistant:
    case CoordinateType::bonne:
    case CoordinateType::cassini:
    case CoordinateType::cylindricalEqualArea:
    case CoordinateType::gnomonic:
    case CoordinateType::orthographic:
    case CoordinateType::polyconic:
    case CoordinateType::stereographic:
      return coordinateSystemState[direction].parameters.mapProjection4Parameters;
    case CoordinateType::eckert4:
    case CoordinateType::eckert6:
    case CoordinateType::millerCylindrical:
    case CoordinateType::mollweide:
    case CoordinateType::sinusoidal:
    case CoordinateType::vanDerGrinten:
      return coordinateSystemState[direction].parameters.mapProjection3Parameters;
    case CoordinateType::equidistantCylindrical:
      return coordinateSystemState[direction].parameters.equidistantCylindricalParameters;
    case CoordinateType::geodetic:
      return coordinateSystemState[direction].parameters.geodeticParameters;
    case CoordinateType::lambertConformalConic1Parallel:
    case CoordinateType::transverseMercator:
    case CoordinateType::transverseCylindricalEqualArea:
      return coordinateSystemState[direction].parameters.mapProjection5Parameters;
    case CoordinateType::localCartesian:
      return coordinateSystemState[direction].parameters.localCartesianParameters;
    case CoordinateType::mercatorStandardParallel:
      return coordinateSystemState[direction].parameters.mercatorStandardParallelParameters;
     ///    return ((Mercator*)(coordinateSystemState[direction].coordinateSystem))->getParameters(); // gets the calculated scale factor
    case CoordinateType::mercatorScaleFactor:
      return coordinateSystemState[direction].parameters.mercatorScaleFactorParameters;
    case CoordinateType::neys:
      return coordinateSystemState[direction].parameters.neysParameters;
    case CoordinateType::obliqueMercator:
      return coordinateSystemState[direction].parameters.obliqueMercatorParameters;
    case CoordinateType::polarStereographicStandardParallel:
      return coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters;
    case CoordinateType::polarStereographicScaleFactor:
      return coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters;
    case CoordinateType::universalTransverseMercator:
      return coordinateSystemState[direction].parameters.utmParameters;
    case CoordinateType::britishNationalGrid:
    case CoordinateType::geocentric:
    case CoordinateType::georef:
    case CoordinateType::globalAreaReferenceSystem:
    case CoordinateType::militaryGridReferenceSystem:
    case CoordinateType::newZealandMapGrid:
    case CoordinateType::universalPolarStereographic:
    case CoordinateType::usNationalGrid:
      return coordinateSystemState[direction].parameters.coordinateSystemParameters;
    default:
      throw CoordinateConversionException(ErrorMessages::invalidType);
  }
}


void SpreadsheetTester::convertSourceToTarget()
{
  Accuracy sourceAccuracy;
  Accuracy targetAccuracy;
  MSP::CCS::CoordinateTuple* _sourceCoordinates;
  
  switch( coordinateSystemState[SourceOrTarget::source].coordinateType )
  {
    case CoordinateType::albersEqualAreaConic:
    case CoordinateType::azimuthalEquidistant:
    case CoordinateType::bonne:
    case CoordinateType::cassini:
    case CoordinateType::cylindricalEqualArea:
    case CoordinateType::eckert4:
    case CoordinateType::eckert6:
    case CoordinateType::equidistantCylindrical:
    case CoordinateType::gnomonic:
    case CoordinateType::lambertConformalConic1Parallel:
    case CoordinateType::lambertConformalConic2Parallels:
    case CoordinateType::mercatorStandardParallel:
    case CoordinateType::mercatorScaleFactor:
    case CoordinateType::millerCylindrical:
    case CoordinateType::mollweide:
    case CoordinateType::neys:
    case CoordinateType::newZealandMapGrid:
    case CoordinateType::obliqueMercator:
    case CoordinateType::orthographic:
    case CoordinateType::polarStereographicStandardParallel:
    case CoordinateType::polarStereographicScaleFactor:
    case CoordinateType::polyconic:
    case CoordinateType::sinusoidal:
    case CoordinateType::stereographic:
    case CoordinateType::transverseCylindricalEqualArea:
    case CoordinateType::transverseMercator:
    case CoordinateType::vanDerGrinten:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.mapProjectionCoordinates;
      break;
    }
    case CoordinateType::militaryGridReferenceSystem:
    case CoordinateType::usNationalGrid:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.mgrsOrUSNGCoordinates;
      break;
    }
    case CoordinateType::britishNationalGrid:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.bngCoordinates;
      break;
    }
    case CoordinateType::globalAreaReferenceSystem:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.garsCoordinates;
      break;
    }
    case CoordinateType::georef:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.georefCoordinates;
      break;
    }
    case CoordinateType::geocentric:
    case CoordinateType::localCartesian:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.cartesianCoordinates;
      break;
    }
    case CoordinateType::geodetic:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.geodeticCoordinates;
      break;
    }
    case CoordinateType::universalPolarStereographic:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.upsCoordinates;
      break;
    }
    case CoordinateType::universalTransverseMercator:
    {
      _sourceCoordinates = coordinateSystemState[SourceOrTarget::source].coordinates.utmCoordinates;
      break;
    }
  }


  switch( coordinateSystemState[SourceOrTarget::target].coordinateType )
  {
    case CoordinateType::albersEqualAreaConic:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::albersEqualAreaConic);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::azimuthalEquidistant:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::azimuthalEquidistant);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::bonne:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::bonne);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::britishNationalGrid:
    {
      MSP::CCS::BNGCoordinates* _targetCoordinates = new MSP::CCS::BNGCoordinates(CoordinateType::britishNationalGrid);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.bngCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.bngCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::cassini:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::cassini);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::cylindricalEqualArea:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::cylindricalEqualArea);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
     }

      break;
    }
    case CoordinateType::eckert4:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::eckert4);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::eckert6:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::eckert6);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::equidistantCylindrical:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::equidistantCylindrical);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::geocentric:
    {
      MSP::CCS::CartesianCoordinates* _targetCoordinates = new MSP::CCS::CartesianCoordinates(CoordinateType::geocentric);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.cartesianCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.cartesianCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
     }

      break;
    }
    case CoordinateType::geodetic:
    {
      MSP::CCS::GeodeticCoordinates* _targetCoordinates = new MSP::CCS::GeodeticCoordinates(CoordinateType::geodetic);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.geodeticCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.geodeticCoordinates = _targetCoordinates;
       throw CoordinateConversionException(e.getMessage());
     }

      break;
    }
    case CoordinateType::georef:
    {
      MSP::CCS::GEOREFCoordinates* _targetCoordinates = new MSP::CCS::GEOREFCoordinates(CoordinateType::georef);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.georefCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.georefCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::globalAreaReferenceSystem:
    {
      MSP::CCS::GARSCoordinates* _targetCoordinates = new MSP::CCS::GARSCoordinates(CoordinateType::globalAreaReferenceSystem);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.garsCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.garsCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::gnomonic:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::gnomonic);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
     }

      break;
    }
    case CoordinateType::lambertConformalConic1Parallel:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::lambertConformalConic1Parallel);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::lambertConformalConic2Parallels:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::lambertConformalConic2Parallels);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::localCartesian:
    {
      MSP::CCS::CartesianCoordinates* _targetCoordinates = new MSP::CCS::CartesianCoordinates(CoordinateType::localCartesian);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.cartesianCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.cartesianCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
     }

      break;
    }
    case CoordinateType::mercatorStandardParallel:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::mercatorStandardParallel);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::mercatorScaleFactor:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::mercatorScaleFactor);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::militaryGridReferenceSystem:
    {
      MSP::CCS::MGRSorUSNGCoordinates* _targetCoordinates = new MSP::CCS::MGRSorUSNGCoordinates(CoordinateType::militaryGridReferenceSystem);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mgrsOrUSNGCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mgrsOrUSNGCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::millerCylindrical:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::millerCylindrical);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::mollweide:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::mollweide);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::newZealandMapGrid:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::newZealandMapGrid);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::neys:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::neys);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::obliqueMercator:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::obliqueMercator);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::orthographic:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::orthographic);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::polarStereographicStandardParallel:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::polarStereographicStandardParallel);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::polarStereographicScaleFactor:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::polarStereographicScaleFactor);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::polyconic:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::polyconic);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::sinusoidal:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::sinusoidal);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::stereographic:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::stereographic);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::transverseCylindricalEqualArea:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::transverseCylindricalEqualArea);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::transverseMercator:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::transverseMercator);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::universalPolarStereographic:
    {
      MSP::CCS::UPSCoordinates* _targetCoordinates = new MSP::CCS::UPSCoordinates(CoordinateType::universalPolarStereographic);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.upsCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.upsCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::universalTransverseMercator:
    {
      MSP::CCS::UTMCoordinates* _targetCoordinates = new MSP::CCS::UTMCoordinates(CoordinateType::universalTransverseMercator);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.utmCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.utmCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::usNationalGrid:
    {
      MSP::CCS::MGRSorUSNGCoordinates* _targetCoordinates = new MSP::CCS::MGRSorUSNGCoordinates(CoordinateType::usNationalGrid);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mgrsOrUSNGCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mgrsOrUSNGCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    case CoordinateType::vanDerGrinten:
    {
      MSP::CCS::MapProjectionCoordinates* _targetCoordinates = new MSP::CCS::MapProjectionCoordinates(CoordinateType::vanDerGrinten);
      try
      {
        coordinateConversionService->convertSourceToTarget( _sourceCoordinates, &sourceAccuracy, *_targetCoordinates, targetAccuracy );
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
      }
      catch(CoordinateConversionException e)
      {
        _targetCoordinates->setErrorMessage(e.getMessage());
        coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates = _targetCoordinates;
        throw CoordinateConversionException(e.getMessage());
      }

      break;
    }
    default:
      throw CoordinateConversionException(ErrorMessages::invalidType);
  }

}


/* Write the converted row to the output file, also test if the calculated coordinates */
/* agree with the expected coordinates */
void SpreadsheetTester::Write_To_File(bool file_read_error)
{
  BOOLEAN warning_different = FALSE;
  BOOLEAN warning_equivalent = FALSE;
  BOOLEAN successful_different = FALSE;
  BOOLEAN successful_equivalent = FALSE;

  fprintf(output_file, "%s\t%s\t%s\t%s\t%s\t", id, 
                                               input_datum, 
                                               input_projection, 
                                               output_datum, 
                                               output_projection);
  if (!file_read_error)
  {
    CoordinateType::Enum input_projection_type = coordinateSystemState[SourceOrTarget::source].coordinateType;
    CoordinateType::Enum output_projection_type = coordinateSystemState[SourceOrTarget::target].coordinateType;

    if (input_projection_type == CoordinateType::universalTransverseMercator)
      fprintf(output_file, "%d\t", zone);
    else if ((input_projection_type == CoordinateType::universalPolarStereographic) || 
             (input_projection_type == CoordinateType::polarStereographicScaleFactor))
      fprintf(output_file, "%c\t", input_hemisphere);
    else
      fprintf(output_file, "\t");

    if (output_projection_type == CoordinateType::universalTransverseMercator)
      fprintf(output_file, "%d\t", zone);
    else if ((output_projection_type  == CoordinateType::universalPolarStereographic) ||
             (output_projection_type  == CoordinateType::polarStereographicScaleFactor))
      fprintf(output_file, "%c\t", output_hemisphere);
    else
      fprintf(output_file, "\t");

    if ((input_projection_type == CoordinateType::militaryGridReferenceSystem) || (input_projection_type == CoordinateType::usNationalGrid) || (input_projection_type == CoordinateType::georef) || (input_projection_type == CoordinateType::britishNationalGrid))
      fprintf(output_file, "%s\t\t\t", input_coord1_str);
    else
    {
      fprintf(output_file, "%f\t%f\t", input_coord1, input_coord2);
      if (input_height)
        fprintf(output_file, "%f\t", input_coord3);
      else
        fprintf(output_file, "\t");
    }

    if ((output_projection_type == CoordinateType::militaryGridReferenceSystem) || (output_projection_type == CoordinateType::usNationalGrid) || (output_projection_type == CoordinateType::georef) || (output_projection_type == CoordinateType::britishNationalGrid))
      fprintf(output_file, "%s\t\t\t", output_coord1_str);
    else
    {
      fprintf(output_file, "%f\t%f\t", output_coord1, output_coord2);
      if (output_height)
        fprintf(output_file, "%f\t", output_coord3);
      else
        fprintf(output_file, "\t");
    }

    /* Write calculated coordinates to file */
    switch (output_projection_type)
    {
      case CoordinateType::albersEqualAreaConic:
      case CoordinateType::azimuthalEquidistant:
      case CoordinateType::bonne:
      case CoordinateType::cassini:
      case CoordinateType::cylindricalEqualArea:
      case CoordinateType::eckert4:
      case CoordinateType::eckert6:
      case CoordinateType::equidistantCylindrical:
      case CoordinateType::gnomonic:
      case CoordinateType::lambertConformalConic1Parallel:
      case CoordinateType::lambertConformalConic2Parallels:
      case CoordinateType::mercatorStandardParallel:
      case CoordinateType::mercatorScaleFactor:
      case CoordinateType::millerCylindrical:
      case CoordinateType::mollweide:
      case CoordinateType::neys:
      case CoordinateType::newZealandMapGrid:
      case CoordinateType::obliqueMercator:
      case CoordinateType::orthographic:
      case CoordinateType::polarStereographicStandardParallel:
      case CoordinateType::polarStereographicScaleFactor:
      case CoordinateType::polyconic:
      case CoordinateType::sinusoidal:
      case CoordinateType::stereographic:
      case CoordinateType::transverseCylindricalEqualArea:
      case CoordinateType::transverseMercator:
      case CoordinateType::vanDerGrinten:
      {
        if( coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates )
        {
          MSP::CCS::MapProjectionCoordinates mapProjectionCoordinates = *coordinateSystemState[SourceOrTarget::target].coordinates.mapProjectionCoordinates;
          if (strlen( mapProjectionCoordinates.errorMessage() ) > 0 )
            fprintf(output_file, "n/a\tn/a\t\t\t");
          else
          {
            double easting = mapProjectionCoordinates.easting();
            double northing = mapProjectionCoordinates.northing();

            fprintf(output_file, "%.6f\t%.6f\t\t\t", easting, northing);

            if ((fabs(easting - output_coord1) > 1.0) || (fabs(northing - output_coord2) > 1.0))
            {
              if (strlen( mapProjectionCoordinates.warningMessage() ) > 0 )
              {
                warning_different = TRUE;
                warning_different_count++;
                printf("Test Case Id: %s - Expected: %.6f, %.6f   Calculated: %.6f, %.6f - Warning: %s\n", id, output_coord1, output_coord2, easting, northing, mapProjectionCoordinates.warningMessage()); 
              }
              else
              {
                successful_different = TRUE;
                successful_different_count++;
                printf("Test Case Id: %s - Expected: %.6f, %.6f   Calculated: %.6f, %.6f \n", id, output_coord1, output_coord2, easting, northing); 
              }
            }
            else
            {
              if (strlen( mapProjectionCoordinates.warningMessage() ) > 0 )
              { 
                warning_equivalent = TRUE;
                warning_equivalent_count++; 
                if( showWarnings )
                  printf("Test Case Id: %s - Warning: %s\n", id, mapProjectionCoordinates.warningMessage()); 
             }
              else
              {
                successful_equivalent = TRUE;
                successful_equivalent_count++;
              }
            }
          }
        }
        else
          fprintf(output_file, "n/a\tn/a\t\t\t");

        break;
      }
      case CoordinateType::britishNationalGrid:
      {
        if( coordinateSystemState[SourceOrTarget::target].coordinates.bngCoordinates )
        {
          MSP::CCS::BNGCoordinates bngCoordinates = *coordinateSystemState[SourceOrTarget::target].coordinates.bngCoordinates;
          if (strlen( bngCoordinates.errorMessage() ) > 0 )
            fprintf(output_file, "n/a\t\t\t\t");
          else
          {
            fprintf(output_file, "%s\t\t\t\t", bngCoordinates.BNGString());
            if (strncmp(bngCoordinates.BNGString(), output_coord1_str, strlen(output_coord1_str)))
            {
              if (strlen( bngCoordinates.warningMessage() ) > 0 )
              {
                warning_different = TRUE;
                warning_different_count++;
                printf("Test Case Id: %s - Expected: %s   Calculated: %s - Warning: %s\n", id, output_coord1_str, bngCoordinates.BNGString(), bngCoordinates.warningMessage()); 
              }
              else
              {
                successful_different = TRUE;
                successful_different_count++;
                printf("Test Case Id: %s - Expected: %s   Calculated: %s \n", id, output_coord1_str, bngCoordinates.BNGString()); 
              }
            }
            else
            {
              if (strlen( bngCoordinates.warningMessage() ) > 0 )
              { 
                warning_equivalent = TRUE;
                warning_equivalent_count++; 
                if( showWarnings )
                  printf("Test Case Id: %s - Warning: %s\n", id, bngCoordinates.warningMessage()); 
              }
              else
              {
                successful_equivalent = TRUE;
                successful_equivalent_count++;
              }
            }
          }
        }
        else
          fprintf(output_file, "n/a\t\t\t\t");

        break;
      }
      case CoordinateType::geocentric:
      case CoordinateType::localCartesian:
      {
        if( coordinateSystemState[SourceOrTarget::target].coordinates.cartesianCoordinates )
        {
          MSP::CCS::CartesianCoordinates cartesianCoordinates = *coordinateSystemState[SourceOrTarget::target].coordinates.cartesianCoordinates;
          if (strlen( cartesianCoordinates.errorMessage() ) > 0 )
            fprintf(output_file, "n/a\tn/a\tn/a\t\t");
          else
          {
            double x = cartesianCoordinates.x();
            double y = cartesianCoordinates.y();
            double z = cartesianCoordinates.z();

            fprintf(output_file, "%.6f\t%.6f\t%.6f\t\t", x, y, z);
            if ((fabs(x - output_coord1) > 2.0) || (fabs(y - output_coord2) > 2.0) || (fabs(z - output_coord3) > 2.0))
            {
              if (strlen( cartesianCoordinates.warningMessage() ) > 0 )
              {
                warning_different = TRUE;
                warning_different_count++;
                printf("Test Case Id: %s - Expected: %.6f, %.6f, %.6f   Calculated: %.6f, %.6f, %.6f - Warning: %s\n", id, output_coord1, output_coord2, output_coord3, x, y, z, cartesianCoordinates.warningMessage()); 
              }
              else
              {
                successful_different = TRUE;
                successful_different_count++;
                printf("Test Case Id: %s - Expected: %.6f, %.6f, %.6f   Calculated: %.6f, %.6f, %.6f \n", id, output_coord1, output_coord2, output_coord3, x, y, z); 
              }
            }
            else
            {
              if (strlen( cartesianCoordinates.warningMessage() ) > 0 )
              { 
                warning_equivalent = TRUE;
                warning_equivalent_count++; 
                if( showWarnings )
                  printf("Test Case Id: %s - Warning: %s\n", id, cartesianCoordinates.warningMessage()); 
              }
              else
              {
                successful_equivalent = TRUE;
                successful_equivalent_count++;
              }
            }
          }
        }
        else
          fprintf(output_file, "n/a\tn/a\tn/a\t\t");

        break;
      }
      case CoordinateType::geodetic:
      {
        if( coordinateSystemState[SourceOrTarget::target].coordinates.geodeticCoordinates )
        {
          MSP::CCS::GeodeticCoordinates geodeticCoordinates = *coordinateSystemState[SourceOrTarget::target].coordinates.geodeticCoordinates;
          if (strlen( geodeticCoordinates.errorMessage() ) > 0 )
            fprintf(output_file, "n/a\tn/a\tn/a\t\t");
          else
          {
            double longitude = geodeticCoordinates.longitude();
            double latitude = geodeticCoordinates.latitude();
            double height = geodeticCoordinates.height();

            fprintf(output_file, "%.6f\t", latitude * _180_OVER_PI);
            if (output_coord2 > PI_DEGREES)
            {
              output_coord2 -= TWO_PI_DEGREES;
            }
            if (output_coord2 < -PI_DEGREES)
            {
              output_coord2 += TWO_PI_DEGREES;
            }
            if (longitude > PI)
              longitude -= TWO_PI;
            if (longitude < -PI)
              longitude += TWO_PI;
            if (output_height)
            {
              fprintf(output_file, "%.6f\t%.6f\t\t", longitude * _180_OVER_PI, height);
              // longitude test is used to check for -180, 180 cases
              if ((fabs(latitude * _180_OVER_PI - output_coord1) > .172) || ((fabs(longitude * _180_OVER_PI - output_coord2) > .172) && (fabs(longitude * _180_OVER_PI) - fabs(output_coord2)) > .1) || (fabs(height - output_coord3) > 2.0))
              {
                if (strlen( geodeticCoordinates.warningMessage() ) > 0 )
                {
                  warning_different = TRUE;
                  warning_different_count++;
                  printf("Test Case Id: %s - Expected: %.6f, %.6f, %.6f   Calculated: %.6f, %.6f, %.6f - Warning: %s\n", id, output_coord2, output_coord1, output_coord3, longitude * _180_OVER_PI, latitude * _180_OVER_PI, height, geodeticCoordinates.warningMessage()); 
                }
                else
                {
                  successful_different = TRUE;
                  successful_different_count++;
                  printf("Test Case Id: %s - Expected: %.6f, %.6f, %.6f   Calculated: %.6f, %.6f, %.6f \n", id, output_coord2, output_coord1, output_coord3, longitude * _180_OVER_PI, latitude * _180_OVER_PI, height); 
                }
              }
              else
              {
                if (strlen( geodeticCoordinates.warningMessage() ) > 0 )
                { 
                  warning_equivalent = TRUE;
                  warning_equivalent_count++; 
                  if( showWarnings )
                    printf("Test Case Id: %s - Warning: %s\n", id, geodeticCoordinates.warningMessage()); 
                }
                else
                {
                  successful_equivalent = TRUE;
                  successful_equivalent_count++;
                }
              }
            }
            else
            {
              fprintf(output_file, "%.6f\t\t\t", longitude * _180_OVER_PI);
  ////            if ((fabs(latitude * _180_OVER_PI - output_coord1) > .00001) || ((fabs(longitude * _180_OVER_PI - output_coord2) > .00001) && (fabs(longitude * _180_OVER_PI) - fabs(output_coord2)) > .1))
              if ((fabs(latitude * _180_OVER_PI - output_coord1) > .172) || ((fabs(longitude * _180_OVER_PI - output_coord2) > .172) && (fabs(longitude * _180_OVER_PI) - fabs(output_coord2)) > .1))
              {
                if (strlen( geodeticCoordinates.warningMessage() ) > 0 )
                {
                  warning_different = TRUE;
                  warning_different_count++;
                  printf("Test Case Id: %s - Expected: %.6f, %.6f   Calculated: %.6f, %.6f - Warning: %s\n", id, output_coord2, output_coord1, longitude * _180_OVER_PI, latitude * _180_OVER_PI, geodeticCoordinates.warningMessage()); 
                }
                else
                {
                  successful_different = TRUE;
                  successful_different_count++;
                  printf("Test Case Id: %s - Expected: %.6f, %.6f   Calculated: %.6f, %.6f \n", id, output_coord2, output_coord1, longitude * _180_OVER_PI, latitude * _180_OVER_PI); 
                }
              }
              else
              {
                if (strlen( geodeticCoordinates.warningMessage() ) > 0 )
                { 
                  warning_equivalent = TRUE;
                  warning_equivalent_count++; 
                  if( showWarnings )
                    printf("Test Case Id: %s - Warning: %s\n", id, geodeticCoordinates.warningMessage()); 
                }
                else
                {
                  successful_equivalent = TRUE;
                  successful_equivalent_count++;
                }
              }
            }
          }
        }
        else
          fprintf(output_file, "n/a\tn/a\tn/a\t\t");

        break;
      }
      case CoordinateType::georef:
      {
        if( coordinateSystemState[SourceOrTarget::target].coordinates.georefCoordinates )
        {
          MSP::CCS::GEOREFCoordinates georefCoordinates = *coordinateSystemState[SourceOrTarget::target].coordinates.georefCoordinates;
          if (strlen( georefCoordinates.errorMessage() ) > 0 )
            fprintf(output_file, "n/a\t\t\t\t");
          else
          {
            fprintf(output_file, "%s\t\t\t\t", georefCoordinates.GEOREFString());
            if (!Compare_GEOREF_Result(output_coord1_str, georefCoordinates.GEOREFString()))
            {
              if (strlen( georefCoordinates.warningMessage() ) > 0 )
              {
                warning_different = TRUE;
                warning_different_count++;
                printf("Test Case Id: %s - Expected: %s   Calculated: %s - Warning: %s\n", id, output_coord1_str, georefCoordinates.GEOREFString(), georefCoordinates.warningMessage()); 
              }
              else
              {
                successful_different = TRUE;
                successful_different_count++;
                printf("Test Case Id: %s - Expected: %s   Calculated: %s \n", id, output_coord1_str, georefCoordinates.GEOREFString()); 
              }
            }
            else
            {
              if (strlen( georefCoordinates.warningMessage() ) > 0 )
              { 
                warning_equivalent = TRUE;
                warning_equivalent_count++; 
                if( showWarnings )
                  printf("Test Case Id: %s - Warning: %s\n", id, georefCoordinates.warningMessage()); 
              }
              else
              {
                successful_equivalent = TRUE;
                successful_equivalent_count++;
              }
            }
          }
        }
        else
          fprintf(output_file, "n/a\t\t\t\t");

        break;
      }
      case CoordinateType::militaryGridReferenceSystem:
      case CoordinateType::usNationalGrid:
      {
        if( coordinateSystemState[SourceOrTarget::target].coordinates.mgrsOrUSNGCoordinates )
        {
          MSP::CCS::MGRSorUSNGCoordinates mgrsOrUSNGCoordinates = *coordinateSystemState[SourceOrTarget::target].coordinates.mgrsOrUSNGCoordinates;
          if (strlen( mgrsOrUSNGCoordinates.errorMessage() ) > 0 )
            fprintf(output_file, "n/a\t\t\t\t");
          else
          {
            fprintf(output_file, "%s\t\t\t\t", mgrsOrUSNGCoordinates.MGRSString());
            if (!Compare_MGRS_Result(output_coord1_str, mgrsOrUSNGCoordinates.MGRSString()))
            {
              if (strlen( mgrsOrUSNGCoordinates.warningMessage() ) > 0 )
              {
                warning_different = TRUE;
                warning_different_count++;
                printf("Test Case Id: %s - Expected: %s   Calculated: %s - Warning: %s\n", id, output_coord1_str, mgrsOrUSNGCoordinates.MGRSString(), mgrsOrUSNGCoordinates.warningMessage()); 
              }
              else
              {
                successful_different = TRUE;
                successful_different_count++;
                printf("Test Case Id: %s - Expected: %s   Calculated: %s \n", id, output_coord1_str, mgrsOrUSNGCoordinates.MGRSString()); 
              }
            }
            else
            {
              if (strlen( mgrsOrUSNGCoordinates.warningMessage() ) > 0 )
              { 
                warning_equivalent = TRUE;
                warning_equivalent_count++; 
                if( showWarnings )
                  printf("Test Case Id: %s - Warning: %s\n", id, mgrsOrUSNGCoordinates.warningMessage()); 
              }
              else
              {
                successful_equivalent = TRUE;
                successful_equivalent_count++;
              }
            }
          }
        }
        else
          fprintf(output_file, "n/a\t\t\t\t");

        break;
      }
      case CoordinateType::universalPolarStereographic:
      {
        if( coordinateSystemState[SourceOrTarget::target].coordinates.upsCoordinates )
        {
          MSP::CCS::UPSCoordinates upsCoordinates = *coordinateSystemState[SourceOrTarget::target].coordinates.upsCoordinates;
          if (strlen( upsCoordinates.errorMessage() ) > 0 )
            fprintf(output_file, "n/a\tn/a\t\tn/a\t");
          else
          {
            BOOLEAN error = FALSE;

            char hemisphere = upsCoordinates.hemisphere();
            double easting = upsCoordinates.easting();
            double northing = upsCoordinates.northing();

            fprintf(output_file, "%.6f\t%.6f\t\t%c\t", easting, northing, hemisphere);
            if ((fabs(easting - output_coord1) > 1.0) || (fabs(northing - output_coord2) > 1.0))
            {
              error = TRUE;
              printf("Test Case Id: %s - Expected: %.6f, %.6f   Calculated: %.6f, %.6f ", id, output_coord1, output_coord2, easting, northing); 
            }
            if (hemisphere != output_hemisphere)
            {
              if (error)
                printf("\n");

              error = TRUE;
              printf("Test Case Id: %s - Expected Hemisphere: %c   Calculated Hemisphere: %c ", id, output_hemisphere, hemisphere); 
            }
            if (error)
            {
              if (strlen( upsCoordinates.warningMessage() ) > 0 )
              {
                warning_different = TRUE;
                warning_different_count++;
                printf("- Warning: %s\n", upsCoordinates.warningMessage());
              }
              else
              {
                successful_different = TRUE;
                successful_different_count++;
                printf("\n");
              }
            }
            else
            {
              if (strlen( upsCoordinates.warningMessage() ) > 0 )
              { 
                warning_equivalent = TRUE;
                warning_equivalent_count++; 
                if( showWarnings )
                  printf("Test Case Id: %s - Warning: %s\n", id, upsCoordinates.warningMessage()); 
              }
              else
              {
                successful_equivalent = TRUE;
                successful_equivalent_count++;
              }
            }
          }
        }
        else
          fprintf(output_file, "n/a\tn/a\t\tn/a\t");

        break;
      }
      case CoordinateType::universalTransverseMercator:
      {
        if( coordinateSystemState[SourceOrTarget::target].coordinates.utmCoordinates )
        {
          MSP::CCS::UTMCoordinates utmCoordinates = *coordinateSystemState[SourceOrTarget::target].coordinates.utmCoordinates;
          if (strlen( utmCoordinates.errorMessage() ) > 0 )
            fprintf(output_file, "n/a\tn/a\t\tn/a\t");
          else
          {
            BOOLEAN error = FALSE;

            long zone = utmCoordinates.zone();
            char hemisphere = utmCoordinates.hemisphere();
            double easting = utmCoordinates.easting();
            double northing = utmCoordinates.northing();

            if (hemisphere == 'N')
                fprintf(output_file, "%.6f\t%.6f\t\t%d\t", easting, northing, zone);
            else if (hemisphere == 'S')
                fprintf(output_file, "%.6f\t%.6f\t\t%d\t", easting, northing, -zone);
            if ((fabs(easting - output_coord1) > 1.0) || (fabs(northing - output_coord2) > 1.0))
            {
              error = TRUE;
              printf("Test Case Id: %s - Expected: %.6f, %.6f   Calculated: %.6f, %.6f ", id, output_coord1, output_coord2, easting, northing); 
            }
            if (zone != abs(zone))
            {
              if (error)
                printf("\n");

              error = TRUE;
              printf("Test Case Id: %s - Expected Zone: %d   Calculated Zone: %d ", id, zone, zone); 
            }
            if (hemisphere != output_hemisphere)
            {
              if (error)
                printf("\n");

              error = TRUE;
              printf("Test Case Id: %s - Expected Hemisphere: %c   Calculated Hemisphere: %c ", id, output_hemisphere, hemisphere); 
            }
            if (error)
            {
              if (strlen( utmCoordinates.warningMessage() ) > 0 )
              {
                warning_different = TRUE;
                warning_different_count++;
                printf("- Warning: %s\n", utmCoordinates.warningMessage());
              }
              else
              {
                successful_different = TRUE;
                successful_different_count++;
                printf("\n");
              }
            }
            else
            {
              if (strlen( utmCoordinates.warningMessage() ) > 0 )
              { 
                warning_equivalent = TRUE;
                warning_equivalent_count++; 
                if( showWarnings )
                  printf("Test Case Id: %s - Warning: %s\n", id, utmCoordinates.warningMessage()); 
              }
              else
              {
                successful_equivalent = TRUE;
                successful_equivalent_count++;
              }
            }
          }
        }
        else
          fprintf(output_file, "n/a\tn/a\t\tn/a\t");

        break;
      }
      default:
      {
        break;
      }
    }

    Write_Parameters( SourceOrTarget::source, input_projection_type );

    Write_Parameters( SourceOrTarget::target, output_projection_type );

    if (successful_equivalent)
      fprintf(output_file, "Successful-Equivalent\n");
    else if (successful_different)
      fprintf(output_file, "Successful-Different\n");
    else if (warning_equivalent)
      fprintf(output_file, "Warning-Equivalent\n");
    else if (warning_different)
      fprintf(output_file, "Warning-Different\n");
    else
      fprintf(output_file, "Error\n");
  }
  else
  {
    fprintf(output_file, "%s", data);
    fprintf(output_file, "Error\n");
  }
}


void SpreadsheetTester::Write_Parameters( SourceOrTarget::Enum direction, CoordinateType::Enum projectionType )
{
  /* Write projection parameters to file */
  switch(projectionType)
  {
    case CoordinateType::albersEqualAreaConic:
    case CoordinateType::lambertConformalConic2Parallels:
    {
      MSP::CCS::MapProjection6Parameters mapProjection6Parameters = *coordinateSystemState[direction].parameters.mapProjection6Parameters;

      fprintf( output_file, "%f\t%f\t%f\t%f\t%f\t%f\t\t\t\t\t", mapProjection6Parameters.standardParallel1() * _180_OVER_PI,
                                                                mapProjection6Parameters.standardParallel2() * _180_OVER_PI,
                                                                mapProjection6Parameters.centralMeridian() * _180_OVER_PI,
                                                                mapProjection6Parameters.originLatitude() * _180_OVER_PI,
                                                                mapProjection6Parameters.falseEasting(),
                                                                mapProjection6Parameters.falseNorthing() );
      break;
    }
    case CoordinateType::neys:
    {
      MSP::CCS::NeysParameters neysParameters = *coordinateSystemState[direction].parameters.neysParameters;

      fprintf(output_file, "%f\t\t%f\t%f\t%f\t%f\t\t\t\t\t", neysParameters.standardParallel1() * _180_OVER_PI,
                                                             neysParameters.centralMeridian() * _180_OVER_PI,
                                                             neysParameters.originLatitude() * _180_OVER_PI,
                                                             neysParameters.falseEasting(),
                                                             neysParameters.falseNorthing() );
      break;
    }
    case CoordinateType::lambertConformalConic1Parallel:
    case CoordinateType::transverseCylindricalEqualArea:
    case CoordinateType::transverseMercator:
    {
      MSP::CCS::MapProjection5Parameters mapProjection5Parameters = *coordinateSystemState[direction].parameters.mapProjection5Parameters;

      fprintf(output_file, "%f\t\t%f\t%f\t%f\t%f\t\t\t\t\t", mapProjection5Parameters.scaleFactor(),
                                                             mapProjection5Parameters.centralMeridian() * _180_OVER_PI,
                                                             mapProjection5Parameters.originLatitude() * _180_OVER_PI,
                                                             mapProjection5Parameters.falseEasting(),
                                                             mapProjection5Parameters.falseNorthing());
      break;
    }
    case CoordinateType::obliqueMercator:
    {
      MSP::CCS::ObliqueMercatorParameters obliqueMercatorParameters = *coordinateSystemState[direction].parameters.obliqueMercatorParameters;

      fprintf(output_file, "%f\t\t\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t", obliqueMercatorParameters.scaleFactor(),
                                                                   obliqueMercatorParameters.originLatitude() * _180_OVER_PI,
                                                                   obliqueMercatorParameters.falseEasting(),
                                                                   obliqueMercatorParameters.falseNorthing(),
                                                                   obliqueMercatorParameters.longitude1() * _180_OVER_PI,
                                                                   obliqueMercatorParameters.latitude1()* _180_OVER_PI,
                                                                   obliqueMercatorParameters.longitude2() * _180_OVER_PI,
                                                                   obliqueMercatorParameters.latitude2() * _180_OVER_PI);
      break;
    }
    case CoordinateType::azimuthalEquidistant:
    case CoordinateType::bonne:
    case CoordinateType::cassini:
    case CoordinateType::cylindricalEqualArea:
    case CoordinateType::gnomonic:
    case CoordinateType::orthographic:
    case CoordinateType::polyconic:
    case CoordinateType::stereographic:
    {
      MSP::CCS::MapProjection4Parameters mapProjection4Parameters = *coordinateSystemState[direction].parameters.mapProjection4Parameters;

      fprintf(output_file, "\t\t%f\t%f\t%f\t%f\t\t\t\t\t", mapProjection4Parameters.centralMeridian() * _180_OVER_PI,
                                                           mapProjection4Parameters.originLatitude() * _180_OVER_PI,
                                                           mapProjection4Parameters.falseEasting(),
                                                           mapProjection4Parameters.falseNorthing());
      break;
    }
    case CoordinateType::equidistantCylindrical:
    {
      MSP::CCS::EquidistantCylindricalParameters equidistantCylindricalParameters = *coordinateSystemState[direction].parameters.equidistantCylindricalParameters;

      fprintf(output_file, "\t\t%f\t%f\t%f\t%f\t\t\t\t\t", equidistantCylindricalParameters.centralMeridian() * _180_OVER_PI,
                                                           equidistantCylindricalParameters.standardParallel() * _180_OVER_PI,
                                                           equidistantCylindricalParameters.falseEasting(),
                                                           equidistantCylindricalParameters.falseNorthing());
      break;
    }
    case CoordinateType::mercatorStandardParallel:
    {
      MSP::CCS::MercatorStandardParallelParameters mercatorStandardParallelParameters = *coordinateSystemState[direction].parameters.mercatorStandardParallelParameters;

      fprintf(output_file, "\t\t%f\t%f\t%f\t%f\t\t\t\t\t", mercatorStandardParallelParameters.centralMeridian() * _180_OVER_PI,
                                                           mercatorStandardParallelParameters.standardParallel() * _180_OVER_PI,
                                                           mercatorStandardParallelParameters.falseEasting(),
                                                           mercatorStandardParallelParameters.falseNorthing());
      break;
    }
    case CoordinateType::mercatorScaleFactor:
    {
      MSP::CCS::MercatorScaleFactorParameters mercatorScaleFactorParameters = *coordinateSystemState[direction].parameters.mercatorScaleFactorParameters;

      fprintf(output_file, "%f\t\t%f\t\t%f\t%f\t\t\t\t\t", mercatorScaleFactorParameters.scaleFactor(),
                                                           mercatorScaleFactorParameters.centralMeridian() * _180_OVER_PI,
                                                           mercatorScaleFactorParameters.falseEasting(),
                                                           mercatorScaleFactorParameters.falseNorthing());
      break;
    }
    case CoordinateType::polarStereographicStandardParallel:
    {
      MSP::CCS::PolarStereographicStandardParallelParameters polarStereographicStandardParallelParameters = *coordinateSystemState[direction].parameters.polarStereographicStandardParallelParameters;

      fprintf(output_file, "\t\t%f\t%f\t%f\t%f\t\t\t\t\t", polarStereographicStandardParallelParameters.centralMeridian() * _180_OVER_PI,
                                                           polarStereographicStandardParallelParameters.standardParallel() * _180_OVER_PI,
                                                           polarStereographicStandardParallelParameters.falseEasting(),
                                                           polarStereographicStandardParallelParameters.falseNorthing());
      break;
    }
    case CoordinateType::polarStereographicScaleFactor:
    {
      MSP::CCS::PolarStereographicScaleFactorParameters polarStereographicScaleFactorParameters = *coordinateSystemState[direction].parameters.polarStereographicScaleFactorParameters;

      fprintf(output_file, "%f\t\t%f\t\t%f\t%f\t\t\t\t\t", polarStereographicScaleFactorParameters.scaleFactor(),
                                                           polarStereographicScaleFactorParameters.centralMeridian() * _180_OVER_PI,
                                                           polarStereographicScaleFactorParameters.falseEasting(),
                                                           polarStereographicScaleFactorParameters.falseNorthing());
      break;
    }
    case CoordinateType::localCartesian:
    {
      MSP::CCS::LocalCartesianParameters localCartesianParameters = *coordinateSystemState[direction].parameters.localCartesianParameters;

      fprintf(output_file, "\t\t%f\t%f\t%f\t%f\t\t\t\t\t", localCartesianParameters.longitude() * _180_OVER_PI,
                                                           localCartesianParameters.latitude() * _180_OVER_PI,
                                                           localCartesianParameters.height(),
                                                           localCartesianParameters.orientation() * _180_OVER_PI);
      break;
    }
    case CoordinateType::eckert4:
    case CoordinateType::eckert6:
    case CoordinateType::millerCylindrical:
    case CoordinateType::mollweide:
    case CoordinateType::sinusoidal:
    case CoordinateType::vanDerGrinten:
    {
      MSP::CCS::MapProjection3Parameters mapProjection3Parameters = *coordinateSystemState[direction].parameters.mapProjection3Parameters;

      fprintf(output_file, "\t\t%f\t\t%f\t%f\t\t\t\t\t", mapProjection3Parameters.centralMeridian() * _180_OVER_PI,
                                                         mapProjection3Parameters.falseEasting(),
                                                         mapProjection3Parameters.falseNorthing());
      break;
    }
    case CoordinateType::britishNationalGrid:
    case CoordinateType::universalPolarStereographic:
    case CoordinateType::universalTransverseMercator:
    case CoordinateType::militaryGridReferenceSystem:
    case CoordinateType::usNationalGrid:
    case CoordinateType::newZealandMapGrid:
    case CoordinateType::georef:
    case CoordinateType::geocentric:
    case CoordinateType::geodetic:
    {
      fprintf(output_file, "\t\t\t\t\t\t\t\t\t\t"); 
      break;
    }
    default:
    {
      break;
    }
  }
}




// CLASSIFICATION: UNCLASSIFIED

