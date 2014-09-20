// CLASSIFICATION: UNCLASSIFIED

/***************************************************************************/
/* RSC IDENTIFIER: FIOMETHS
 *
 * ABSTRACT
 *
 *    This component provides file processing capability to MSPCCS.
 *
 *    This component depends on the following modules:
 *    Coordinate Conversion Service, ERRHAND, COMPHACK, FTOVAL, STRTOVAL.
 *
 * ERROR HANDLING
 *
 *    This component checks for errors in the file input data and if
 *    an error is found it passes an error code back to the calling routine.
 *
 *
 * REUSE NOTES
 *
 *    FIOMETHS is not specifically intended for reuse, as it is designed
 *    to work with files of a very specific format.
 *
 *
 * REFERENCES
 *
 *    Further information on FIOMETHS can be found in the MSPCCS Programmer's
 *    Guide.
 *
 *    FIOMETHS originated from :  U.S. Army Topographic Engineering Center
 *                             Digital Concepts & Analysis Center
 *                             7701 Telegraph Road
 *                             Alexandria, VA  22310-3864
 *
 * LICENSES
 *
 *    None apply to this component.
 *
 * RESTRICTIONS
 *
 *    FIOMETHS has no restrictions.
 *
 * ENVIRONMENT
 *
 *    FIOMETHS was tested and certified in the following environments:
 *
 *    1. Solaris 2.5 with GCC, version 2.7.2.1
 *    2. Windows XP with MS Visual C++, version 6
 *
 * MODIFICATIONS
 *
 *    Date              Description
 *    ----              -----------
 *    04-22-97          Original Code
 *    06-28-99          Added new DT&CC Modules
 *    09-13-00          Added new DT&CC Modules
 *    03-29-01          Improved file formatting flexibility
 *    08-17-05          Changed Lambert_Conformal_Conic to 
                        CoordinateType::lambertConformalConic2Parallels
 *    01-18-06          Added changes for new height types
 *    04-18-07          Updated to use C++ MSPCCS
 *    05-12-10          S. Gillis, BAEts26542, MSP TS MSL-HAE conversion
 *                      should use CCS
 *    10-25-10          S. Gillis, BAEts27155, Fix sample point Long/Lat order
 *                      for coordinate file header
 *    01-10-11          J. Chelos BAEts26267 Added EGM2008
 *    02-14-11          S. Gillis, BAEts26267, Fixed EGM2008
 *    03-29-11          S. Gillis, BAEts28564, Fixed Windows memory crash
 *    06-09-11          K. Lam, BAEts28972, Fixed the timer for file processing
 *    11-30-11          K.Lam, BAEts29174, Set output format for file operation
 */


/***************************************************************************/

/*                              INCLUDES                                 */
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include "fiomeths.h"
#include "CoordinateConversionService.h"
#include "DatumLibrary.h"
#include "EllipsoidLibrary.h"
#include "CoordinateSystemParameters.h"
#include "EquidistantCylindricalParameters.h"
#include "GeodeticParameters.h"
#include "LocalCartesianParameters.h"
#include "MapProjection3Parameters.h"
#include "MapProjection4Parameters.h"
#include "MapProjection5Parameters.h"
#include "MapProjection6Parameters.h"
#include "MercatorStandardParallelParameters.h"
#include "MercatorScaleFactorParameters.h"
#include "NeysParameters.h"
#include "ObliqueMercatorParameters.h"
#include "PolarStereographicStandardParallelParameters.h"
#include "PolarStereographicScaleFactorParameters.h"
#include "UTMParameters.h"
#include "BNGCoordinates.h"
#include "CartesianCoordinates.h"
#include "GARSCoordinates.h"
#include "GeodeticCoordinates.h"
#include "GEOREFCoordinates.h"
#include "MapProjectionCoordinates.h"
#include "MGRSorUSNGCoordinates.h"
#include "UPSCoordinates.h"
#include "UTMCoordinates.h"
#include "HeightType.h"
#include "strtoval.h"
#include "Accuracy.h"
#include "CoordinateConversionException.h"

#include <iostream>


#define PI    3.14159265358979323e0  /* PI        */
#define PI_Over_180  PI / 180.0  /* PI        */

using namespace MSP::CCS;

#define FILE_OPEN_ERROR        "Input file: error opening file\n"
#define FILE_CREATE_ERROR      "Output file: error creating file\n"
//#define FILE_HEADER_WRITE_ERROR "Output file: error writing header to file\n"
#define ERROR_PARSING_FILE     "Error reading input file coordinates\n"
#define INVALID_SOURCE_CS "Input file: Invalid source coordinate system\n"
#define INVALID_TARGET_CS "Output file: Invalid target coordinate system\n"
#define INVALID_TARGET_PARAMETERS       "Output file: Target parameters invalid for target coordinate system type\n"
#define INVALID_TARGET_COORDINATES      "Output file: Target coordinates invalid for target coordinate system type\n"

/* Local variable definitions */

typedef enum File_Header_Declarations
{
  FHD_End_of_Header = 0,        /* End of header */
  FHD_Header,                   /* Header */
  FHD_Comment,                  /* Separator/Comment */
  FHD_Projection,               /* Coordinate System, Grid, or Projection */
  FHD_Datum,                    /* Datum */
  FHD_No_Height,                /* No Height */
  FHD_Ellipsoid_Height,         /* Ellipsoid Height */
  FHD_MSL_EGM96_15M_BL_Height,  /* MSL EGM96 15M BL Height */
  FHD_MSL_EGM96_VG_NS_Height,   /* MSL EGM96 VG NS Height */
  FHD_MSL_EGM84_10D_BL_Height,  /* MSL EGM84 10D BL Height */
  FHD_MSL_EGM84_10D_NS_Height,  /* MSL EGM84 10D NS Height */
  FHD_MSL_EGM84_30M_BL_Height,  /* MSL EGM84 30M BL Height */
  FHD_MSL_EGM2008_TWOPOINTFIVEM_BCS_Height,/* MSL EGM2008 Bicubic Spline Height */
  FHD_Central_Meridian,         /* Central Meridian */
  FHD_Origin_Latitude,          /* Origin Latitude */
  FHD_Origin_Longitude,         /* Origin Longitude */
  FHD_Latitude_of_True_Scale,   /* Latitude of True Scale */
  FHD_Longitude_Down_from_Pole, /* Longitude Down from Pole */
  FHD_Latitude_One,             /* Latitude One */
  FHD_Longitude_One,            /* Longitude One */
  FHD_Latitude_Two,             /* Latitude Two */
  FHD_Longitude_Two,            /* Longitude Two */
  FHD_Origin_Height,            /* Origin Height */
  FHD_Orientation,              /* Orientation */
  FHD_Standard_Parallel,        /* Standard Parallel  */
  FHD_Standard_Parallel_One,    /* Standard Parallel One */
  FHD_Standard_Parallel_Two,    /* Standard Parallel Two */
  FHD_Hemisphere,               /* Hemisphere: 'N' or 'S'  */
  FHD_False_Easting,            /* False Easting */
  FHD_False_Northing,           /* False Northing */
  FHD_Scale_Factor,             /* Scale Factor */
  FHD_Coordinate_Order,         /* Order of Geodetic coordinates: LATITUDE-LONGITUDE or LONGITUDE-LATITUDE */
  FHD_Invalid_Header
} FHD_Value;


/* File processing error conditions */
typedef enum File_Input_and_Output_Conversion_Status
{
  /* Success */
  FIO_Success = 0,
  FIO_Error_Reading_File = -11,
  FIO_Error_Writing_File = -12,
  FIO_Error_Parsing_Header = -13,
  FIO_Error_Parsing_String = -14,
  FIO_Error_Parsing_File = -15,
  FIO_Error_Writing_Header = -16,
  FIO_EOFile = -17,
  FIO_Error_Latitude_of_True_Scale = -18,
  FIO_Error_Longitude_Down_from_Pole = -19,
  FIO_Error_Projection = -20,
  FIO_Error_Datum = -21,
  FIO_Error_Central_Meridian = -22,
  FIO_Error_Origin_Latitude = -23,
  FIO_Error_Origin_Longitude = -24,
  FIO_Error_Origin_Height = -25,
  FIO_Error_Orientation = -26,
  FIO_Error_Latitude_One = -27,
  FIO_Error_Longitude_One = -28,
  FIO_Error_Latitude_Two = -29,
  FIO_Error_Longitude_Two = -30,
  FIO_Error_Standard_Parallel = -31,
  FIO_Error_Standard_Parallel_One = -32,
  FIO_Error_Standard_Parallel_Two = -33,
  FIO_Error_False_Easting = -34,
  FIO_Error_False_Northing = -35,
  FIO_Error_Scale_Factor = -36,
  FIO_Error_Label = -37,
  FIO_Error_Hemisphere = -38,
  FIO_Error_Coordinate_Order = -39,
  /* General error */
  FIO_Error = -1000
} FIO_Status;

typedef enum File_to_Value_Conversion_Status
{
  /* Success */
  FVC_Success = 0,
  /* File_to_Longitude errors */
  FVC_Degrees_out_of_Bounds = -11,
  FVC_Minutes_out_of_Bounds = -12,
  FVC_Seconds_out_of_Bounds = -13,
  FVC_Minutes_or_Seconds_Overflow = -14,
  FVC_Inappropriate_Hemisphere = -15,
  FVC_Sign_and_Hemisphere = -16,
  FVC_Misplaced_Hemisphere = -17,
  FVC_File_Read_Longitude = -18,
  FVC_File_Read_Latitude = -19,
  FVC_File_Read_Hemisphere = -20,
  /* <Degrees>_to_File errors */
  FVC_DMS_to_File = -21,
  FVC_DM_to_File = -22,
  FVC_D_to_File = -23,
  FVC_File_Write_Longitude = -24,
  FVC_File_Write_Latitude = -25,
  /* <Parameter>_to_File errors */
  /* Parameter Specific errors */
  FVC_Zone_out_of_Bounds = -71,
  FVC_Scale_Factor_out_of_Bounds = -72,
  /* File errors */
  FVC_Error_Opening_File = -501,
  FVC_Error_Closing_File = -502,
  FVC_Error_Writing_String = -503,
  FVC_Error_Reading_Noise = -504,
  FVC_Error_Parsing_Header = -505,
  FVC_Invalid_Header = -506,
  FVC_Error_Parsing_File = -507,
  FVC_EOF = -508,
  /* General error */
  FVC_Error = -1000
} FVC_Status;

const char* LATITUDE_LONGITUDE = "LATITUDE-LONGITUDE";
const char* LONGITUDE_LATITUDE = "LONGITUDE-LATITUDE";

const char* coordinates_Header_String = "COORDINATES";
const char* projection_Header_String = "PROJECTION";
const char* datum_Header_String = "DATUM";
const char* no_Height_Header_String = "NO HEIGHT";
const char* ellipsoid_Height_Header_String = "ELLIPSOID HEIGHT";
const char* geoid_Height_Header_String = "GEOID/MSL HEIGHT";
const char* msl_EGM96_15M_BL_Height_Header_String = "MSL-EGM96-15M-BL HEIGHT";
const char* msl_EGM96_VG_NS_Height_Header_String = "MSL-EGM96-VG-NS HEIGHT";
const char* msl_EGM84_10D_BL_Height_Header_String = "MSL-EGM84-10D-BL HEIGHT";
const char* msl_EGM84_10D_NS_Height_Header_String = "MSL-EGM84-10D-NS HEIGHT";
const char* msl_EGM84_30M_BL_Height_Header_String = "MSL-EGM84-30M-BL HEIGHT";
const char* msl_EGM2008_TWOPOINTFIVEM_BCS_Height_Header_String = "MSL-EGM2008-2.5M-BCS HEIGHT";
const char* central_Meridian_Header_String = "CENTRAL MERIDIAN";
const char* origin_Latitude_Header_String = "ORIGIN LATITUDE";
const char* origin_Longitude_Header_String = "ORIGIN LONGITUDE";
const char* lat_Of_True_Scale_Header_String = "LATITUDE OF TRUE SCALE";
const char* polarst_Longitude_Header_String = "LONGITUDE DOWN FROM POLE";
const char* latitude_One_Header_String = "LATITUDE ONE";
const char* longitude_One_Header_String = "LONGITUDE ONE";
const char* latitude_Two_Header_String = "LATITUDE TWO";
const char* longitude_Two_Header_String = "LONGITUDE TWO";
const char* origin_Height_Header_String = "ORIGIN HEIGHT";
const char* orientation_Header_String = "ORIENTATION";
const char* standard_Parallel_Header_String = "STANDARD PARALLEL";
const char* standard_Parallel_One_Header_String = "STANDARD PARALLEL ONE";
const char* standard_Parallel_Two_Header_String = "STANDARD PARALLEL TWO";
const char* scale_Factor_Header_String = "SCALE FACTOR";
const char* hemisphere_Header_String = "HEMISPHERE";
const char* false_Easting_Header_String = "FALSE EASTING";
const char* false_Northing_Header_String = "FALSE NORTHING";
const char* coordinate_Order_Header_String = "COORDINATE ORDER";
const char* header_String = "HEADER";
const char* end_Of_Header_String = "END OF HEADER";
const char* ce90_String = "CE90";
const char* le90_String = "LE90";
const char* se90_String = "SE90";



/************************************************************************/
/*                              LOCAL FUNCTIONS     
 *
 */

FVC_Status Open_File(SourceOrTarget::Enum IOValue, const char *filename, FILE **file)
{ /* Open_File */
  FVC_Status error_Code = FVC_Success;
  char control_String[2];
  control_String[1] = '\0';
  control_String[0] = 'r';
  FILE *local_File = NULL;

  if (IOValue == SourceOrTarget::source)
    control_String[0] = 'r';
  else if (IOValue == SourceOrTarget::target)
    control_String[0] = 'w';

  local_File = fopen(filename, control_String);

  if (local_File)
    *file = local_File;
  else
  {
    error_Code = FVC_Error_Opening_File;
    *file = NULL;
  }
  return error_Code;
} /* Open_File */


FVC_Status Close_File(FILE **file)
{ /* Close_File  */
  FVC_Status error_Code = FVC_Success;
  if (*file != NULL)
  {
    fclose(*file);
    *file = NULL;
  }
  return error_Code;
} /* Close_File */


FVC_Status Eat_Noise(FILE *file)
{ /* Eat_Noise */
  char noise[256] = "";
  if ((fscanf(file, "%255[:, \n\r\t]", noise) == 0) && feof(file))
    return FVC_EOF;
  return FVC_Success;
} /* Eat_Noise */


FVC_Status Next_Header_Line(FILE *file, FHD_Value *header)
{
  FVC_Status error_Code = FVC_Success;
  FHD_Value header_Line = FHD_Comment;
  int next_Character = '\0';
  char garbage_Array[512];
  while (header_Line == FHD_Comment && !error_Code)
  {
    /* eat all whitespace */
    fscanf(file, " ");
    next_Character = fgetc(file);
    if (next_Character == '#')
      fgets(garbage_Array, 511, file);
    else
    {
      if (ungetc(next_Character, file) != next_Character)
        error_Code = FVC_Error_Parsing_Header;
      else
      {
        int i, length;
        char header_Value[32] = "";
        long ftell_Location = ftell(file);
        fscanf(file, "%31[^:\n]", header_Value);
        length = strlen(header_Value);
        for (i = 0; i < length; i++)
          header_Value[i] = (char)toupper(header_Value[i]);

        if (strstr(header_Value, projection_Header_String) ||
            strstr(header_Value, coordinates_Header_String))
          header_Line = FHD_Projection;
        else if (strstr(header_Value, datum_Header_String))
          header_Line = FHD_Datum;
        else if (strstr(header_Value, no_Height_Header_String))
          header_Line = FHD_No_Height;
        else if (strstr(header_Value, ellipsoid_Height_Header_String))
          header_Line = FHD_Ellipsoid_Height;
        else if (strstr(header_Value, geoid_Height_Header_String))
          header_Line = FHD_MSL_EGM96_15M_BL_Height;
        else if (strstr(header_Value, msl_EGM96_15M_BL_Height_Header_String))
          header_Line = FHD_MSL_EGM96_15M_BL_Height;
        else if (strstr(header_Value, msl_EGM96_VG_NS_Height_Header_String))
          header_Line = FHD_MSL_EGM96_VG_NS_Height;
        else if (strstr(header_Value, msl_EGM84_10D_BL_Height_Header_String))
          header_Line = FHD_MSL_EGM84_10D_BL_Height;
        else if (strstr(header_Value, msl_EGM84_10D_NS_Height_Header_String))
          header_Line = FHD_MSL_EGM84_10D_NS_Height;
        else if (strstr(header_Value, msl_EGM84_30M_BL_Height_Header_String))
          header_Line = FHD_MSL_EGM84_30M_BL_Height;
        else if (strstr(header_Value, msl_EGM2008_TWOPOINTFIVEM_BCS_Height_Header_String))
          header_Line = FHD_MSL_EGM2008_TWOPOINTFIVEM_BCS_Height;
        else if (strstr(header_Value, central_Meridian_Header_String))
          header_Line = FHD_Central_Meridian;
        else if (strstr(header_Value, lat_Of_True_Scale_Header_String))
          header_Line = FHD_Latitude_of_True_Scale;
        else if (strstr(header_Value, polarst_Longitude_Header_String))
          header_Line = FHD_Longitude_Down_from_Pole;
        else if (strstr(header_Value, origin_Latitude_Header_String))
          header_Line = FHD_Origin_Latitude;
        else if (strstr(header_Value, origin_Longitude_Header_String))
          header_Line = FHD_Origin_Longitude;
        else if (strstr(header_Value, origin_Height_Header_String))
          header_Line = FHD_Origin_Height;
        else if (strstr(header_Value, orientation_Header_String))
          header_Line = FHD_Orientation;
        else if (strstr(header_Value, latitude_One_Header_String))
          header_Line = FHD_Latitude_One;
        else if (strstr(header_Value, longitude_One_Header_String))
          header_Line = FHD_Longitude_One;
        else if (strstr(header_Value, latitude_Two_Header_String))
          header_Line = FHD_Latitude_Two;
        else if (strstr(header_Value, longitude_Two_Header_String))
          header_Line = FHD_Longitude_Two;
        else if (strstr(header_Value, standard_Parallel_One_Header_String))
          header_Line = FHD_Standard_Parallel_One;
        else if (strstr(header_Value, standard_Parallel_Two_Header_String))
          header_Line = FHD_Standard_Parallel_Two;
        else if (strstr(header_Value, standard_Parallel_Header_String))
          header_Line = FHD_Standard_Parallel;
        else if (strstr(header_Value, scale_Factor_Header_String))
          header_Line = FHD_Scale_Factor;
        else if (strstr(header_Value, hemisphere_Header_String))
          header_Line = FHD_Hemisphere;
        else if (strstr(header_Value, false_Easting_Header_String))
          header_Line = FHD_False_Easting;
        else if (strstr(header_Value, false_Northing_Header_String))
          header_Line = FHD_False_Northing;
        else if (strstr(header_Value, coordinate_Order_Header_String))
          header_Line = FHD_Coordinate_Order;
        else if (strstr(header_Value, end_Of_Header_String))
          header_Line = FHD_End_of_Header;
        else if (strstr(header_Value, header_String))
          header_Line = FHD_Header;
        else
        {
          error_Code = FVC_Invalid_Header;
          header_Line = FHD_Invalid_Header;
          fseek(file, ftell_Location, SEEK_SET);
        }
        Eat_Noise(file);
      }
    }
  }
  *header = header_Line;
  return error_Code;
}


FVC_Status Output_String(FILE *file, const char *sequence)
{/* BEGIN Output_String */
  FVC_Status error_Code = FVC_Success;
  if (fprintf(file, "%s", sequence) <= 0)
    error_Code = FVC_Error_Writing_String;
  return error_Code;
}/* END Output_String */


FVC_Status Output_Newline(FILE *file)
{
  static const char *newline_Sequence = "\n";
  return Output_String(file, newline_Sequence);
}


char* remove_trailing_spaces( char* str )
{
  int length = strlen( str );
  int index = length - 1;

  while( str[index] == ' ' )
    index --;

  str[index + 1] = '\0';

  return str;
}



/************************************************************************/
/*                             PUBLIC FUNCTIONS     
 *
 */

Fiomeths::Fiomeths() :
  coordinateConversionService( 0 ),
  inputFile( 0 ),
  outputFile( 0 ),
  sourceProjectionType( CoordinateType::geodetic ),
  targetProjectionType( CoordinateType::geodetic ),
  sourceDatumCode( 0 ),
  targetDatumCode( 0 ),
  coordinateSystemParameters( 0 ),
  equidistantCylindricalParameters( 0 ),
  geodeticParameters( 0 ),
  localCartesianParameters( 0 ),
  mapProjection3Parameters( 0 ),
  mapProjection4Parameters( 0 ),
  mapProjection5Parameters( 0 ),
  mapProjection6Parameters( 0 ),
  mercatorStandardParallelParameters( 0 ),
  mercatorScaleFactorParameters( 0 ),
  neysParameters( 0 ),
  obliqueMercatorParameters( 0 ),
  polarStereographicStandardParallelParameters( 0 ),
  polarStereographicScaleFactorParameters( 0 ),
  utmParameters( 0 ),
  targetParameters( 0 ),
  _numErrors( 0 ),
  _numProcessed( 0 ),
  _numWarnings( 0 ),
  _elapsedTime( 0 ),
  invalid( -1 ),
  inputLatitudeLongitudeOrder( true ),
  outputLatitudeLongitudeOrder( true )
{
  sourceDatumCode = new char[4];
  strcpy( sourceDatumCode, "WGE" );

  geodeticParameters = new GeodeticParameters();
}


Fiomeths::Fiomeths( const char* fileName ) :
  coordinateConversionService( 0 ),
  inputFile( 0 ),
  outputFile( 0 ),
  sourceProjectionType( CoordinateType::geodetic ),
  targetProjectionType( CoordinateType::geodetic ),
  sourceDatumCode( 0 ),
  targetDatumCode( 0 ),
  coordinateSystemParameters( 0 ),
  equidistantCylindricalParameters( 0 ),
  geodeticParameters( 0 ),
  localCartesianParameters( 0 ),
  mapProjection3Parameters( 0 ),
  mapProjection4Parameters( 0 ),
  mapProjection5Parameters( 0 ),
  mapProjection6Parameters( 0 ),
  mercatorStandardParallelParameters( 0 ),
  mercatorScaleFactorParameters( 0 ),
  neysParameters( 0 ),
  obliqueMercatorParameters( 0 ),
  polarStereographicStandardParallelParameters( 0 ),
  polarStereographicScaleFactorParameters( 0 ),
  utmParameters( 0 ),
  targetParameters( 0 ),
  _numErrors( 0 ),
  _numProcessed( 0 ),
  _numWarnings( 0 ),
  _elapsedTime( 0 ),
  invalid( -1 ),
  inputLatitudeLongitudeOrder( true ),
  outputLatitudeLongitudeOrder( true )
{
  try
  {
    setInputFilename( fileName );
  }
  catch( CoordinateConversionException e )
  {
    throw CoordinateConversionException( e.getMessage() );
  }
}


Fiomeths::~Fiomeths()
{
  delete coordinateConversionService;
  coordinateConversionService = 0;

  if( sourceDatumCode )
  {
    delete [] sourceDatumCode;
    sourceDatumCode = 0;
  }

  if( targetDatumCode )
  {
    delete [] targetDatumCode;
    targetDatumCode = 0;
  }

  if( coordinateSystemParameters )
  {
    delete coordinateSystemParameters;
    coordinateSystemParameters = 0;
  }

  if( equidistantCylindricalParameters )
  {
    delete equidistantCylindricalParameters;
    equidistantCylindricalParameters = 0;
  }

  if( geodeticParameters )
  {
    delete geodeticParameters;
    geodeticParameters = 0;
  }

  if( localCartesianParameters )
  {
    delete localCartesianParameters;
    localCartesianParameters = 0;
  }

  if( mapProjection3Parameters )
  {
    delete mapProjection3Parameters;
    mapProjection3Parameters = 0;
  }

  if( mapProjection4Parameters )
  {
    delete mapProjection4Parameters;
    mapProjection4Parameters = 0;
  }

  if( mapProjection5Parameters )
  {
    delete mapProjection5Parameters;
    mapProjection5Parameters = 0;
  }

  if( mapProjection6Parameters )
  {
    delete mapProjection6Parameters;
    mapProjection6Parameters = 0;
  }

  if( mercatorStandardParallelParameters )
  {
    delete mercatorStandardParallelParameters;
    mercatorStandardParallelParameters = 0;
  }

  if( mercatorScaleFactorParameters )
  {
    delete mercatorScaleFactorParameters;
    mercatorScaleFactorParameters = 0;
  }

  if( neysParameters )
  {
    delete neysParameters;
    neysParameters = 0;
  }

  if( obliqueMercatorParameters )
  {
    delete obliqueMercatorParameters;
    obliqueMercatorParameters = 0;
  }

  if( polarStereographicStandardParallelParameters )
  {
    delete polarStereographicStandardParallelParameters;
    polarStereographicStandardParallelParameters = 0;
  }

  if( polarStereographicScaleFactorParameters )
  {
    delete polarStereographicScaleFactorParameters;
    polarStereographicScaleFactorParameters = 0;
  }

  if( utmParameters )
  {
    delete utmParameters;
    utmParameters = 0;
  }


  delete targetParameters;
  targetParameters = 0;
}


void Fiomeths::setOutputFilename( const char *filename, const char* _targetDatumCode, CoordinateSystemParameters* _targetParameters )
{
  targetDatumCode = new char[ strlen( _targetDatumCode ) + 1 ];
  strcpy( targetDatumCode, _targetDatumCode );

  setCoordinateSystemParameters( _targetParameters );

  FIO_Status error_Code = FIO_Success;
  FVC_Status error_Code_File = FVC_Success;

  if (outputFile != NULL)
    closeOutputFile();

  error_Code_File = Open_File( SourceOrTarget::target, filename, &outputFile );

  if (!error_Code_File)
  {
    coordinateConversionService = new CoordinateConversionService( sourceDatumCode, getCoordinateSystemParameters(), _targetDatumCode, _targetParameters );

    writeOutputFileHeader( _targetDatumCode, _targetParameters );
  }
  else
    throw CoordinateConversionException( FILE_CREATE_ERROR );

}


void Fiomeths::convertFile()
{
  std::vector<MSP::CCS::CoordinateTuple*> sourceCoordinateCollection;
  std::vector<MSP::CCS::Accuracy*>        sourceAccuracyCollection;
  std::vector<TrailingHeight>             trailingHeightCollection;
  std::vector<CoordinateTuple*>           targetCoordinateCollection;
  std::vector<Accuracy*>                  targetAccuracyCollection;

  FIO_Status error_Code = FIO_Success;
  long tempErrorCode = 0;
  char next_Character;
  char Comment[256] = "";

  char next_Letters[7];
  long position;
  long coordinate = 1;

  long MAX_CONVERSIONS = 100000;

  static clock_t startTime, stopTime;
  startTime = clock();

  while (!feof(inputFile))
  {
    CoordinateTuple* sourceCoordinate = 0;
    Accuracy*        sourceAccuracy = 0;
    TrailingHeight trailingHeight;
    trailingHeight.heightPresent = false;
    Accuracy*        targetAccuracy = new Accuracy();

    next_Character = (char)fgetc(inputFile);
    ungetc(next_Character, inputFile);
    if (next_Character == '#')
    {
      fgets(Comment, 255, inputFile);
      sourceCoordinate = new CoordinateTuple( ( CoordinateType::Enum )invalid );
      sourceCoordinate->setErrorMessage( Comment );
      coordinate = 0;
    }
    else if (next_Character == '\n')
    {
      fgetc(inputFile);
      sourceCoordinate = new CoordinateTuple( ( CoordinateType::Enum )invalid );
      sourceCoordinate->setErrorMessage( "\n" );
      coordinate = 0;
    }
    else if (next_Character == '\r')
    {
      fgetc(inputFile);
      sourceCoordinate = new CoordinateTuple( ( CoordinateType::Enum )invalid );
      sourceCoordinate->setErrorMessage( "\r" );
      coordinate = 0;
    }
    else if (next_Character == 'H')
    {
      position = ftell(inputFile);
      fgets(next_Letters, 7, inputFile);
      if (strcmp(next_Letters, "HEADER") == 0)
      {
        convert(
           sourceCoordinateCollection,
           sourceAccuracyCollection,
           trailingHeightCollection,
           targetCoordinateCollection,
           targetAccuracyCollection );

        long errorCode = parseInputFileHeader(inputFile);
        if( errorCode )
        {
          char errorStr[256];
          getFileErrorString( errorCode, errorStr ); 
          closeInputFile();
          throw CoordinateConversionException( errorStr );
        }
        CoordinateConversionService* tempCoordinateConversionService = 
           new CoordinateConversionService(
              sourceDatumCode, 
              getCoordinateSystemParameters(),
              targetDatumCode,
              targetParameters );

        if( coordinateConversionService )
          delete coordinateConversionService;

        coordinateConversionService = tempCoordinateConversionService;
        sourceCoordinate = new CoordinateTuple( ( CoordinateType::Enum )invalid );
        coordinate = 0;
      }
      else
      {
        fseek(inputFile, position, 0);
        fputs(next_Letters, inputFile);
        coordinate = 1;
      }
    }
    else if (feof(inputFile))
    {
      sourceCoordinate = new CoordinateTuple( ( CoordinateType::Enum )invalid );
      coordinate = 0;
    }
    else
      coordinate = 1;

    if( coordinate )
    {
      // Read source coordinate values
      try
      {
        sourceCoordinate = readCoordinate();

        if( readHeight( trailingHeight.height ) == 0 )
          trailingHeight.heightPresent = true;

        // Read source accuracy values if given
        char buf[256] = "";
        fscanf(inputFile, "%[^\n#]", buf);
        if(strstr(buf, ce90_String) && strstr(buf, le90_String) && strstr(buf, se90_String))
        {
          Accuracy* tempSourceAccuracy = readConversionErrors(buf);
          sourceAccuracy = new Accuracy(
             tempSourceAccuracy->circularError90(),
             tempSourceAccuracy->linearError90(),
             tempSourceAccuracy->sphericalError90() );
          delete tempSourceAccuracy;
          tempSourceAccuracy = 0;
        }
        else
          sourceAccuracy = new Accuracy();

        // Store any trailing comments on the coordinate line
        fgets(Comment, 255, inputFile);
        if(Comment[0] != '\n')
          sourceCoordinate->setErrorMessage( Comment );
      }
      catch(CoordinateConversionException e)
      {
        fgets(Comment, 255, inputFile);
        sourceCoordinate = new CoordinateTuple( ( CoordinateType::Enum )invalid );
        sourceCoordinate->setErrorMessage( "# Error reading coordinates from input file\n");
        _numErrors ++;
      }

      _numProcessed++;

    }

    CoordinateTuple* targetCoordinate = initTargetCoordinate();

    sourceCoordinateCollection.push_back( sourceCoordinate );
    sourceAccuracyCollection.push_back( sourceAccuracy );
    trailingHeightCollection.push_back( trailingHeight );
    targetCoordinateCollection.push_back( targetCoordinate );
    targetAccuracyCollection.push_back( targetAccuracy );

    if( sourceCoordinateCollection.size() == MAX_CONVERSIONS )
    {
       convert(
          sourceCoordinateCollection,
          sourceAccuracyCollection,
          trailingHeightCollection,
          targetCoordinateCollection,
          targetAccuracyCollection );
    }
  }

  convert(
     sourceCoordinateCollection,
     sourceAccuracyCollection,
     trailingHeightCollection,
     targetCoordinateCollection,
     targetAccuracyCollection );

  stopTime = clock();

  _elapsedTime = ( double )( stopTime - startTime ) / CLOCKS_PER_SEC;

  closeInputFile();
  closeOutputFile();
}


void Fiomeths::closeInputFile()
{
  if (inputFile != NULL)
    Close_File(&inputFile);
}


void Fiomeths::closeOutputFile()
{
  if (outputFile != NULL)
    Close_File(&outputFile);
}


const char* Fiomeths::getDatumCode() const
{
/*
 *  The function getDatumCode returns the code of the current datum
 */

  return sourceDatumCode;
}


/**
 * Returns the input coordinate system parameters. 
 *
 * @return    coordinate system parameters contained in the header of the input file
 * @throws   CoordinateConversionException    invalid coordinate system
 */
CoordinateSystemParameters* Fiomeths::getCoordinateSystemParameters() const
{
  switch(sourceProjectionType)
  {
    case CoordinateType::albersEqualAreaConic:
    case CoordinateType::lambertConformalConic2Parallels:
      return mapProjection6Parameters;
    case CoordinateType::azimuthalEquidistant:
    case CoordinateType::bonne:
    case CoordinateType::cassini:
    case CoordinateType::cylindricalEqualArea:
    case CoordinateType::gnomonic:
    case CoordinateType::orthographic:
    case CoordinateType::polyconic:
    case CoordinateType::stereographic:
      return mapProjection4Parameters;
    case CoordinateType::britishNationalGrid:
    case CoordinateType::globalAreaReferenceSystem:
    case CoordinateType::geocentric:
    case CoordinateType::georef:
    case CoordinateType::militaryGridReferenceSystem:
    case CoordinateType::newZealandMapGrid:
    case CoordinateType::universalPolarStereographic:
    case CoordinateType::usNationalGrid:
     return coordinateSystemParameters;
    case CoordinateType::eckert4:
    case CoordinateType::eckert6:
    case CoordinateType::millerCylindrical:
    case CoordinateType::mollweide:
    case CoordinateType::sinusoidal:
    case CoordinateType::vanDerGrinten:
      return mapProjection3Parameters;
    case CoordinateType::equidistantCylindrical:
      return equidistantCylindricalParameters;
    case CoordinateType::geodetic:
      return geodeticParameters;
    case CoordinateType::localCartesian:
      return localCartesianParameters;
    case CoordinateType::mercatorStandardParallel:
      return mercatorStandardParallelParameters;
    case CoordinateType::mercatorScaleFactor:
      return mercatorScaleFactorParameters;
    case CoordinateType::lambertConformalConic1Parallel:
    case CoordinateType::transverseCylindricalEqualArea:
    case CoordinateType::transverseMercator:
      return mapProjection5Parameters;
    case CoordinateType::neys:
      return neysParameters;
    case CoordinateType::obliqueMercator:
      return obliqueMercatorParameters;
    case CoordinateType::polarStereographicStandardParallel:
      return polarStereographicStandardParallelParameters;
    case CoordinateType::polarStereographicScaleFactor:
      return polarStereographicScaleFactorParameters;
    case CoordinateType::universalTransverseMercator:
      return utmParameters;
    default:
      throw CoordinateConversionException( INVALID_SOURCE_CS );
  }
}


long Fiomeths::getNumErrors() const
{
  return _numErrors;
}


long Fiomeths::getNumProcessed() const
{
  return _numProcessed;
}


long Fiomeths::getNumWarnings() const
{
  return _numWarnings;
}


double Fiomeths::getElapsedTime() const
{
  return _elapsedTime;
}


void Fiomeths::setUseNSEW(bool value)
{
  _useNSEW = value;
}


/*boole Get_Use_NSEW()
{
  return (_useNSEW);
}*/

void Fiomeths::setUseMinutes(bool value)
{
  _useMinutes = value;
}


/*boole Get_Use_Minutes()
{
  return (_useMinutes);
}*/


void Fiomeths::setUseSeconds(bool value)
{
  _useSeconds = value;
}


/*boole Get_Use_Seconds()
{
  return (_useSeconds);
}*/


void Fiomeths::setLatLongPrecision(int _precision)
{
  Set_Lat_Long_Precision((Precision::Enum)_precision);
}

// BAEts29174
void Fiomeths::setSeparator(char value)
{
  Set_Separator(value);
}

void Fiomeths::setLonRange(int value)
{
  Set_Long_Range((Range)value);
}

void Fiomeths::showLeadingZeros(bool value)
{
  Show_Leading_Zeros( value);
}

void Fiomeths::setGeodeticCoordinateOrder(bool _outputLatitudeLongitudeOrder)
{
  outputLatitudeLongitudeOrder = _outputLatitudeLongitudeOrder;
}


/************************************************************************/
/*                              PRIVATE FUNCTIONS     
 *
 */

void Fiomeths::setInputFilename( const char *filename )
{
  FVC_Status error_Code_File = Open_File( SourceOrTarget::source, filename, &inputFile);
  if (error_Code_File)
    throw CoordinateConversionException( FILE_OPEN_ERROR );
  else
  {
    long error_Code = parseInputFileHeader(inputFile);
    if( error_Code )
    {
      char errorStr[256];
      getFileErrorString( error_Code, errorStr ); 
      throw CoordinateConversionException( errorStr );
    }
  }

  Eat_Noise(inputFile);
}


void Fiomeths::getFileErrorString(long error, char *str)
{
  switch (error)
  {
  case FIO_Error_Reading_File:
    {
      strcpy(str,"Input file error opening file\n");
      break;
    }
  case FIO_Error_Projection:
    {
      strcpy(str,"Input file error in coordinate type\n");
      break;
    }
  case FIO_Error_Datum:
    {
      strcpy(str,"Input file error in datum\n");
      break;
    }
  case FIO_Error_Central_Meridian:
    {
      strcpy(str,"Input file error in Central Meridian\n");
      break;
    }
  case FIO_Error_Origin_Latitude:
    {
      strcpy(str,"Input file error in Origin Latitude\n");
      break;
    }
  case FIO_Error_Origin_Longitude:
    {
      strcpy(str,"Input file error in Origin Longitude\n");
      break;
    }
  case FIO_Error_Origin_Height:
    {
      strcpy(str,"Input file error in Origin Height\n");
      break;
    }
  case FIO_Error_Orientation:
    {
      strcpy(str,"Input file error in Orientation\n");
      break;
    }
  case FIO_Error_Latitude_One:
    {
      strcpy(str,"Input file error in Latitude 1\n");
      break;
    }
  case FIO_Error_Longitude_One:
    {
      strcpy(str,"Input file error in Longitude 1\n");
      break;
    }
  case FIO_Error_Latitude_Two:
    {
      strcpy(str,"Input file error in Latitude 2\n");
      break;
    }
  case FIO_Error_Longitude_Two:
    {
      strcpy(str,"Input file error in Longitude 2\n");
      break;
    }
  case FIO_Error_Standard_Parallel:
    {
      strcpy(str,"Input file error in Standard Parallel ");
      break;
    }
  case FIO_Error_Standard_Parallel_One:
    {
      strcpy(str,"Input file error in Standard Parallel 1\n");
      break;
    }
  case FIO_Error_Standard_Parallel_Two:
    {
      strcpy(str,"Input file error in Standard Parallel 2\n");
      break;
    }
  case FIO_Error_False_Easting:
    {
      strcpy(str,"Input file error in False Easting\n");
      break;
    }
  case FIO_Error_False_Northing:
    {
      strcpy(str,"Input file error in False Northing\n");
      break;
    }
  case FIO_Error_Scale_Factor:
    {
      strcpy(str,"Input file error in Scale Factor\n");
      break;
    }
  case FIO_Error_Hemisphere:
    {
      strcpy(str,"Input file error in Hemisphere\n");
      break;
    }
  case FIO_Error_Coordinate_Order:
    {
      strcpy(str,"Input file error in Coordinate Order\n");
      break;
    }
  case FIO_Error_Parsing_Header:
    {
      strcpy(str,"Input file header contains unknown keyword\n");
      break;
    }
  default:
    {
      strcpy(str,"Input file unknown error\n");
      break;
    }
  }
}


long Fiomeths::parseInputFileHeader(FILE *file)
{
  HeightType::Enum heightType = HeightType::ellipsoidHeight;
  double centralMeridian = 0.0;
  double latitudeOfTrueScale = 0.0;
  double longitudeDownFromPole = 0.0;
  double originLatitude = 0.0;
  double originLongitude = 0.0;
  double originHeight = 0.0;
  double orientation = 0.0;
  double latitude1 = 0.0;
  double longitude1 = 0.0;
  double latitude2 = 0.0;
  double longitude2 = 0.0;
  double standardParallel = 0.0;
  double standardParallel1 = 0.0;
  double standardParallel2 = 0.0;
  double falseEasting = 0.0;
  double falseNorthing = 0.0;
  double scaleFactor = 1.0;
  char hemisphere = 'N';
  FHD_Value latitude_header_type = FHD_Origin_Latitude;
  FHD_Value longitude_header_type = FHD_Origin_Longitude;

  FIO_Status error_Code = FIO_Success;
  SVC_Status error_Code_String = SVC_Success;
  FVC_Status error_Code_File = FVC_Success;
  FHD_Value header_Line = FHD_Comment;
  long errorCodeCCS = 0;
  char buf[256];
  long length;

  inputLatitudeLongitudeOrder = true;

  /* Process each header line until end-of-header sequence reached */
  error_Code_File = Next_Header_Line(file, &header_Line);
  if (error_Code_File)
    return FIO_Error_Parsing_Header;
  while (header_Line != FHD_End_of_Header)
  {
    switch (header_Line)
    {
      case FHD_Projection:
      {
        char coord_Type[256] = "";
        if (fgets(coord_Type,256,file) == NULL)
          return FIO_Error_Projection;
        else
        {
          int i = 0;
          while(isalpha(coord_Type[i]) || (coord_Type[i] == ' ') || (coord_Type[i] == '\'')
                || (coord_Type[i] == '(') || (coord_Type[i] == ')')
                || (coord_Type[i] == '1') || (coord_Type[i] == '2'))
          {
            if (isalpha(coord_Type[i]))
              coord_Type[i] = (char)toupper(coord_Type[i]);
            i++;
          }
          coord_Type[i] = '\0';
          strcpy( coord_Type, remove_trailing_spaces( coord_Type ) );
          error_Code_String = String_to_Projection(coord_Type, &sourceProjectionType);
          if(error_Code_String)
            return FIO_Error_Projection;
        }
        break;
      }
      case FHD_Datum:
      {
        char datumCode[7];
        if (fscanf(file, "%s", datumCode) <=0)
          return FIO_Error_Datum;
        else
        {
          int length = strlen( datumCode );
          if ( length > ( DATUM_CODE_LENGTH-1 ) )
            return FIO_Error_Datum;
          else
          {
            if( sourceDatumCode )
              delete [] sourceDatumCode;
            sourceDatumCode = new char[ strlen( datumCode ) + 1 ];
            strcpy( sourceDatumCode, datumCode );
          }
        }
        break;
      }
      case FHD_No_Height:
      {
        heightType = HeightType::noHeight;
        break;
      }
      case FHD_Ellipsoid_Height:
      {
        heightType = HeightType::ellipsoidHeight;
        break;
      }
      case FHD_MSL_EGM96_15M_BL_Height:
      {
        heightType = HeightType::EGM96FifteenMinBilinear;
        break;
      }
      case FHD_MSL_EGM96_VG_NS_Height:
      {
        heightType = HeightType::EGM96VariableNaturalSpline;
        break;
      }
      case FHD_MSL_EGM84_10D_BL_Height:
      {
        heightType = HeightType::EGM84TenDegBilinear;
        break;
      }
      case FHD_MSL_EGM84_10D_NS_Height:
      {
        heightType = HeightType::EGM84TenDegNaturalSpline;
        break;
      }
      case FHD_MSL_EGM84_30M_BL_Height:
      {
        heightType = HeightType::EGM84ThirtyMinBiLinear;
        break;
      }
      case FHD_MSL_EGM2008_TWOPOINTFIVEM_BCS_Height:
      {
        heightType = HeightType::EGM2008TwoPtFiveMinBicubicSpline;
        break;
      }
      case FHD_Central_Meridian:
      {
        double value = 0.0;
        long error_code = 0;
        longitude_header_type = FHD_Central_Meridian;
        Eat_Noise(file);
        error_code = fscanf(file, "%14[ 0123456789NSEW:/+.-]", buf);
        if (error_code)
        {
          error_code = String_to_Longitude(buf,&value);
          if (!error_code)
          {
            centralMeridian = value * PI / 180.0;
          }
          else
            return FIO_Error_Central_Meridian;
        }
        else
          return FIO_Error_Central_Meridian;
        break;
      }
    case FHD_Latitude_of_True_Scale:
      {
        double value = 0.0;
        long error_code = 0;
        latitude_header_type = FHD_Latitude_of_True_Scale;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Latitude(buf,&value);
          if (!error_code)
          {
            latitudeOfTrueScale = value * PI / 180.0;
          }
          else
            return FIO_Error_Latitude_of_True_Scale;
        }
        else
          return FIO_Error_Latitude_of_True_Scale;
        break;
      }
    case FHD_Longitude_Down_from_Pole:
      {
        double value = 0.0;
        long error_code = 0;
        longitude_header_type = FHD_Longitude_Down_from_Pole;
        Eat_Noise(file);
        error_code = fscanf(file, "%14[ 0123456789NSEW:/+.-]", buf);
        if (error_code)
        {
          error_code = String_to_Longitude(buf,&value);
          if (!error_code)
          {
            longitudeDownFromPole = value * PI / 180.0;
          }
          else
            return FIO_Error_Longitude_Down_from_Pole;
        }
        else
          return FIO_Error_Longitude_Down_from_Pole;
        break;
      }
    case FHD_Origin_Latitude:
      {
        double value = 0.0;
        long error_code = 0;
        latitude_header_type = FHD_Origin_Latitude;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Latitude(buf,&value);
          if (!error_code)
          {
            originLatitude = value * PI / 180.0;
          }
          else
            return FIO_Error_Origin_Latitude;
        }
        else
          return FIO_Error_Origin_Latitude;
        break;
      }
    case FHD_Origin_Longitude:
      {
        double value = 0.0;
        long error_code = 0;
        longitude_header_type = FHD_Origin_Longitude;
        Eat_Noise(file);
        error_code = fscanf(file, "%14[ 0123456789NSEW:/+.-]", buf);
        if (error_code)
        {
          error_code = String_to_Longitude(buf,&value);
          if (!error_code)
          {
            originLongitude = value * PI / 180.0;
          }
          else
            return FIO_Error_Origin_Longitude;
        }
        else
          return FIO_Error_Origin_Longitude;
        break;
      }
    case FHD_Origin_Height:
      {
        double value = 0.0;
        long error_code = fscanf(file, "%s", buf);
        if (error_code)
        {
          length = strlen(buf)-1;
          if (!isdigit(buf[length]))
            buf[length] = 0;
          error_code = String_to_Double(buf,&value);
          if (!error_code)
            originHeight = value;
          else
            return FIO_Error_Origin_Height;
        }
        else
          return FIO_Error_Origin_Height;
        break;
      }
    case FHD_Orientation:
      {
        double value = 0.0;
        long error_code = 0;
        Eat_Noise(file);
        error_code = fscanf(file, "%14[ 0123456789NSEW:/+.-]", buf);
        if (error_code)
        {
          error_code = String_to_Longitude(buf,&value);
          if (!error_code)
          {
            orientation = value * PI / 180.0;
          }
          else
            return FIO_Error_Orientation;
        }
        else
          return FIO_Error_Orientation;
        break;
      }
    case FHD_Latitude_One:
      {
        double value = 0.0;
        long error_code = 0;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Latitude(buf,&value);
          if (!error_code)
          {
            latitude1 = value * PI / 180.0;
          }
          else
            return FIO_Error_Latitude_One;
        }
        else
          return FIO_Error_Latitude_One;
        break;
      }
    case FHD_Longitude_One:
      {
        double value = 0.0;
        long error_code = 0;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Longitude(buf,&value);
          if (!error_code)
          {
            longitude1 = value * PI / 180.0;
          }
          else
            return FIO_Error_Longitude_One;
        }
        else
          return FIO_Error_Longitude_One;
        break;
      }
    case FHD_Latitude_Two:
      {
        double value = 0.0;
        long error_code = 0;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Latitude(buf,&value);
          if (!error_code)
          {
            latitude2 = value * PI / 180.0;
          }
          else
            return FIO_Error_Latitude_Two;
        }
        else
          return FIO_Error_Latitude_Two;
        break;
      }
    case FHD_Longitude_Two:
      {
        double value = 0.0;
        long error_code = 0;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Longitude(buf,&value);
          if (!error_code)
          {
            longitude2 = value * PI / 180.0;
          }
          else
            return FIO_Error_Longitude_Two;
        }
        else
          return FIO_Error_Longitude_Two;
        break;
      }
    case FHD_Standard_Parallel:
      {
        double value = 0.0;
        long error_code = 0;
        latitude_header_type = FHD_Standard_Parallel;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Latitude(buf,&value);
          if (!error_code)
          {
            standardParallel = value * PI / 180.0;
          }
          else
            return FIO_Error_Standard_Parallel;
        }
        else
          return FIO_Error_Standard_Parallel;
        break;
      }
    case FHD_Standard_Parallel_One:
      {
        double value = 0.0;
        long error_code = 0;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Latitude(buf,&value);
          if (!error_code)
          {
            standardParallel1 = value * PI / 180.0;
          }
          else
            return FIO_Error_Standard_Parallel_One;
        }
        else
          return FIO_Error_Standard_Parallel_One;
        break;
      }
    case FHD_Standard_Parallel_Two:
      {
        double value = 0.0;
        long error_code = 0;
        Eat_Noise(file);
        error_code = fscanf(file, "%13[ 0123456789NSEW:/.+-]", buf);
        if (error_code)
        {
          error_code = String_to_Latitude(buf,&value);
          if (!error_code)
          {
            standardParallel2 = value * PI / 180.0;
          }
          else
            return FIO_Error_Standard_Parallel_Two;
        }
        else
          return FIO_Error_Standard_Parallel_Two;
        break;
      }
    case FHD_False_Easting:
      {
        double value = 0.0;
        long error_code = fscanf(file, "%s", buf);
        if (error_code)
        {
          length = strlen(buf)-1;
          if (!isdigit(buf[length]))
            buf[length] = 0;
          error_code = String_to_Double(buf,&value);
          if (!error_code)
            falseEasting = value;
          else
            return FIO_Error_False_Easting;
        }
        else
          return FIO_Error_False_Easting;
        break;
      }
      case FHD_False_Northing:
      {
        double value = 0.0;
        long error_code = fscanf(file, "%s", buf);
        if (error_code)
        {
          length = strlen(buf)-1;
          if (!isdigit(buf[length]))
            buf[length] = 0;
          error_code = String_to_Double(buf,&value);
          if (!error_code)
            falseNorthing = value;
          else
            return FIO_Error_False_Northing;
        }
        else
          return FIO_Error_False_Northing;
        break;
      }
      case FHD_Scale_Factor:
      {
        double value = 0.0;
        long error_code = fscanf(file, "%s", buf);
        if (error_code)
        {
          length = strlen(buf)-1;
          if (!isdigit(buf[length]))
            buf[length] = 0;
          error_code = String_to_Double(buf,&value);
          if (!error_code)
          {
            if ((0.3 <= value) &&(value <= 3.0))
              scaleFactor = value;
            else
              return FIO_Error_Scale_Factor;
          }
          else
            return FIO_Error_Scale_Factor;
        }
        else
          return FIO_Error_Scale_Factor;
        break;
      }
      case FHD_Coordinate_Order:
      {
        int i = 0;
        long error_code = fscanf(file, "%s", buf);
        if (error_code)
        {
          int length = strlen(buf);
          for(i = 0; i < length; i++)
            buf[i] = (char)toupper(buf[i]);

          if(strstr(LONGITUDE_LATITUDE, buf))
            inputLatitudeLongitudeOrder = false;
          else if(strstr(LATITUDE_LONGITUDE, buf))
            inputLatitudeLongitudeOrder = true;
          else
            return FIO_Error_Coordinate_Order;
        }
        else
          return FIO_Error_Coordinate_Order;
        break;
      }
      case FHD_Hemisphere:
      {
        int i = 0;
        long error_code = fscanf(file, "%s", buf);
        if (error_code)
        {
          int length = strlen(buf);
          for(i = 0; i < length; i++)
            buf[i] = (char)toupper(buf[i]);
          if(strstr("N", buf))
            hemisphere = 'N';
          else if(strstr("S", buf))
            hemisphere = 'S';
          else
            return FIO_Error_Hemisphere;
        }
        else
          return FIO_Error_Hemisphere;
        break;
      }
    case FHD_Header:
      break;
    case FHD_Invalid_Header:
    default:
      return FIO_Error_Parsing_Header;
    } /* switch */
    error_Code_File = Next_Header_Line(file, &header_Line);
    if (error_Code_File)
      return FIO_Error_Parsing_Header;
  } /* while */

  /* Set parameters */
  switch( sourceProjectionType )
  {
    case CoordinateType::albersEqualAreaConic:
      if( mapProjection6Parameters )
        delete mapProjection6Parameters;
      mapProjection6Parameters = new MapProjection6Parameters(CoordinateType::albersEqualAreaConic, centralMeridian, originLatitude, standardParallel1, standardParallel2, falseEasting, falseNorthing);
      break;
    case CoordinateType::azimuthalEquidistant:
      if( mapProjection4Parameters )
        delete mapProjection4Parameters;
      mapProjection4Parameters = new MapProjection4Parameters(CoordinateType::azimuthalEquidistant, centralMeridian, originLatitude, falseEasting, falseNorthing);
      break;
    case CoordinateType::bonne:
      if( mapProjection4Parameters )
        delete mapProjection4Parameters;
        mapProjection4Parameters = new MapProjection4Parameters(CoordinateType::bonne, centralMeridian, originLatitude, falseEasting, falseNorthing);
        break;
    case CoordinateType::britishNationalGrid:
      if( coordinateSystemParameters )
        delete coordinateSystemParameters;
      coordinateSystemParameters = new CoordinateSystemParameters(CoordinateType::britishNationalGrid);
      break;
    case CoordinateType::cassini:
      if( mapProjection4Parameters )
        delete mapProjection4Parameters;
      mapProjection4Parameters = new MapProjection4Parameters(CoordinateType::cassini, centralMeridian, originLatitude, falseEasting, falseNorthing);
      break;
    case CoordinateType::cylindricalEqualArea:
      if( mapProjection4Parameters )
        delete mapProjection4Parameters;
      mapProjection4Parameters = new MapProjection4Parameters(CoordinateType::cylindricalEqualArea, centralMeridian, originLatitude, falseEasting, falseNorthing);
      break;
    case CoordinateType::eckert4:
      if( mapProjection3Parameters )
        delete mapProjection3Parameters;
      mapProjection3Parameters = new MapProjection3Parameters(CoordinateType::eckert4, centralMeridian, falseEasting, falseNorthing);
      break;
    case CoordinateType::eckert6:
      if( mapProjection3Parameters )
        delete mapProjection3Parameters;
      mapProjection3Parameters = new MapProjection3Parameters(CoordinateType::eckert6, centralMeridian, falseEasting, falseNorthing);
      break;
    case CoordinateType::equidistantCylindrical:
      if( equidistantCylindricalParameters )
        delete equidistantCylindricalParameters;
      equidistantCylindricalParameters = new EquidistantCylindricalParameters(CoordinateType::equidistantCylindrical, centralMeridian, standardParallel, falseEasting, falseNorthing);
      break;
    case CoordinateType::geocentric:
      if( coordinateSystemParameters )
        delete coordinateSystemParameters;
      coordinateSystemParameters = new CoordinateSystemParameters(CoordinateType::geocentric);
      break;
    case CoordinateType::geodetic:
      if( geodeticParameters )
        delete geodeticParameters;
      geodeticParameters = new GeodeticParameters(CoordinateType::geodetic, heightType);
      break;
    case CoordinateType::georef:
      if( coordinateSystemParameters )
        delete coordinateSystemParameters;
      coordinateSystemParameters = new CoordinateSystemParameters(CoordinateType::georef);
      break;
    case CoordinateType::globalAreaReferenceSystem:
      if( coordinateSystemParameters )
        delete coordinateSystemParameters;
      coordinateSystemParameters = new CoordinateSystemParameters(CoordinateType::globalAreaReferenceSystem);
      break;
    case CoordinateType::gnomonic:
      if( mapProjection4Parameters )
        delete mapProjection4Parameters;
      mapProjection4Parameters = new MapProjection4Parameters(CoordinateType::gnomonic, centralMeridian, originLatitude, falseEasting, falseNorthing);
      break;
    case CoordinateType::lambertConformalConic1Parallel:
      if( mapProjection5Parameters )
        delete mapProjection5Parameters;
      mapProjection5Parameters = new MapProjection5Parameters(CoordinateType::lambertConformalConic1Parallel, centralMeridian, originLatitude, scaleFactor, falseEasting, falseNorthing);
      break;
    case CoordinateType::lambertConformalConic2Parallels:
      if( mapProjection6Parameters )
        delete mapProjection6Parameters;
      mapProjection6Parameters = new MapProjection6Parameters(CoordinateType::lambertConformalConic2Parallels, centralMeridian, originLatitude, standardParallel1, standardParallel2, falseEasting, falseNorthing);
      break;
    case CoordinateType::localCartesian:
      if( localCartesianParameters )
        delete localCartesianParameters;
      localCartesianParameters = new LocalCartesianParameters(CoordinateType::localCartesian, originLongitude, originLatitude, originHeight, orientation);
      break;
    case CoordinateType::mercatorStandardParallel:
    {
      if( mercatorStandardParallelParameters )
        delete mercatorStandardParallelParameters;
      if( latitude_header_type == FHD_Standard_Parallel )
        mercatorStandardParallelParameters = new MercatorStandardParallelParameters(CoordinateType::mercatorStandardParallel, centralMeridian, standardParallel, scaleFactor, falseEasting, falseNorthing);
      else if( latitude_header_type == FHD_Origin_Latitude )
        mercatorStandardParallelParameters = new MercatorStandardParallelParameters(CoordinateType::mercatorStandardParallel, centralMeridian, originLatitude, scaleFactor, falseEasting, falseNorthing);
      else
        mercatorStandardParallelParameters = new MercatorStandardParallelParameters(CoordinateType::mercatorStandardParallel, centralMeridian, latitudeOfTrueScale, scaleFactor, falseEasting, falseNorthing);
     
      GeodeticParameters geodeticParametersLoc( CoordinateType::geodetic, HeightType::noHeight );  
      CoordinateConversionService tempCoordinateConversionService( sourceDatumCode, mercatorStandardParallelParameters, "WGE", &geodeticParametersLoc );
      MercatorStandardParallelParameters tempMercatorStandardParallelParameters = *dynamic_cast< MercatorStandardParallelParameters* >( tempCoordinateConversionService.getCoordinateSystem( SourceOrTarget::source ) );
      
      mercatorStandardParallelParameters->setScaleFactor( tempMercatorStandardParallelParameters.scaleFactor() );
      break;
    }
    case CoordinateType::mercatorScaleFactor:
      if( mercatorScaleFactorParameters )
        delete mercatorScaleFactorParameters;
      mercatorScaleFactorParameters = new MercatorScaleFactorParameters(CoordinateType::mercatorScaleFactor, centralMeridian, scaleFactor, falseEasting, falseNorthing);
      break;
    case CoordinateType::militaryGridReferenceSystem:
      if( coordinateSystemParameters )
        delete coordinateSystemParameters;
      coordinateSystemParameters = new CoordinateSystemParameters(CoordinateType::militaryGridReferenceSystem);
      break;
    case CoordinateType::millerCylindrical:
      if( mapProjection3Parameters )
        delete mapProjection3Parameters;
      mapProjection3Parameters = new MapProjection3Parameters(CoordinateType::millerCylindrical, centralMeridian, falseEasting, falseNorthing);
      break;
    case CoordinateType::mollweide:
      if( mapProjection3Parameters )
        delete mapProjection3Parameters;
      mapProjection3Parameters = new MapProjection3Parameters(CoordinateType::mollweide, centralMeridian, falseEasting, falseNorthing);
      break;
    case CoordinateType::neys:
      if( neysParameters )
        delete neysParameters;
      neysParameters = new NeysParameters(CoordinateType::neys, centralMeridian, originLatitude, standardParallel1, falseEasting, falseNorthing);
      break;
    case CoordinateType::newZealandMapGrid:
      if( coordinateSystemParameters )
        delete coordinateSystemParameters;
      coordinateSystemParameters = new CoordinateSystemParameters(CoordinateType::newZealandMapGrid);
      break;
    case CoordinateType::obliqueMercator:
      if( obliqueMercatorParameters )
        delete obliqueMercatorParameters;
      obliqueMercatorParameters = new ObliqueMercatorParameters(CoordinateType::obliqueMercator, originLatitude, longitude1, latitude1, longitude2, latitude2, falseEasting, falseNorthing, scaleFactor);
      break;
    case CoordinateType::orthographic:
      if( mapProjection4Parameters )
        delete mapProjection4Parameters;
      mapProjection4Parameters = new MapProjection4Parameters(CoordinateType::orthographic, centralMeridian, originLatitude, falseEasting, falseNorthing);
      break;
    case CoordinateType::polarStereographicStandardParallel:
      if( polarStereographicStandardParallelParameters )
        delete polarStereographicStandardParallelParameters;
      if( latitude_header_type == FHD_Standard_Parallel )
      {
        if( longitude_header_type == FHD_Central_Meridian )
          polarStereographicStandardParallelParameters = new PolarStereographicStandardParallelParameters(CoordinateType::polarStereographicStandardParallel, centralMeridian, standardParallel, falseEasting, falseNorthing);
        else
          polarStereographicStandardParallelParameters = new PolarStereographicStandardParallelParameters(CoordinateType::polarStereographicStandardParallel, longitudeDownFromPole, standardParallel, falseEasting, falseNorthing);
      }
      else
      {
        if( longitude_header_type == FHD_Central_Meridian )
          polarStereographicStandardParallelParameters = new PolarStereographicStandardParallelParameters(CoordinateType::polarStereographicStandardParallel, centralMeridian, latitudeOfTrueScale, falseEasting, falseNorthing);
        else
          polarStereographicStandardParallelParameters = new PolarStereographicStandardParallelParameters(CoordinateType::polarStereographicStandardParallel, longitudeDownFromPole, latitudeOfTrueScale, falseEasting, falseNorthing);
      }
      break;
    case CoordinateType::polarStereographicScaleFactor:
      if( polarStereographicScaleFactorParameters )
        delete polarStereographicScaleFactorParameters;
      if( longitude_header_type == FHD_Central_Meridian )
        polarStereographicScaleFactorParameters = new PolarStereographicScaleFactorParameters(CoordinateType::polarStereographicScaleFactor, centralMeridian, scaleFactor, hemisphere, falseEasting, falseNorthing);
      else
        polarStereographicScaleFactorParameters = new PolarStereographicScaleFactorParameters(CoordinateType::polarStereographicScaleFactor, longitudeDownFromPole, scaleFactor, hemisphere, falseEasting, falseNorthing);
      break;
    case CoordinateType::polyconic:
      if( mapProjection4Parameters )
        delete mapProjection4Parameters;
      mapProjection4Parameters = new MapProjection4Parameters(CoordinateType::polyconic, centralMeridian, originLatitude, falseEasting, falseNorthing);
      break;
    case CoordinateType::sinusoidal:
      if( mapProjection3Parameters )
        delete mapProjection3Parameters;
      mapProjection3Parameters = new MapProjection3Parameters(CoordinateType::sinusoidal, centralMeridian, falseEasting, falseNorthing);
      break;
    case CoordinateType::stereographic:
      if( mapProjection4Parameters )
        delete mapProjection4Parameters;
      mapProjection4Parameters = new MapProjection4Parameters(CoordinateType::stereographic, centralMeridian, originLatitude, falseEasting, falseNorthing);
      break;
    case CoordinateType::transverseCylindricalEqualArea:
      if( mapProjection5Parameters )
        delete mapProjection5Parameters;
      mapProjection5Parameters = new MapProjection5Parameters(CoordinateType::transverseCylindricalEqualArea, centralMeridian, originLatitude, scaleFactor, falseEasting, falseNorthing);
      break;
    case CoordinateType::transverseMercator:
      if( mapProjection5Parameters )
        delete mapProjection5Parameters;
      mapProjection5Parameters = new MapProjection5Parameters(CoordinateType::transverseMercator, centralMeridian, originLatitude, scaleFactor, falseEasting, falseNorthing);
      break;
    case CoordinateType::universalPolarStereographic:
      if( coordinateSystemParameters )
        delete coordinateSystemParameters;
      coordinateSystemParameters = new CoordinateSystemParameters(CoordinateType::universalPolarStereographic);
      break;
    case CoordinateType::universalTransverseMercator:
      if( utmParameters )
        delete utmParameters;
      utmParameters = new UTMParameters(CoordinateType::universalTransverseMercator, 0);
      break;
    case CoordinateType::usNationalGrid:
      if( coordinateSystemParameters )
        delete coordinateSystemParameters;
      coordinateSystemParameters = new CoordinateSystemParameters(CoordinateType::usNationalGrid);
      break;
    case CoordinateType::vanDerGrinten:
      if( mapProjection3Parameters )
        delete mapProjection3Parameters;
      mapProjection3Parameters = new MapProjection3Parameters(CoordinateType::vanDerGrinten, centralMeridian, falseEasting, falseNorthing);
      break;
    default:
      throw CoordinateConversionException( INVALID_SOURCE_CS );
  }

  return error_Code;
}


void Fiomeths::writeOutputFileHeader(
   const char*                  targetDatumCodeLoc,
   CoordinateSystemParameters* _targetParameters )
{
  char projectionName[COORD_SYS_NAME_LENGTH] = "";
  char tempCode[DATUM_CODE_LENGTH] = "";
  char tempName[DATUM_NAME_LENGTH] = "";
  char ellipsoidCode[ELLIPSOID_CODE_LENGTH] = "";
  char latitude_str[17] = "";
  char longitude_str[17] = "";
  char meter_str[17] = "";

  CoordinateSystemParameters parameters;
  long datum_Index = 0;

  targetProjectionType = _targetParameters->coordinateType();

  if (Projection_to_String(targetProjectionType, projectionName))
    throw CoordinateConversionException( INVALID_TARGET_CS );

  if ((targetProjectionType == CoordinateType::britishNationalGrid) ||
      (targetProjectionType == CoordinateType::globalAreaReferenceSystem) ||
      (targetProjectionType == CoordinateType::geodetic) ||
      (targetProjectionType == CoordinateType::georef) ||
      (targetProjectionType == CoordinateType::geocentric) ||
      (targetProjectionType == CoordinateType::localCartesian) ||
      (targetProjectionType == CoordinateType::militaryGridReferenceSystem) ||
      (targetProjectionType == CoordinateType::usNationalGrid) ||
      (targetProjectionType == CoordinateType::universalTransverseMercator) ||
      (targetProjectionType == CoordinateType::universalPolarStereographic))
    fprintf(outputFile, "%s: ", coordinates_Header_String);
  else
    fprintf(outputFile, "%s: ", projection_Header_String);

  fprintf(outputFile, "%s\n", projectionName);

  fprintf(outputFile, "%s: ", datum_Header_String);
  fprintf(outputFile, "%s\n", targetDatumCodeLoc);

  DatumLibrary* datumLibrary = coordinateConversionService->getDatumLibrary();
  long datumIndex;// = datumLibrary->datumIndex(targetDatumCodeLoc);
  datumLibrary->getDatumIndex( targetDatumCodeLoc, &datumIndex );
  datumLibrary->getDatumInfo( datumIndex, tempCode, tempName, ellipsoidCode );

  fprintf(outputFile, "# ELLIPSOID: %s\n", ellipsoidCode);

  switch (targetProjectionType)
  {
    case CoordinateType::geodetic:
    {
      if( dynamic_cast< GeodeticParameters* >( _targetParameters ) )
      {
        GeodeticParameters params = *dynamic_cast< GeodeticParameters* >( _targetParameters );

        if (params.heightType() == HeightType::noHeight)
          fprintf(outputFile, "%s", no_Height_Header_String);
        else if (params.heightType() == HeightType::ellipsoidHeight)
          fprintf(outputFile, "%s", ellipsoid_Height_Header_String);
        else if (params.heightType() == HeightType::EGM96FifteenMinBilinear)
          fprintf(outputFile, "%s", msl_EGM96_15M_BL_Height_Header_String);
        else if (params.heightType() == HeightType::EGM96VariableNaturalSpline)
          fprintf(outputFile, "%s", msl_EGM96_VG_NS_Height_Header_String);
        else if (params.heightType() == HeightType::EGM84TenDegBilinear)
          fprintf(outputFile, "%s", msl_EGM84_10D_BL_Height_Header_String);
        else if (params.heightType() == HeightType::EGM84TenDegNaturalSpline)
          fprintf(outputFile, "%s", msl_EGM84_10D_NS_Height_Header_String);
        else if (params.heightType() == HeightType::EGM84ThirtyMinBiLinear)
          fprintf(outputFile, "%s", msl_EGM84_30M_BL_Height_Header_String);
        else if (params.heightType() == HeightType::EGM2008TwoPtFiveMinBicubicSpline)
          fprintf(outputFile, "%s", msl_EGM2008_TWOPOINTFIVEM_BCS_Height_Header_String);
        fprintf(outputFile,"\n");

        if(outputLatitudeLongitudeOrder)
          fprintf(outputFile, "%s: %s\n", coordinate_Order_Header_String, LATITUDE_LONGITUDE);
        else
          fprintf(outputFile, "%s: %s\n", coordinate_Order_Header_String, LONGITUDE_LATITUDE);
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::britishNationalGrid:
    case CoordinateType::globalAreaReferenceSystem:
    case CoordinateType::georef:
    case CoordinateType::geocentric:
    case CoordinateType::militaryGridReferenceSystem:
    case CoordinateType::newZealandMapGrid:
    case CoordinateType::universalPolarStereographic:
    case CoordinateType::usNationalGrid:
    case CoordinateType::universalTransverseMercator:
    break;
    case CoordinateType::localCartesian:
    {
      if( dynamic_cast< LocalCartesianParameters* >( _targetParameters ) )
      {
        LocalCartesianParameters params = *dynamic_cast< LocalCartesianParameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", origin_Latitude_Header_String);
        Latitude_to_String((params.latitude()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", origin_Longitude_Header_String);
        Longitude_to_String((params.longitude()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", origin_Height_Header_String);
        Meter_to_String(params.height(), meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", orientation_Header_String);
        Longitude_to_String((params.orientation()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::albersEqualAreaConic:
    case CoordinateType::lambertConformalConic2Parallels:
    {
      if( dynamic_cast< MapProjection6Parameters* >( _targetParameters ) )
      {
        MapProjection6Parameters params = *dynamic_cast< MapProjection6Parameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", origin_Latitude_Header_String);
        Latitude_to_String((params.originLatitude()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", standard_Parallel_One_Header_String);
        Latitude_to_String((params.standardParallel1()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", standard_Parallel_Two_Header_String);
        Latitude_to_String((params.standardParallel2()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
  /*  {
      if( dynamic_cast< MapProjection5Parameters* >( _targetParameters ) )
      {
        MapProjection5Parameters params = *dynamic_cast< MapProjection5Parameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", origin_Latitude_Header_String);
        Latitude_to_String((params.originLatitude()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", scale_Factor_Header_String);
        fprintf(outputFile, "%.5f", params.scaleFactor());

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }*/
    case CoordinateType::lambertConformalConic1Parallel:
    case CoordinateType::transverseCylindricalEqualArea:
    case CoordinateType::transverseMercator:
    {
      if( dynamic_cast< MapProjection5Parameters* >( _targetParameters ) )
      {
        MapProjection5Parameters params = *dynamic_cast< MapProjection5Parameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", origin_Latitude_Header_String);
        Latitude_to_String((params.originLatitude()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", scale_Factor_Header_String);
        fprintf(outputFile, "%.5f", params.scaleFactor());

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::mercatorStandardParallel:
    {
      if( dynamic_cast< MercatorStandardParallelParameters* >( _targetParameters ) )
      {
        MercatorStandardParallelParameters params = *dynamic_cast< MercatorStandardParallelParameters* >( coordinateConversionService->getCoordinateSystem( SourceOrTarget::target ) );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", standard_Parallel_Header_String);
        Latitude_to_String((params.standardParallel()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", scale_Factor_Header_String);
        fprintf(outputFile, "%.5f", params.scaleFactor());

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::mercatorScaleFactor:
    {
      if( dynamic_cast< MercatorScaleFactorParameters* >( _targetParameters ) )
      {
        MercatorScaleFactorParameters params = *dynamic_cast< MercatorScaleFactorParameters* >( coordinateConversionService->getCoordinateSystem( SourceOrTarget::target ) );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", scale_Factor_Header_String);
        fprintf(outputFile, "%.5f", params.scaleFactor());

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::polarStereographicStandardParallel:
    {
      if( dynamic_cast< PolarStereographicStandardParallelParameters* >( _targetParameters ) )
      {
        PolarStereographicStandardParallelParameters params = *dynamic_cast< PolarStereographicStandardParallelParameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", standard_Parallel_Header_String);
        Latitude_to_String((params.standardParallel()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::polarStereographicScaleFactor:
    {
      if( dynamic_cast< PolarStereographicScaleFactorParameters* >( _targetParameters ) )
      {
        PolarStereographicScaleFactorParameters params = *dynamic_cast< PolarStereographicScaleFactorParameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", scale_Factor_Header_String);
        fprintf(outputFile, "%.5f", params.scaleFactor());

        fprintf(outputFile, "\n%s: ", hemisphere_Header_String);
        fprintf(outputFile, "%c", params.hemisphere());

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

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
      if( dynamic_cast< MapProjection4Parameters* >( _targetParameters ) )
      {
        MapProjection4Parameters params = *dynamic_cast< MapProjection4Parameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", origin_Latitude_Header_String);
        Latitude_to_String((params.originLatitude()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::eckert4:
    case CoordinateType::eckert6:
    case CoordinateType::millerCylindrical:
    case CoordinateType::mollweide:
    case CoordinateType::sinusoidal:
    case CoordinateType::vanDerGrinten:
    {
      if( dynamic_cast< MapProjection3Parameters* >( _targetParameters ) )
      {
        MapProjection3Parameters params = *dynamic_cast< MapProjection3Parameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::equidistantCylindrical:
    {
      if( dynamic_cast< EquidistantCylindricalParameters* >( _targetParameters ) )
      {
        EquidistantCylindricalParameters params = *dynamic_cast< EquidistantCylindricalParameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", standard_Parallel_Header_String);
        Latitude_to_String((params.standardParallel()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::neys:
    {
      if( dynamic_cast< NeysParameters* >( _targetParameters ) )
      {
        NeysParameters params = *dynamic_cast< NeysParameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", central_Meridian_Header_String);
        Longitude_to_String((params.centralMeridian()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", origin_Latitude_Header_String);
        Latitude_to_String((params.originLatitude()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", standard_Parallel_One_Header_String);
        Latitude_to_String((params.standardParallel1()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
    case CoordinateType::obliqueMercator:
    {
      if( dynamic_cast< ObliqueMercatorParameters* >( _targetParameters ) )
      {
        ObliqueMercatorParameters params = *dynamic_cast< ObliqueMercatorParameters* >( _targetParameters );

        fprintf(outputFile, "%s: ", origin_Latitude_Header_String);
        Latitude_to_String((params.originLatitude()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", latitude_One_Header_String);
        Latitude_to_String((params.latitude1()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", longitude_One_Header_String);
        Longitude_to_String((params.longitude1()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", latitude_Two_Header_String);
        Latitude_to_String((params.latitude2()*180)/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", latitude_str);

        fprintf(outputFile, "\n%s: ", longitude_Two_Header_String);
        Longitude_to_String((params.longitude2()*180)/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds);
        fprintf(outputFile, "%s", longitude_str);

        fprintf(outputFile, "\n%s: ", scale_Factor_Header_String);
        fprintf(outputFile, "%.5f", params.scaleFactor());

        fprintf(outputFile, "\n%s: ", false_Easting_Header_String);
        Meter_to_String(params.falseEasting(),meter_str);
        fprintf(outputFile, "%s", meter_str);

        fprintf(outputFile, "\n%s: ", false_Northing_Header_String);
        Meter_to_String(params.falseNorthing(),meter_str);
        fprintf(outputFile, "%s", meter_str);
        fprintf(outputFile, "\n");
      }
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );

      break;
    }
  }

  fprintf(outputFile, "%s\n\n", end_Of_Header_String);
}


void Fiomeths::setCoordinateSystemParameters( MSP::CCS::CoordinateSystemParameters* parameters )
{
/*
 *  The function setCoordinateSystem sets the coordinate system.
 *
 *  parameters : Coordinate system parameters to be used               (input)
 */

  switch( parameters->coordinateType() )
  {
    case CoordinateType::albersEqualAreaConic:
      if( dynamic_cast< MapProjection6Parameters* >( parameters ) )
        targetParameters = new MapProjection6Parameters( *dynamic_cast< MapProjection6Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
     break;
    case CoordinateType::azimuthalEquidistant:
      if( dynamic_cast< MapProjection4Parameters* >( parameters ) )
        targetParameters = new MapProjection4Parameters( *dynamic_cast< MapProjection4Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::bonne:
      if( dynamic_cast< MapProjection4Parameters* >( parameters ) )
        targetParameters = new MapProjection4Parameters( *dynamic_cast< MapProjection4Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::britishNationalGrid:
      if( dynamic_cast< CoordinateSystemParameters* >( parameters ) )
        targetParameters = new CoordinateSystemParameters( *dynamic_cast< CoordinateSystemParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::cassini:
      if( dynamic_cast< MapProjection4Parameters* >( parameters ) )
        targetParameters = new MapProjection4Parameters( *dynamic_cast< MapProjection4Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::cylindricalEqualArea:
      if( dynamic_cast< MapProjection4Parameters* >( parameters ) )
        targetParameters = new MapProjection4Parameters( *dynamic_cast< MapProjection4Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::eckert4:
      if( dynamic_cast< MapProjection3Parameters* >( parameters ) )
        targetParameters = new MapProjection3Parameters( *dynamic_cast< MapProjection3Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::eckert6:
      if( dynamic_cast< MapProjection3Parameters* >( parameters ) )
        targetParameters = new MapProjection3Parameters( *dynamic_cast< MapProjection3Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::equidistantCylindrical:
      if( dynamic_cast< EquidistantCylindricalParameters* >( parameters ) )
        targetParameters = new EquidistantCylindricalParameters( *dynamic_cast< EquidistantCylindricalParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::geocentric:
      if( dynamic_cast< CoordinateSystemParameters* >( parameters ) )
        targetParameters = new CoordinateSystemParameters( *dynamic_cast< CoordinateSystemParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::geodetic:
      if( dynamic_cast< GeodeticParameters* >( parameters ) )
        targetParameters = new GeodeticParameters( *dynamic_cast< GeodeticParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::georef:
      if( dynamic_cast< CoordinateSystemParameters* >( parameters ) )
        targetParameters = new CoordinateSystemParameters( *dynamic_cast< CoordinateSystemParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::globalAreaReferenceSystem:
      if( dynamic_cast< CoordinateSystemParameters* >( parameters ) )
        targetParameters = new CoordinateSystemParameters( *dynamic_cast< CoordinateSystemParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::gnomonic:
      if( dynamic_cast< MapProjection4Parameters* >( parameters ) )
        targetParameters = new MapProjection4Parameters( *dynamic_cast< MapProjection4Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::lambertConformalConic1Parallel:
      if( dynamic_cast< MapProjection5Parameters* >( parameters ) )
        targetParameters = new MapProjection5Parameters( *dynamic_cast< MapProjection5Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::lambertConformalConic2Parallels:
      if( dynamic_cast< MapProjection6Parameters* >( parameters ) )
        targetParameters = new MapProjection6Parameters( *dynamic_cast< MapProjection6Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::localCartesian:
      if( dynamic_cast< LocalCartesianParameters* >( parameters ) )
        targetParameters = new LocalCartesianParameters( *dynamic_cast< LocalCartesianParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::mercatorStandardParallel:
      if( dynamic_cast< MercatorStandardParallelParameters* >( parameters ) )
        targetParameters = new MercatorStandardParallelParameters( *dynamic_cast< MercatorStandardParallelParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::mercatorScaleFactor:
      if( dynamic_cast< MercatorScaleFactorParameters* >( parameters ) )
        targetParameters = new MercatorScaleFactorParameters( *dynamic_cast< MercatorScaleFactorParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::militaryGridReferenceSystem:
      if( dynamic_cast< CoordinateSystemParameters* >( parameters ) )
        targetParameters = new CoordinateSystemParameters( *dynamic_cast< CoordinateSystemParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::millerCylindrical:
      if( dynamic_cast< MapProjection3Parameters* >( parameters ) )
        targetParameters = new MapProjection3Parameters( *dynamic_cast< MapProjection3Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::mollweide:
      if( dynamic_cast< MapProjection3Parameters* >( parameters ) )
        targetParameters = new MapProjection3Parameters( *dynamic_cast< MapProjection3Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::newZealandMapGrid:
      if( dynamic_cast< CoordinateSystemParameters* >( parameters ) )
        targetParameters = new CoordinateSystemParameters( *dynamic_cast< CoordinateSystemParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::neys:
      if( dynamic_cast< NeysParameters* >( parameters ) )
        targetParameters = new NeysParameters( *dynamic_cast< NeysParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::obliqueMercator:
      if( dynamic_cast< ObliqueMercatorParameters* >( parameters ) )
        targetParameters = new ObliqueMercatorParameters( *dynamic_cast< ObliqueMercatorParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::orthographic:
      if( dynamic_cast< MapProjection4Parameters* >( parameters ) )
        targetParameters = new MapProjection4Parameters( *dynamic_cast< MapProjection4Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::polarStereographicStandardParallel:
      if( dynamic_cast< PolarStereographicStandardParallelParameters* >( parameters ) )
        targetParameters = new PolarStereographicStandardParallelParameters( *dynamic_cast< PolarStereographicStandardParallelParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::polarStereographicScaleFactor:
      if( dynamic_cast< PolarStereographicScaleFactorParameters* >( parameters ) )
        targetParameters = new PolarStereographicScaleFactorParameters( *dynamic_cast< PolarStereographicScaleFactorParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::polyconic:
      if( dynamic_cast< MapProjection4Parameters* >( parameters ) )
        targetParameters = new MapProjection4Parameters( *dynamic_cast< MapProjection4Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::sinusoidal:
      if( dynamic_cast< MapProjection3Parameters* >( parameters ) )
        targetParameters = new MapProjection3Parameters( *dynamic_cast< MapProjection3Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::stereographic:
      if( dynamic_cast< MapProjection4Parameters* >( parameters ) )
        targetParameters = new MapProjection4Parameters( *dynamic_cast< MapProjection4Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::transverseCylindricalEqualArea:
      if( dynamic_cast< MapProjection5Parameters* >( parameters ) )
        targetParameters = new MapProjection5Parameters( *dynamic_cast< MapProjection5Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::transverseMercator:
      if( dynamic_cast< MapProjection5Parameters* >( parameters ) )
        targetParameters = new MapProjection5Parameters( *dynamic_cast< MapProjection5Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::universalPolarStereographic:
      if( dynamic_cast< CoordinateSystemParameters* >( parameters ) )
        targetParameters = new CoordinateSystemParameters( *dynamic_cast< CoordinateSystemParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::universalTransverseMercator:
      if( dynamic_cast< UTMParameters* >( parameters ) )
        targetParameters = new UTMParameters( *dynamic_cast< UTMParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::usNationalGrid:
      if( dynamic_cast< CoordinateSystemParameters* >( parameters ) )
        targetParameters = new CoordinateSystemParameters( *dynamic_cast< CoordinateSystemParameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    case CoordinateType::vanDerGrinten:
      if( dynamic_cast< MapProjection3Parameters* >( parameters ) )
        targetParameters = new MapProjection3Parameters( *dynamic_cast< MapProjection3Parameters* >( parameters ) );
      else
        throw CoordinateConversionException( INVALID_TARGET_PARAMETERS );
      break;
    default:
      throw CoordinateConversionException( INVALID_TARGET_CS );
  }
}


CoordinateTuple* Fiomeths::readCoordinate()
{
  long tempErrorCode = 0;

  switch(sourceProjectionType)
  {
    case CoordinateType::geodetic:
    {
      char buf[256] = "";
      long length;
      double temp_val = 0;
      double longitude = 0;
      double latitude = 0;
      double height = 0;

      Eat_Noise(inputFile);
      if (!feof(inputFile) && (fscanf(inputFile, "%255[^,\t\n]", buf)))
      {
        if( inputLatitudeLongitudeOrder )  // first value is latitude
          tempErrorCode = String_to_Latitude(buf, &temp_val);
        else                      // first value is longitude
          tempErrorCode = String_to_Longitude(buf, &temp_val);

        if(!tempErrorCode)
        {
          if( inputLatitudeLongitudeOrder )  // first value is latitude
            latitude = temp_val*PI/180;
          else                      // first value is longitude
            longitude =(temp_val*PI)/180;

          fscanf(inputFile, "%[, \t]", buf);
          if (!feof(inputFile) && (fscanf(inputFile, "%255[^,\t\n#;]", buf)))
          {
            length = strlen(buf)-1;
            if (!isdigit(buf[length]) && !isalpha(buf[length]))
              buf[length] = 0;

            if( inputLatitudeLongitudeOrder )  // second value is longitude
              tempErrorCode = String_to_Longitude(buf, &temp_val);
            else                      // second value is latitude
              tempErrorCode = String_to_Latitude(buf, &temp_val);

            if(!tempErrorCode)
            {
              if( inputLatitudeLongitudeOrder )  // second value is longitude
                longitude = temp_val*PI/180;
              else                     // second value is latitude
                latitude =(temp_val*PI)/180;

              /*only read height if not No Height*/
              if(geodeticParameters && geodeticParameters->heightType() != HeightType::noHeight)
              {
                fscanf(inputFile, "%[, \t]", buf);
                if (!feof(inputFile) && (fscanf(inputFile, "%[^, \t\n#;]", buf)))
                {
                  length = strlen(buf)-1;
                  if (!isdigit(buf[length]))
                    buf[length] = 0;
                  double tempHeight;
                  tempErrorCode = String_to_Double(buf,&tempHeight);
                  height = tempHeight;
                }
                return new GeodeticCoordinates(CoordinateType::geodetic, longitude, latitude, height);
              }
              else
                return new GeodeticCoordinates(CoordinateType::geodetic, longitude, latitude, 0.0);

            }
            throw CoordinateConversionException( ERROR_PARSING_FILE );
          }
          else
            throw CoordinateConversionException( ERROR_PARSING_FILE );
        }
        else
          throw CoordinateConversionException( ERROR_PARSING_FILE );
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::georef:
    {
      char return_Parameter[32];
      int i = 0;

      Eat_Noise(inputFile);
      if (!feof(inputFile) && (fscanf(inputFile, "%255[1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ]", return_Parameter)))
      {
        for (i = 0;(unsigned int)i < strlen(return_Parameter); i++)
          return_Parameter[i] = (char)toupper(return_Parameter[i]);

        return new GEOREFCoordinates(CoordinateType::georef, return_Parameter);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );

      break;
    }
    case CoordinateType::localCartesian:
    {
      char buf[256];
      double x, y, z;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &x, &y );
      if(!tempErrorCode)
      {
        fscanf(inputFile, "%[, \t]", buf);
        if (!feof(inputFile) && (fscanf(inputFile, "%255[^, \t\n;]", buf)))
        {
          tempErrorCode = String_to_Double( buf, &z );
          if(!tempErrorCode)
          {
            return new CartesianCoordinates(CoordinateType::localCartesian, x, y, z);
          }
          if( tempErrorCode )
            throw CoordinateConversionException( ERROR_PARSING_FILE );
        }
        else
          throw CoordinateConversionException( ERROR_PARSING_FILE );
      }
    }
    case CoordinateType::geocentric:
    {
      char buf[256];
      double x, y, z;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &x, &y );
      if(!tempErrorCode)
      {
        fscanf(inputFile, "%[, \t]", buf);
        if (!feof(inputFile) && (fscanf(inputFile, "%255[^, \t\n;]", buf)))
        {
          tempErrorCode = String_to_Double( buf, &z );
          if(!tempErrorCode)
          {
            return new CartesianCoordinates(CoordinateType::geocentric, x, y, z);
          }
          if( tempErrorCode )
            throw CoordinateConversionException( ERROR_PARSING_FILE );
        }
        else
          throw CoordinateConversionException( ERROR_PARSING_FILE );
      }
    }
    case CoordinateType::militaryGridReferenceSystem:
    {
      char return_Parameter[32];
      int i = 0;

      Eat_Noise(inputFile);
      if (!feof(inputFile) && (fscanf(inputFile, "%255[1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ]", return_Parameter)))
      {
        for (i = 0;(unsigned int)i < strlen(return_Parameter); i++)
          return_Parameter[i] = (char)toupper(return_Parameter[i]);
        return new MGRSorUSNGCoordinates(CoordinateType::militaryGridReferenceSystem, return_Parameter, Precision::tenthOfSecond);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );

      break;
    }
    case CoordinateType::usNationalGrid:
    {
      char return_Parameter[32];
      int i = 0;

      Eat_Noise(inputFile);
      if (!feof(inputFile) && (fscanf(inputFile, "%255[1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ]", return_Parameter)))
      {
        for (i = 0;(unsigned int)i < strlen(return_Parameter); i++)
          return_Parameter[i] = (char)toupper(return_Parameter[i]);
        return new MGRSorUSNGCoordinates(CoordinateType::usNationalGrid, return_Parameter, Precision::tenthOfSecond);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );

      break;
    }
    case CoordinateType::universalTransverseMercator:
    {
      char buf[256];
      long zone;
      char hemisphere;
      double easting, northing;
      double temp = 0;

      Eat_Noise(inputFile);
      if (!feof(inputFile) && (fscanf(inputFile, "%255[^, \t\n]", buf)))
      {
        tempErrorCode = String_to_Double(buf,&temp);
        if(!tempErrorCode)
        {
          zone = (long)temp;
          fscanf(inputFile, "%[, \t]", buf);
          if (!feof(inputFile) && (fscanf(inputFile, "%255[^, \t\n]", &hemisphere)))
          {
            hemisphere = (char)toupper(hemisphere);
            if ((hemisphere != 'N') && (hemisphere != 'S'))
              throw CoordinateConversionException( ERROR_PARSING_FILE );
            else
            {
              fscanf(inputFile, "%[, \t]", buf);
              tempErrorCode = readCoord( &easting, &northing );
              if(!tempErrorCode)
              {
                return new UTMCoordinates(CoordinateType::universalTransverseMercator, zone, hemisphere, easting, northing);
              }
              else
                throw CoordinateConversionException( ERROR_PARSING_FILE );
            }
          }
          else
            throw CoordinateConversionException( ERROR_PARSING_FILE );
        }
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::universalPolarStereographic:
    {
      char buf[256];
      char hemisphere;
      double easting, northing;

      Eat_Noise(inputFile);
      if (!feof(inputFile) && (fscanf(inputFile, "%255[^, \t\n]", &hemisphere)))
      {
        hemisphere = (char)toupper(hemisphere);
        if ((hemisphere != 'N') && (hemisphere != 'S'))
          throw CoordinateConversionException( ERROR_PARSING_FILE );
        else
        {
          fscanf(inputFile, "%[, \t]", buf);
          tempErrorCode = readCoord(&easting, &northing);
          if(!tempErrorCode)
          {
            return new UPSCoordinates(CoordinateType::universalPolarStereographic, hemisphere, easting, northing);
          }
          if( tempErrorCode )
            throw CoordinateConversionException( ERROR_PARSING_FILE );
        }
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::albersEqualAreaConic:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::albersEqualAreaConic, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::azimuthalEquidistant:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::azimuthalEquidistant, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::bonne:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::bonne, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::cassini:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::cassini, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::cylindricalEqualArea:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::cylindricalEqualArea, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::eckert4:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::eckert4, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::eckert6:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::eckert6, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::equidistantCylindrical:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::equidistantCylindrical, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::gnomonic:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::gnomonic, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::lambertConformalConic1Parallel:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::lambertConformalConic1Parallel, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::lambertConformalConic2Parallels:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::lambertConformalConic2Parallels, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::mercatorStandardParallel:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::mercatorStandardParallel, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::mercatorScaleFactor:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::mercatorScaleFactor, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::millerCylindrical:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::millerCylindrical, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::mollweide:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::mollweide, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::neys:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::neys, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::newZealandMapGrid:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::newZealandMapGrid, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::obliqueMercator:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::obliqueMercator, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::orthographic:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::orthographic, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::polarStereographicStandardParallel:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::polarStereographicStandardParallel, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::polarStereographicScaleFactor:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::polarStereographicScaleFactor, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::polyconic:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::polyconic, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::sinusoidal:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::sinusoidal, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::stereographic:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::stereographic, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::transverseCylindricalEqualArea:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::transverseCylindricalEqualArea, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::transverseMercator:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::transverseMercator, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::vanDerGrinten:
    {
      double easting, northing;

      Eat_Noise(inputFile);
      tempErrorCode = readCoord( &easting, &northing );
      if(!tempErrorCode)
      {
        return new MapProjectionCoordinates(CoordinateType::vanDerGrinten, easting, northing);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );
      break;
    }
    case CoordinateType::britishNationalGrid:
    {
      BNGCoordinates coord;
      char return_Parameter[32];
      int i = 0;

      Eat_Noise(inputFile);
      if (!feof(inputFile) && (fscanf(inputFile,
         "%255[ 1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ]",
         return_Parameter)))
      {
        for (i = 0;(unsigned int)i < strlen(return_Parameter); i++)
          return_Parameter[i] = (char)toupper(return_Parameter[i]);

        return new BNGCoordinates(CoordinateType::britishNationalGrid, return_Parameter);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );

      break;
    }
    case CoordinateType::globalAreaReferenceSystem:
    {
      GARSCoordinates coord;
      char return_Parameter[32];
      int i = 0;

      Eat_Noise(inputFile);
      if (!feof(inputFile) && (fscanf(inputFile,
         "%255[1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ]",
         return_Parameter)))
      {
        for (i = 0;(unsigned int)i < strlen(return_Parameter); i++)
          return_Parameter[i] = (char)toupper(return_Parameter[i]);

        return new GARSCoordinates(CoordinateType::globalAreaReferenceSystem, return_Parameter);
      }
      else
        throw CoordinateConversionException( ERROR_PARSING_FILE );

      break;
    }
    default:
      throw CoordinateConversionException( INVALID_TARGET_CS );
  }

  return 0;
}


long Fiomeths::readCoord(double *easting, double *northing)
{
  char buf[256];
  FVC_Status error_Code = FVC_Success;

  if (!feof(inputFile) && (fscanf(inputFile, "%255[^, \t\n]", buf)))
    error_Code = (FVC_Status)String_to_Double(buf, easting);
  else
    error_Code = FVC_Error;

  if(!error_Code)
  {
    fscanf(inputFile, "%[, \t]", buf);
    if (!feof(inputFile) && (fscanf(inputFile, "%255[^, \t\n;]", buf)))
      error_Code = (FVC_Status)String_to_Double(buf, northing);
    else
      error_Code = FVC_Error;
  }
  return error_Code;
}


long Fiomeths::readHeight(char* height)
{
  char buf[256];
  FVC_Status error_Code = FVC_Success;

  fscanf(inputFile, "%[, \t]", buf);
  if (!feof(inputFile) && (fscanf(inputFile, "%255[^, \t\n;]", buf)))
    strcpy(height, buf);
  else
    error_Code = FVC_Error;

  return error_Code;
}


Accuracy* Fiomeths::readConversionErrors( char* errors )
{
  char buf[256] = "";
  double ce90 = -1.0;
  double le90 = -1.0;
  double se90 = -1.0;
  int index = 0;

  sscanf(errors, "%[^:\t]", buf);
  if(strstr(errors, ce90_String))
  {
    index = strlen(buf);
    buf[0] = 0;
    sscanf(errors + index, "%[: ]", buf);
    index += strlen(buf);
    buf[0] = 0;
    sscanf(errors + index, "%[^,]", buf);
    if(strcmp(buf, "Unk") == 0)
      ce90 = -1.0;
    else
      ce90 = atof(buf);
    index += strlen(buf);
    buf[0] = 0;

    sscanf(errors + index, "%[, ]", buf);
    index += strlen(buf);
    buf[0] = 0;
    sscanf(errors + index, "%[^:\t]", buf);
    if(strstr(errors, le90_String))
    {
      index += strlen(buf);
      buf[0] = 0;
      sscanf(errors + index, "%[: ]", buf);
      index += strlen(buf);
      buf[0] = 0;
      sscanf(errors + index, "%[^,]", buf);
      if(strcmp(buf, "Unk") == 0)
        le90 = -1.0;
      else
        le90 = atof(buf);
      index += strlen(buf);
      buf[0] = 0;

      sscanf(errors + index, "%[, ]", buf);
      index += strlen(buf);
      buf[0] = 0;
      sscanf(errors + index, "%[^:\t]", buf);
      if(strstr(errors, se90_String))
      {
        index += strlen(buf);
        buf[0] = 0;
        sscanf(errors + index, "%[: ]", buf);
        index += strlen(buf);
        buf[0] = 0;
        sscanf(errors + index, "%[^,\n]", buf);
        if(strcmp(buf, "Unk") == 0 || strcmp(buf, "Unk ") == 0)
          se90 = -1.0;
        else
          se90 = atof(buf);
      }
    }
  }

  return new Accuracy( ce90, le90, se90 );
}


void Fiomeths::convert( 
   std::vector<MSP::CCS::CoordinateTuple*>& sourceCoordinateCollection,
   std::vector<MSP::CCS::Accuracy*>&        sourceAccuracyCollection,
   std::vector<TrailingHeight>&             trailingHeightCollection,
   std::vector<MSP::CCS::CoordinateTuple*>& targetCoordinateCollection,
   std::vector<MSP::CCS::Accuracy*>&        targetAccuracyCollection )
{
  int numSourceCoordinates = sourceCoordinateCollection.size();

  // Convert
/*  std::vector<CoordinateTuple*> targetCoordinateCollection;
  targetCoordinateCollection.reserve( numSourceCoordinates );
  std::vector<Accuracy> targetAccuracyCollection;
  targetAccuracyCollection.reserve( numSourceCoordinates );*/
//  targetCoordinateCollection.reserve( numSourceCoordinates );
//  targetAccuracyCollection.reserve( numSourceCoordinates );

///  static time_t t_start, t_stop;

///	t_start = time( NULL );

  coordinateConversionService->convertSourceToTargetCollection(
     sourceCoordinateCollection,
     sourceAccuracyCollection,
     targetCoordinateCollection,
     targetAccuracyCollection );

///	t_stop = time( NULL );

///  _elapsedTime = ( double )( t_stop - t_start );

  // Write coordinates to output file
  int numTargetCoordinates = targetCoordinateCollection.size();
  int numTargetAccuracies  = targetAccuracyCollection.size();
  int numTrailingHeights   = trailingHeightCollection.size();

  if(( numTargetCoordinates == numTargetAccuracies ) &&
     ( numTargetCoordinates == numTrailingHeights  ) )
  {
    for( int i = 0; i < numTargetCoordinates; i++ )
    {
      CoordinateTuple* targetCoordinate = targetCoordinateCollection[i];
      Accuracy* targetAccuracy          = targetAccuracyCollection[i];
      TrailingHeight trailingHeight     = trailingHeightCollection[i];

      if( targetCoordinate->coordinateType() == invalid )
      {
        Output_String( outputFile, targetCoordinate->errorMessage() );
      }
      else
      {
        // Write any warning messages
        int length = strlen( targetCoordinate->warningMessage() );
        if( length > 0 )
        {
          Output_String( outputFile, "# Warning: " );

          char buf[256];
          const char* targetWarningMessage = targetCoordinate->warningMessage();

          int index = 0;
          do
          {
            buf[0] = 0;
            const char* tempWarningMessage = targetWarningMessage + index;
            sscanf(tempWarningMessage, "%[^\n]", buf);
            index += strlen( buf );
            Output_String( outputFile, buf );
            buf[0] = 0;
            sscanf(targetWarningMessage + index, "%[\n]", buf);
            index += strlen( buf );
          }
          while( strlen( buf ) > 0 && index < length );

          Output_Newline(outputFile);

          _numWarnings ++;
        }

        // Write any error messages
        length = strlen( targetCoordinate->errorMessage() );
        if( length > 0)
        {
          Output_String( outputFile, "# Error: " );

          char buf[256];
          const char* targetErrorMessage = targetCoordinate->errorMessage();

          int index = 0;
          do
          {
            buf[0] = 0;
            const char* tempErrorMessage = targetErrorMessage + index;
            sscanf(tempErrorMessage, "%[^\n]", buf);
            index += strlen( buf );
            Output_String( outputFile, buf );
            buf[0] = 0;
            sscanf(targetErrorMessage + index, "%[\n]", buf);
            index += strlen( buf );
          }
          while( strlen( buf ) > 0 && index < length );

          _numErrors ++;
        }
        else
        {
          writeTargetCoordinate( targetCoordinate );
          if( trailingHeight.heightPresent )
            writeHeight( trailingHeight.height );
          writeTargetAccuracy( targetAccuracy );
        }

        // Write out any comments on the coordinate line in the source file
        // or end the current coordinate line
        CoordinateTuple* sourceCoordinate = sourceCoordinateCollection[i];
        if( strlen( sourceCoordinate->errorMessage() ) > 0 )
        {
          Output_String( outputFile, " " );
          Output_String( outputFile, sourceCoordinate->errorMessage() );
        }
        else
          Output_Newline(outputFile);
      }
    }
  }

  // Delete lists
  int i = 0;  // initialize i outside loop for unix
  for( i = 0; i < numSourceCoordinates; i++ )
  {
    delete sourceCoordinateCollection[i];
  }
  sourceCoordinateCollection.clear();

  int numSourceAccuracy = sourceAccuracyCollection.size();
  for( i = 0; i < numSourceAccuracy; i++ )
  {
    delete sourceAccuracyCollection[i];
  }
  sourceAccuracyCollection.clear();

  for( i = 0; i < numTargetCoordinates; i++ )
  {
    delete targetCoordinateCollection[i];
  }
  targetCoordinateCollection.clear();

  for( i = 0; i < numTargetAccuracies; i++ )
  {
    delete targetAccuracyCollection[i];
  }
  targetAccuracyCollection.clear();
  trailingHeightCollection.clear(); // ???
}


CoordinateTuple* Fiomeths::initTargetCoordinate()
{
  switch( targetProjectionType )
  {
    case CoordinateType::albersEqualAreaConic:
      return new MapProjectionCoordinates(CoordinateType::albersEqualAreaConic);
    case CoordinateType::azimuthalEquidistant:
      return new MapProjectionCoordinates(CoordinateType::azimuthalEquidistant);
    case CoordinateType::bonne:
      return new MapProjectionCoordinates(CoordinateType::bonne);
    case CoordinateType::britishNationalGrid:
      return new BNGCoordinates(CoordinateType::britishNationalGrid);
    case CoordinateType::cassini:
      return new MapProjectionCoordinates(CoordinateType::cassini);
    case CoordinateType::cylindricalEqualArea:
      return new MapProjectionCoordinates(CoordinateType::cylindricalEqualArea);
    case CoordinateType::eckert4:
      return new MapProjectionCoordinates(CoordinateType::eckert4);
    case CoordinateType::eckert6:
      return new MapProjectionCoordinates(CoordinateType::eckert6);
    case CoordinateType::equidistantCylindrical:
      return new MapProjectionCoordinates(CoordinateType::equidistantCylindrical);
    case CoordinateType::geocentric:
      return new CartesianCoordinates(CoordinateType::geocentric);
    case CoordinateType::geodetic:
      return new GeodeticCoordinates(CoordinateType::geodetic);
    case CoordinateType::georef:
      return new GEOREFCoordinates(CoordinateType::georef);
    case CoordinateType::globalAreaReferenceSystem:
      return new GARSCoordinates(CoordinateType::globalAreaReferenceSystem);
    case CoordinateType::gnomonic:
      return new MapProjectionCoordinates(CoordinateType::gnomonic);
    case CoordinateType::lambertConformalConic1Parallel:
      return new MapProjectionCoordinates(CoordinateType::lambertConformalConic1Parallel);
    case CoordinateType::lambertConformalConic2Parallels:
      return new MapProjectionCoordinates(CoordinateType::lambertConformalConic2Parallels);
    case CoordinateType::localCartesian:
      return new CartesianCoordinates(CoordinateType::localCartesian);
    case CoordinateType::mercatorStandardParallel:
      return new MapProjectionCoordinates(CoordinateType::mercatorStandardParallel);
    case CoordinateType::mercatorScaleFactor:
      return new MapProjectionCoordinates(CoordinateType::mercatorScaleFactor);
    case CoordinateType::militaryGridReferenceSystem:
      return new MGRSorUSNGCoordinates(CoordinateType::militaryGridReferenceSystem);
    case CoordinateType::millerCylindrical:
      return new MapProjectionCoordinates(CoordinateType::millerCylindrical);
    case CoordinateType::mollweide:
      return new MapProjectionCoordinates(CoordinateType::mollweide);
    case CoordinateType::newZealandMapGrid:
      return new MapProjectionCoordinates(CoordinateType::newZealandMapGrid);
    case CoordinateType::neys:
      return new MapProjectionCoordinates(CoordinateType::neys);
    case CoordinateType::obliqueMercator:
      return new MapProjectionCoordinates(CoordinateType::obliqueMercator);
    case CoordinateType::orthographic:
      return new MapProjectionCoordinates(CoordinateType::orthographic);
    case CoordinateType::polarStereographicStandardParallel:
      return new MapProjectionCoordinates(CoordinateType::polarStereographicStandardParallel);
    case CoordinateType::polarStereographicScaleFactor:
      return new MapProjectionCoordinates(CoordinateType::polarStereographicScaleFactor);
    case CoordinateType::polyconic:
      return new MapProjectionCoordinates(CoordinateType::polyconic);
    case CoordinateType::sinusoidal:
      return new MapProjectionCoordinates(CoordinateType::sinusoidal);
    case CoordinateType::stereographic:
      return new MapProjectionCoordinates(CoordinateType::stereographic);
    case CoordinateType::transverseCylindricalEqualArea:
      return new MapProjectionCoordinates(CoordinateType::transverseCylindricalEqualArea);
    case CoordinateType::transverseMercator:
      return new MapProjectionCoordinates(CoordinateType::transverseMercator);
    case CoordinateType::universalPolarStereographic:
      return new UPSCoordinates(CoordinateType::universalPolarStereographic);
    case CoordinateType::universalTransverseMercator:
      return new UTMCoordinates(CoordinateType::universalTransverseMercator);
    case CoordinateType::usNationalGrid:
      return new MGRSorUSNGCoordinates(CoordinateType::usNationalGrid);
    case CoordinateType::vanDerGrinten:
      return new MapProjectionCoordinates(CoordinateType::vanDerGrinten);
    default:   
      throw CoordinateConversionException( INVALID_TARGET_CS );
  }
}


void Fiomeths::writeTargetCoordinate( CoordinateTuple* targetCoordinate )
{
  char meter_str[17];
  switch( targetProjectionType )
  {
    case CoordinateType::geodetic:
    {
      char latitude_str[17];
      char longitude_str[17];
      if( dynamic_cast< GeodeticParameters* >( targetParameters ) && dynamic_cast< GeodeticCoordinates* >( targetCoordinate ) )
      {
        GeodeticParameters params = ( *dynamic_cast< GeodeticParameters* >( targetParameters ) );
        GeodeticCoordinates coord = ( *dynamic_cast< GeodeticCoordinates* >( targetCoordinate ) );
        if( outputLatitudeLongitudeOrder )  // first value is latitude, second value is longitude
        {
          Latitude_to_String( ( coord.latitude()*180 )/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds );
          fprintf(outputFile, "%s", latitude_str);
          fprintf(outputFile, ", ");
          Longitude_to_String( ( coord.longitude()*180 )/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds );
          fprintf(outputFile, "%s", longitude_str);
        }
        else
        {
          Longitude_to_String( ( coord.longitude()*180 )/PI, longitude_str, _useNSEW, _useMinutes, _useSeconds );
          fprintf(outputFile, "%s", longitude_str);
          fprintf(outputFile, ", ");
          Latitude_to_String( ( coord.latitude()*180 )/PI, latitude_str, _useNSEW, _useMinutes, _useSeconds );
          fprintf(outputFile, "%s", latitude_str);
        }
        /*only output height if not No_Height*/
        if( params.heightType() != HeightType::noHeight )
        {
          fprintf(outputFile, ", ");
          Meter_to_String(coord.height(), meter_str);
          fprintf(outputFile, "%s", meter_str);
        }
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
    case CoordinateType::georef: 
    {
      if( dynamic_cast< GEOREFCoordinates* >( targetCoordinate ) )
      {
        GEOREFCoordinates coord = ( *dynamic_cast< GEOREFCoordinates* >( targetCoordinate ) );
        fprintf(outputFile, "%s", coord.GEOREFString());
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
    case CoordinateType::geocentric: 
    case CoordinateType::localCartesian:  
    {
      if( dynamic_cast< CartesianCoordinates* >( targetCoordinate ) )
      {
        CartesianCoordinates coord = ( *dynamic_cast< CartesianCoordinates* >( targetCoordinate ) );
        writeCoord( coord.x(), coord.y() );
        fprintf(outputFile, ", ");
        Meter_to_String( coord.z(), meter_str );
        fprintf(outputFile, "%s", meter_str);
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
    case CoordinateType::militaryGridReferenceSystem:  
    case CoordinateType::usNationalGrid: 
    {
      if( dynamic_cast< MGRSorUSNGCoordinates* >( targetCoordinate ) )
      {
        MGRSorUSNGCoordinates coord = ( *dynamic_cast< MGRSorUSNGCoordinates* >( targetCoordinate ) );
        fprintf( outputFile, "%s", coord.MGRSString() );
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
    case CoordinateType::universalTransverseMercator:  
    {
      if( dynamic_cast< UTMCoordinates* >( targetCoordinate ) )
      {
        UTMCoordinates coord = ( *dynamic_cast< UTMCoordinates* >( targetCoordinate ) );
        fprintf(outputFile, "%d", coord.zone());
        fprintf(outputFile, ", ");
        fprintf(outputFile, "%c", coord.hemisphere());
        fprintf(outputFile, ", ");
        writeCoord( coord.easting(), coord.northing() );
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
    case CoordinateType::universalPolarStereographic:  
    {
      if( dynamic_cast< UPSCoordinates* >( targetCoordinate ) )
      {
        UPSCoordinates coord = ( *dynamic_cast< UPSCoordinates* >( targetCoordinate ) );
        fprintf(outputFile, "%c", coord.hemisphere());
        fprintf(outputFile, ", ");
        writeCoord( coord.easting(), coord.northing() );
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
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
      if( dynamic_cast< MapProjectionCoordinates* >( targetCoordinate ) )
      {
        MapProjectionCoordinates coord = *( dynamic_cast< MapProjectionCoordinates* >( targetCoordinate ) );
        writeCoord( coord.easting(), coord.northing() );
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
    case CoordinateType::britishNationalGrid:  
    {
      if( dynamic_cast< BNGCoordinates* >( targetCoordinate ) )
      {
        BNGCoordinates coord = ( *dynamic_cast< BNGCoordinates* >( targetCoordinate ) );
        fprintf( outputFile, "%s", coord.BNGString() );
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
    case CoordinateType::globalAreaReferenceSystem: 
    {
      if( dynamic_cast< GARSCoordinates* >( targetCoordinate ) )
      {
        GARSCoordinates coord = ( *dynamic_cast< GARSCoordinates* >( targetCoordinate ) );
        fprintf( outputFile, "%s", coord.GARSString() );
      }
      else
      {
        fprintf(outputFile, INVALID_TARGET_COORDINATES );
        _numErrors++;
      }
      break;
    }
  }
}


void Fiomeths::writeCoord(double easting, double northing)
{
  char meter_str[17];

  Meter_to_String(easting,meter_str);
  fprintf(outputFile, "%s", meter_str);
  fprintf(outputFile, ", ");
  Meter_to_String(northing,meter_str);
  fprintf(outputFile, "%s", meter_str);
}


void Fiomeths::writeHeight(char* height)
{
  fprintf(outputFile, ", ");
  fprintf(outputFile, "%s", height);
}


void Fiomeths::writeTargetAccuracy( Accuracy* accuracy )
{
  double ce90 = accuracy->circularError90();
  double le90 = accuracy->linearError90();
  double se90 = accuracy->sphericalError90();

  if(!((ce90 == -1.0) && (le90 == -1.0) && (se90 == -1.0)))
  {
    fprintf(outputFile, "; CE90: ");
    if (ce90 == -1.0)
      fprintf(outputFile,"Unk");
    else
      fprintf(outputFile,"%1.0f",ce90);
    fprintf(outputFile, ", LE90: ");
    if (le90 == -1.0)
      fprintf(outputFile,"Unk");
    else
      fprintf(outputFile,"%1.0f",le90);
    fprintf(outputFile, ", SE90: ");
    if (se90 == -1.0)
      fprintf(outputFile,"Unk");
    else
      fprintf(outputFile,"%1.0f",se90);
  }
}


void Fiomeths::writeExampleCoord()
{
  char latitude_str[17];
  char longitude_str[17];
  char meter_str[17];

  switch( targetProjectionType )
  {
    case CoordinateType::globalAreaReferenceSystem:
    {
      fprintf(outputFile, "361HN37");
      break;
    }
    case CoordinateType::geodetic:
    {
      Latitude_to_String(0, latitude_str, _useNSEW, _useMinutes, _useSeconds);
      Longitude_to_String(0, longitude_str, _useNSEW, _useMinutes, _useSeconds);

      if(outputLatitudeLongitudeOrder)
      {
        fprintf(outputFile, "%s", latitude_str);
        fprintf(outputFile, ", ");
        fprintf(outputFile, "%s", longitude_str);
      }
      else
      {
        fprintf(outputFile, "%s", longitude_str);
        fprintf(outputFile, ", ");
        fprintf(outputFile, "%s", latitude_str);
      }

      GeodeticParameters params = *dynamic_cast< GeodeticParameters* >( coordinateConversionService->getCoordinateSystem( SourceOrTarget::target ) );

      /*only output height if not No_Height*/
      if( params.heightType() != HeightType::noHeight )
      {
        fprintf(outputFile, ", ");
        Meter_to_String(0,meter_str);
        fprintf(outputFile, "%s", meter_str);
      }
      break;
    }
    case CoordinateType::georef:
    {
      fprintf(outputFile, "NGAA0000000000");
      break;
    }
    case CoordinateType::geocentric:
    {
      writeCoord(0, 0);
      fprintf(outputFile, ", ");
      Meter_to_String(0,meter_str);
      fprintf(outputFile, "%s", meter_str);
      break;
    }
    case CoordinateType::localCartesian:
    {
      writeCoord(0, 0);
      fprintf(outputFile, ", ");
      Meter_to_String(0,meter_str);
      fprintf(outputFile, "%s", meter_str);
      break;
    }
    case CoordinateType::militaryGridReferenceSystem:
    {
      fprintf(outputFile, "31NEA0000000000");
      break;
    }
    case CoordinateType::usNationalGrid:
    {
      fprintf(outputFile, "31NEA0000000000");
      break;
    }
    case CoordinateType::universalTransverseMercator:
    {
      UTMParameters params = *dynamic_cast< UTMParameters* >( coordinateConversionService->getCoordinateSystem( SourceOrTarget::target ) );

      if (params.override())
        fprintf(outputFile, "%d", params.zone());
      else
        fprintf(outputFile, "%d", 0);
      fprintf(outputFile, ", ");
      fprintf(outputFile, "N");
      fprintf(outputFile, ", ");
      writeCoord(500000, 0);
      break;
    }
    case CoordinateType::universalPolarStereographic:
    {
      fprintf(outputFile, "N");
      fprintf(outputFile, ", ");
      writeCoord(2000000, 2000000);
      break;
    }
    case CoordinateType::britishNationalGrid:
    {
      fprintf(outputFile, "SV 0000000000");
      break;
    }
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
      writeCoord(0, 0);
      break;
    }
    case CoordinateType::newZealandMapGrid:
    {
      writeCoord(2510000, 6023150);
      break;
    }
    default:
      throw CoordinateConversionException( INVALID_TARGET_CS );    
  }
}



// CLASSIFICATION: UNCLASSIFIED
