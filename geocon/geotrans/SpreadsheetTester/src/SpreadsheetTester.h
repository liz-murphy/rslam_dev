// CLASSIFICATION: UNCLASSIFIED

#ifndef COORD_TEST_H
  #define COORD_TEST_H


#include <stdio.h>
#include "CoordinateType.h"
#include "SourceOrTarget.h"
#include "CoordinateConversionService.h"


using namespace MSP::CCS;


class SpreadsheetTester
{
public:

  SpreadsheetTester( const char* input_File_Name, const char* output_File_Name, bool _showWarnings );
  ~SpreadsheetTester();

private:

  FILE* input_file;
  FILE* output_file;

  double input_coord1;
  double input_coord2;
  double input_coord3;
  double output_coord1;
  double output_coord2;
  double output_coord3;
  char input_hemisphere;
  char output_hemisphere;
  char *id;
  char *input_coord1_str;
  char *output_coord1_str;
  char *input_datum;
  char *output_datum;
  char *input_projection;
  char *output_projection;
  char *data;
  long input_height;
  long output_height;
  long zone;
  bool showWarnings;
  bool testEGM2008;

  long error_count;
  long warning_different_count;
  long warning_equivalent_count;
  long successful_different_count;
  long successful_equivalent_count;

//compiler bug on solaris 8. Parameters needs to be public
#ifdef SOLARIS
    public:
#endif

  /* Coordinate System Definition with Multiple Variants */
  union Parameters
  {
    MSP::CCS::CoordinateSystemParameters*        coordinateSystemParameters;
    MSP::CCS::MapProjection3Parameters*          mapProjection3Parameters;
    MSP::CCS::MapProjection4Parameters*          mapProjection4Parameters;
    MSP::CCS::MapProjection5Parameters*          mapProjection5Parameters;
    MSP::CCS::MapProjection6Parameters*          mapProjection6Parameters;
    MSP::CCS::EquidistantCylindricalParameters*  equidistantCylindricalParameters;
    MSP::CCS::GeodeticParameters*                geodeticParameters;
    MSP::CCS::LocalCartesianParameters*          localCartesianParameters;
    MSP::CCS::MercatorStandardParallelParameters* mercatorStandardParallelParameters;
    MSP::CCS::MercatorScaleFactorParameters*      mercatorScaleFactorParameters;
    MSP::CCS::NeysParameters*                    neysParameters;
    MSP::CCS::ObliqueMercatorParameters*         obliqueMercatorParameters;
    MSP::CCS::PolarStereographicStandardParallelParameters* polarStereographicStandardParallelParameters;
    MSP::CCS::PolarStereographicScaleFactorParameters*      polarStereographicScaleFactorParameters;
    MSP::CCS::UTMParameters*                     utmParameters;
  };

  /* Coordinate Tuple Definition with Multiple Variants */
  union Coordinates
  {
    MSP::CCS::MapProjectionCoordinates*     mapProjectionCoordinates;
    MSP::CCS::BNGCoordinates*               bngCoordinates;
    MSP::CCS::CartesianCoordinates*         cartesianCoordinates;
    MSP::CCS::GeodeticCoordinates*          geodeticCoordinates;
    MSP::CCS::GEOREFCoordinates*            georefCoordinates;
    MSP::CCS::GARSCoordinates*              garsCoordinates;
    MSP::CCS::MGRSorUSNGCoordinates*        mgrsOrUSNGCoordinates;
    MSP::CCS::UPSCoordinates*               upsCoordinates;
    MSP::CCS::UTMCoordinates*               utmCoordinates;
  };

  /* Coordinate Conversion Service State Definition */
  struct Coordinate_State_Row
  {
    char datumCode[7];     /* currently specified datum code  */
    CoordinateType::Enum coordinateType;   /* current coordinate system type  */
    Parameters parameters;                 /* current coordinate system parameters */
    Coordinates coordinates;               /* current coordinate system parameters */
  };

#ifdef SOLARIS
    private:
#endif

  /* coordinateSystemState[x] is set up as follows:
     c = Number of IO states (Source, Target, etc.) */
  Coordinate_State_Row coordinateSystemState[2];

  MSP::CCS::CoordinateConversionService* coordinateConversionService;

  bool Open_Test_File( SourceOrTarget::Enum direction, const char *filename, FILE **file );
  void Close_Test_File(FILE **file);
  void Convert_Test_File();
  void initCoordinateSystemState( const SourceOrTarget::Enum direction );
  void deleteCoordinateSystem( const SourceOrTarget::Enum direction );
  void Read_Header(void);
  long Read_File(void);
  bool Set_Projection( SourceOrTarget::Enum direction, const char* projection_str/*, CoordinateType::Enum* projection_type*/ );
  bool Set_Datums(const char* datum_str, SourceOrTarget::Enum direction);
  void Read_Parameters( SourceOrTarget::Enum direction, CoordinateType::Enum projection_type );
  MSP::CCS::CoordinateSystemParameters* getCoordinateSystem( const SourceOrTarget::Enum direction );
  void convertSourceToTarget();
  void Write_To_File(bool file_read_error);
  void Write_Parameters( SourceOrTarget::Enum direction, CoordinateType::Enum projectionType );
};

#endif

// CLASSIFICATION: UNCLASSIFIED
