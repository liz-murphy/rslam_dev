// CLASSIFICATION: UNCLASSIFIED

#ifndef ErrorMessages_H
#define ErrorMessages_H


/**
 * Defines all error messages which may be returned by the ccs. 
 * 
 * @author comstam
 */

#include "DtccApi.h"


namespace MSP
{
  namespace CCS
  {
    class MSP_DTCC_API ErrorMessages
    {  
      public:

        static char* geoidFileOpenError;
        static char* geoidFileParseError;
  
        static char* ellipsoidFileOpenError;
        static char* ellipsoidFileCloseError;
        static char* ellipsoidFileParseError;
        static char* ellipsoidOverflow;
        static char* ellipse;
        static char* invalidEllipsoidCode;
  
        static char* datumFileOpenError;
        static char* datumFileCloseError;
        static char* datumFileParseError;
        static char* datumDomain;
        static char* datumOverflow;
        static char* datumRotation;
        static char* datumSigma;
        static char* datumType;
        static char* invalidDatumCode;

        static char* notUserDefined;
        static char* ellipseInUse;
   
        // Parameter error messages
        static char* semiMajorAxis;
        static char* ellipsoidFlattening;
        static char* orientation;
        static char* originLatitude;
        static char* originLongitude;
        static char* centralMeridian;
        static char* scaleFactor;
        static char* zone;
        static char* zoneOverride;
        static char* standardParallel1;
        static char* standardParallel2;
        static char* standardParallel1_2;
        static char* standardParallelHemisphere;
        static char* precision;
        static char* bngEllipsoid;
        static char* nzmgEllipsoid;
        static char* latitude1;
        static char* latitude2;
        static char* latitude1_2;
        static char* longitude1;
        static char* longitude2;
        static char* omercHemisphere;
        static char* hemisphere;
        static char* radius;

        // Coordinate error messages
        static char* latitude;
        static char* longitude;
        static char* easting;
        static char* northing;
        static char* projection;
        static char* invalidArea;
        static char* bngString;
        static char* garsString;
        static char* georefString;
        static char* mgrsString;
        static char* usngString;
   /*   const char* utmZoneRow = "Invalid UTM Zone Row";
      const char* zone = "Zone out of range (1-60)\n";
  
      const char* mgrsString = "Invalid MGRS String\n";

      const char* radius = "Easting/Northing too far from center of projection\n";
  
      const char* hemisphere = "Invalid Hemisphere\n";
      const char* signHemisphere = "Mismatched sign and hemisphere \n";
      const char* degrees = "Degrees value out of range\n";
      const char* minutes = "Minutes value out of range\n";
      const char* seconds = "Seconds value out of range\n";*/
  
        static char* invalidIndex;
        static char* invalidName;
        static char* invalidType;
        static char* latitude_min;
        static char* longitude_min;
    };
  }
}
	
#endif 

// CLASSIFICATION: UNCLASSIFIED
