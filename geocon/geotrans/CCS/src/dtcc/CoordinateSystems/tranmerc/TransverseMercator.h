// CLASSIFICATION: UNCLASSIFIED

#ifndef TransverseMercator_H
#define TransverseMercator_H

/***************************************************************************/
/* RSC IDENTIFIER: TRANSVERSE MERCATOR
 *
 * ABSTRACT
 *
 *    This component provides conversions between Geodetic coordinates 
 *    (latitude and longitude) and Transverse Mercator projection coordinates
 *    (easting and northing).
 *
 * ERROR HANDLING
 *
 *    This component checks parameters for valid values.  If an invalid value
 *    is found the error code is combined with the current error code using 
 *    the bitwise or.  This combining allows multiple error codes to be
 *    returned. The possible error codes are:
 *
 *       TRANMERC_NO_ERROR           : No errors occurred in function
 *       TRANMERC_LAT_ERROR          : Latitude outside of valid range
 *                                      (-90 to 90 degrees)
 *       TRANMERC_LON_ERROR          : Longitude outside of valid range
 *                                      (-180 to 360 degrees, and within
 *                                        +/-90 of Central Meridian)
 *       TRANMERC_EASTING_ERROR      : Easting outside of valid range
 *                                      (depending on ellipsoid and
 *                                       projection parameters)
 *       TRANMERC_NORTHING_ERROR     : Northing outside of valid range
 *                                      (depending on ellipsoid and
 *                                       projection parameters)
 *       TRANMERC_ORIGIN_LAT_ERROR   : Origin latitude outside of valid range
 *                                      (-90 to 90 degrees)
 *       TRANMERC_CENT_MER_ERROR     : Central meridian outside of valid range
 *                                      (-180 to 360 degrees)
 *       TRANMERC_A_ERROR            : Semi-major axis less than or equal to zero
 *       TRANMERC_INV_F_ERROR        : Inverse flattening outside of valid range
 *								  	                  (250 to 350)
 *       TRANMERC_SCALE_FACTOR_ERROR : Scale factor outside of valid
 *                                     range (0.3 to 3.0)
 *		   TRANMERC_LON_WARNING        : Distortion will result if longitude is more
 *                                     than 9 degrees from the Central Meridian
 *
 * REUSE NOTES
 *
 *    TRANSVERSE MERCATOR is intended for reuse by any application that 
 *    performs a Transverse Mercator projection or its inverse.
 *    
 * REFERENCES
 *
 *    Further information on TRANSVERSE MERCATOR can be found in the 
 *    Reuse Manual.
 *
 *    TRANSVERSE MERCATOR originated from :  
 *                      U.S. Army Topographic Engineering Center
 *                      Geospatial Information Division
 *                      7701 Telegraph Road
 *                      Alexandria, VA  22310-3864
 *
 * LICENSES
 *
 *    None apply to this component.
 *
 * RESTRICTIONS
 *
 *    TRANSVERSE MERCATOR has no restrictions.
 *
 * ENVIRONMENT
 *
 *    TRANSVERSE MERCATOR was tested and certified in the following 
 *    environments:
 *
 *    1. Solaris 2.5 with GCC, version 2.8.1
 *    2. Windows 95 with MS Visual C++, version 6
 *
 * MODIFICATIONS
 *
 *    Date              Description
 *    ----              -----------
 *    2-26-07           Original C++ Code
 *
 */


#include "DtccApi.h"
#include "CoordinateSystem.h"


namespace MSP
{
  namespace CCS
  {
    class MapProjection5Parameters;
    class MapProjectionCoordinates;
    class GeodeticCoordinates;


    /***************************************************************************/
    /*
     *                              DEFINES
     */

    class MSP_DTCC_API TransverseMercator : public CoordinateSystem
    {
    public:

      /*
       * The constructor receives the ellipsoid
       * parameters and Tranverse Mercator projection parameters as inputs, and
       * sets the corresponding state variables. If any errors occur, an exception 
       * is thrown with a description of the error.
       *
       *    ellipsoidSemiMajorAxis     : Semi-major axis of ellipsoid, in meters    (input)
       *    ellipsoidFlattening        : Flattening of ellipsoid						        (input)
       *    centralMeridian            : Longitude in radians at the center of the  (input)
       *                                 projection
       *    latitudeOfTrueScale        : Latitude in radians at the origin of the   (input)
       *                                 projection
       *    falseEasting               : Easting/X at the center of the projection  (input)
       *    falseNorthing              : Northing/Y at the center of the projection (input)
       *    scaleFactor                : Projection scale factor                    (input) 
       */

	    TransverseMercator( double ellipsoidSemiMajorAxis, double ellipsoidFlattening, double centralMeridian, double latitudeOfTrueScale, double falseEasting, double falseNorthing, double scaleFactor );


      TransverseMercator( const TransverseMercator &tm );


	    ~TransverseMercator( void );


      TransverseMercator& operator=( const TransverseMercator &tm );


      /*
       * The function getParameters returns the current
       * ellipsoid and Transverse Mercator projection parameters.
       *
       *    ellipsoidSemiMajorAxis     : Semi-major axis of ellipsoid, in meters    (output)
       *    ellipsoidFlattening        : Flattening of ellipsoid						        (output)
       *    centralMeridian            : Longitude in radians at the center of the  (output)
       *                                 projection
       *    latitudeOfTrueScale        : Latitude in radians at the origin of the   (output)
       *                                 projection
       *    falseEasting               : Easting/X at the center of the projection  (output)
       *    falseNorthing              : Northing/Y at the center of the projection (output)
       *    scaleFactor                : Projection scale factor                    (output) 
       */

      MapProjection5Parameters* getParameters() const;


      /*
       * The function convertFromGeodetic converts geodetic
       * (latitude and longitude) coordinates to Transverse Mercator projection
       * (easting and northing) coordinates, according to the current ellipsoid
       * and Transverse Mercator projection coordinates.  If any errors occur, 
       * an exception is thrown with a description of the error.
       *
       *    longitude     : Longitude in radians                        (input)
       *    latitude      : Latitude in radians                         (input)
       *    easting       : Easting/X in meters                         (output)
       *    northing      : Northing/Y in meters                        (output)
       */

      MSP::CCS::MapProjectionCoordinates* convertFromGeodetic( MSP::CCS::GeodeticCoordinates* geodeticCoordinates );


      /*
       * The function convertToGeodetic converts Transverse
       * Mercator projection (easting and northing) coordinates to geodetic
       * (latitude and longitude) coordinates, according to the current ellipsoid
       * and Transverse Mercator projection parameters.  If any errors occur, 
       * an exception is thrown with a description of the error.
       *
       *    easting       : Easting/X in meters                         (input)
       *    northing      : Northing/Y in meters                        (input)
       *    longitude     : Longitude in radians                        (output)
       *    latitude      : Latitude in radians                         (output)
       */

      MSP::CCS::GeodeticCoordinates* convertToGeodetic( MSP::CCS::MapProjectionCoordinates* mapProjectionCoordinates );

    private:
    
      /* Ellipsoid Parameters */
      double TranMerc_es;             /* Eccentricity squared */
      double TranMerc_ebs;            /* Second Eccentricity squared */

      /* Transverse_Mercator projection Parameters */
      double TranMerc_Origin_Lat;           /* Latitude of origin in radians */
      double TranMerc_Origin_Long;          /* Longitude of origin in radians */
      double TranMerc_False_Northing;       /* False northing in meters */
      double TranMerc_False_Easting;        /* False easting in meters */
      double TranMerc_Scale_Factor;         /* Scale factor  */

      /* Isometric to geodetic latitude parameters */
      double TranMerc_ap;
      double TranMerc_bp;
      double TranMerc_cp;
      double TranMerc_dp;
      double TranMerc_ep;

      /* Maximum variance for easting and northing values */
      double TranMerc_Delta_Easting;
      double TranMerc_Delta_Northing;


      double sphtmd( double latitude, double sinLat, double cosLat );
      double sphsn( double sinLat );
      double sphsr( double sinLat );
    };
  }
}

#endif 


// CLASSIFICATION: UNCLASSIFIED
