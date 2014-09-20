// CLASSIFICATION: UNCLASSIFIED

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
 *       TRANMERC_NO_ERROR         : No errors occurred in function
 *       TRANMERC_LAT_ERROR        : Latitude outside of valid range
 *                                    (-90 to 90 degrees)
 *       TRANMERC_LON_ERROR        : Longitude outside of valid range
 *                                    (-180 to 360 degrees, and within
 *                                      +/-90 of Central Meridian)
 *       TRANMERC_EASTING_ERROR    : Easting outside of valid range
 *                                    (depending on ellipsoid and
 *                                     projection parameters)
 *       TRANMERC_NORTHING_ERROR   : Northing outside of valid range
 *                                    (depending on ellipsoid and
 *                                     projection parameters)
 *       TRANMERC_ORIGIN_LAT_ERROR : Origin latitude outside of valid range
 *                                    (-90 to 90 degrees)
 *       TRANMERC_CENT_MER_ERROR   : Central meridian outside of valid range
 *                                    (-180 to 360 degrees)
 *       TRANMERC_A_ERROR          : Semi-major axis less than or equal to zero
 *       TRANMERC_INV_F_ERROR      : Inverse flattening outside of valid range
 *                                    (250 to 350)
 *       TRANMERC_SCALE_FACTOR_ERROR : Scale factor outside of valid
 *                                     range (0.3 to 3.0)
 *	 TRANMERC_LON_WARNING      : Distortion will result if longitude is more
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
 *    Date     Description
 *    ----     -----------
 *    2-26-07  Original C++ Code
 *    7/19/10  N. Lundgren BAEts27271 Correct the test for TRANMERC_LON_WARNING
 *             in convertToGeodetic, by removing the multiply by cos(latitude)
 *    3/23/11  N. Lundgren BAEts28583 Updated memory leak checks for 
 *             code consistency.
 *
 */
/***************************************************************************/
/*
 *                               INCLUDES
 */

#include <math.h>
#include "TransverseMercator.h"
#include "MapProjection5Parameters.h"
#include "MapProjectionCoordinates.h"
#include "GeodeticCoordinates.h"
#include "CoordinateConversionException.h"
#include "ErrorMessages.h"
#include "WarningMessages.h"

/*
 *    math.h      - Standard C++ math library
 *    TransverseMercator.h  - Is for prototype error checking
 *    MapProjectionCoordinates.h   - defines map projection coordinates
 *    GeodeticCoordinates.h   - defines geodetic coordinates
 *    CoordinateConversionException.h - Exception handler
 *    ErrorMessages.h  - Contains exception messages
 */

using namespace MSP::CCS;

/***************************************************************************/
/*                               DEFINES 
 *
 */
// const double PI = 3.14159265358979323e0;      /* PI     */
// const double PI_OVER_2 = (PI/2.0e0);          /* PI over 2 */
// const double MAX_LAT = ((PI * 89.99)/180.0);  /* 89.99 degrees in radians */
// const double MAX_DELTA_LONG = ((PI * 90)/180.0); /* 90 degrees in radians */
// const double MIN_SCALE_FACTOR = 0.3;
// const double MAX_SCALE_FACTOR = 3.0;

#define PI                3.14159265358979323e0
#define PI_OVER_2         (PI/2.0e0)
#define MAX_LAT           ((PI * 89.99)/180.0)
#define MAX_DELTA_LONG    ((PI * 90)/180.0)
#define MIN_SCALE_FACTOR  0.3
#define MAX_SCALE_FACTOR  3.0

/************************************************************************/
/*                              FUNCTIONS     
 *
 */

TransverseMercator::TransverseMercator(
   double ellipsoidSemiMajorAxis,
   double ellipsoidFlattening,
   double centralMeridian,
   double latitudeOfTrueScale,
   double falseEasting,
   double falseNorthing,
   double scaleFactor ) :
  CoordinateSystem(),
  TranMerc_es( 0.0066943799901413800 ),
  TranMerc_ebs( 0.0067394967565869 ),
  TranMerc_Origin_Long( 0.0 ),
  TranMerc_Origin_Lat( 0.0 ),
  TranMerc_False_Easting( 0.0 ),
  TranMerc_False_Northing( 0.0 ),
  TranMerc_Scale_Factor( 1.0 ),
  TranMerc_ap( 6367449.1458008 ),
  TranMerc_bp( 16038.508696861 ),
  TranMerc_cp( 16.832613334334 ),
  TranMerc_dp( 0.021984404273757 ),
  TranMerc_ep( 3.1148371319283e-005 ),
  TranMerc_Delta_Easting( 40000000.0 ),
  TranMerc_Delta_Northing( 40000000.0 )
{
/*
 * The constructor receives the ellipsoid
 * parameters and Tranverse Mercator projection parameters as inputs, and
 * sets the corresponding state variables. If any errors occur, an exception 
 * is thrown with a description of the error.
 *
 * ellipsoidSemiMajorAxis  : Semi-major axis of ellipsoid, in meters    (input)
 * ellipsoidFlattening     : Flattening of ellipsoid		        (input)
 * centralMeridian         : Longitude in radians at the center of the  (input)
 *                              projection
 * latitudeOfTrueScale     : Latitude in radians at the origin of the   (input)
 *                              projection
 * falseEasting            : Easting/X at the center of the projection  (input)
 * falseNorthing           : Northing/Y at the center of the projection (input)
 * scaleFactor             : Projection scale factor                    (input)
 */

  double tn;         /* True Meridianal distance constant  */
  double tn2;
  double tn3;
  double tn4;
  double tn5;
  double TranMerc_b; /* Semi-minor axis of ellipsoid, in meters */
  double inv_f = 1 / ellipsoidFlattening;

  if (ellipsoidSemiMajorAxis <= 0.0)
  { /* Semi-major axis must be greater than zero */
    throw CoordinateConversionException( ErrorMessages::semiMajorAxis );
  }
  if ((inv_f < 250) || (inv_f > 350))
  { /* Inverse flattening must be between 250 and 350 */
    throw CoordinateConversionException( ErrorMessages::ellipsoidFlattening );
  }
  if ((latitudeOfTrueScale < -PI_OVER_2) || (latitudeOfTrueScale > PI_OVER_2))
  { /* latitudeOfTrueScale out of range */
    throw CoordinateConversionException( ErrorMessages::originLatitude );
  }
  if ((centralMeridian < -PI) || (centralMeridian > (2*PI)))
  { /* centralMeridian out of range */
    throw CoordinateConversionException( ErrorMessages::centralMeridian );
  }
  if ((scaleFactor < MIN_SCALE_FACTOR) || (scaleFactor > MAX_SCALE_FACTOR))
  {
    throw CoordinateConversionException( ErrorMessages::scaleFactor );
  }


  semiMajorAxis = ellipsoidSemiMajorAxis;
  flattening = ellipsoidFlattening;

  TranMerc_Origin_Lat     = latitudeOfTrueScale;
  if (centralMeridian > PI)
    centralMeridian -= (2*PI);
  TranMerc_Origin_Long    = centralMeridian;
  TranMerc_False_Northing = falseNorthing;
  TranMerc_False_Easting  = falseEasting; 
  TranMerc_Scale_Factor   = scaleFactor;

  /* Eccentricity Squared */
  TranMerc_es = 2 * flattening - flattening * flattening;
  /* Second Eccentricity Squared */
  TranMerc_ebs = (1 / (1 - TranMerc_es)) - 1;

  TranMerc_b = semiMajorAxis * (1 - flattening);    
  /*True meridianal constants  */
  tn = (semiMajorAxis - TranMerc_b) / (semiMajorAxis + TranMerc_b);
  tn2 = tn * tn;
  tn3 = tn2 * tn;
  tn4 = tn3 * tn;
  tn5 = tn4 * tn;

  TranMerc_ap = semiMajorAxis * 
     (1.e0 - tn + 5.e0 * (tn2 - tn3)/4.e0 + 81.e0 * (tn4 - tn5)/64.e0 );
  TranMerc_bp = 3.e0 * semiMajorAxis * 
     (tn - tn2 + 7.e0 * (tn3 - tn4) /8.e0 + 55.e0 * tn5/64.e0 ) / 2.e0;
  TranMerc_cp = 15.e0 * semiMajorAxis *
     (tn2 - tn3 + 3.e0 * (tn4 - tn5 )/4.e0) /16.0;
  TranMerc_dp = 35.e0 * semiMajorAxis *
     (tn3 - tn4 + 11.e0 * tn5 / 16.e0) / 48.e0;
  TranMerc_ep = 315.e0 * semiMajorAxis * (tn4 - tn5) / 512.e0;

  MapProjectionCoordinates* deltaCoordinates = 0;
  try
  {
     GeodeticCoordinates gcDeltaNorthing(
        CoordinateType::geodetic,
        MAX_DELTA_LONG + TranMerc_Origin_Long, MAX_LAT );
     deltaCoordinates = convertFromGeodetic( &gcDeltaNorthing );
     TranMerc_Delta_Northing = deltaCoordinates->northing();
     TranMerc_Delta_Northing ++;

     delete deltaCoordinates;
     deltaCoordinates = 0;

     GeodeticCoordinates gcDeltaEasting(
        CoordinateType::geodetic, MAX_DELTA_LONG + TranMerc_Origin_Long, 0 );
     deltaCoordinates = convertFromGeodetic( &gcDeltaEasting );
     TranMerc_Delta_Easting = deltaCoordinates->easting();
     TranMerc_Delta_Easting++;

     delete deltaCoordinates;
  }
  catch(CoordinateConversionException e)
  {
     delete deltaCoordinates;

     TranMerc_Delta_Easting  = 40000000.0;
     TranMerc_Delta_Northing = 40000000.0;
  }
}


TransverseMercator::TransverseMercator( const TransverseMercator &tm )
{
  semiMajorAxis           = tm.semiMajorAxis;
  flattening              = tm.flattening;
  TranMerc_es             = tm.TranMerc_es;     
  TranMerc_ebs            = tm.TranMerc_ebs; 
  TranMerc_Origin_Long    = tm.TranMerc_Origin_Long; 
  TranMerc_Origin_Lat     = tm.TranMerc_Origin_Lat; 
  TranMerc_False_Easting  = tm.TranMerc_False_Easting; 
  TranMerc_False_Northing = tm.TranMerc_False_Northing; 
  TranMerc_Scale_Factor   = tm.TranMerc_Scale_Factor; 
  TranMerc_ap             = tm.TranMerc_ap; 
  TranMerc_bp             = tm.TranMerc_bp; 
  TranMerc_cp             = tm.TranMerc_cp; 
  TranMerc_dp             = tm.TranMerc_dp; 
  TranMerc_ep             = tm.TranMerc_ep; 
  TranMerc_Delta_Easting  = tm.TranMerc_Delta_Easting; 
  TranMerc_Delta_Northing = tm.TranMerc_Delta_Northing; 
}


TransverseMercator::~TransverseMercator()
{
}


TransverseMercator& TransverseMercator::operator=( const TransverseMercator &tm )
{
  if( this != &tm )
  {
    semiMajorAxis           = tm.semiMajorAxis;
    flattening              = tm.flattening;
    TranMerc_es             = tm.TranMerc_es;     
    TranMerc_ebs            = tm.TranMerc_ebs; 
    TranMerc_Origin_Long    = tm.TranMerc_Origin_Long; 
    TranMerc_Origin_Lat     = tm.TranMerc_Origin_Lat; 
    TranMerc_False_Easting  = tm.TranMerc_False_Easting; 
    TranMerc_False_Northing = tm.TranMerc_False_Northing; 
    TranMerc_Scale_Factor   = tm.TranMerc_Scale_Factor; 
    TranMerc_ap             = tm.TranMerc_ap; 
    TranMerc_bp             = tm.TranMerc_bp; 
    TranMerc_cp             = tm.TranMerc_cp; 
    TranMerc_dp             = tm.TranMerc_dp; 
    TranMerc_ep             = tm.TranMerc_ep; 
    TranMerc_Delta_Easting  = tm.TranMerc_Delta_Easting; 
    TranMerc_Delta_Northing = tm.TranMerc_Delta_Northing; 
  }

  return *this;
}


MapProjection5Parameters* TransverseMercator::getParameters() const
{
/*
 * The function getParameters returns the current
 * ellipsoid and Transverse Mercator projection parameters.
 *
 * ellipsoidSemiMajorAxis : Semi-major axis of ellipsoid, in meters    (output)
 * ellipsoidFlattening    : Flattening of ellipsoid		       (output)
 * centralMeridian        : Longitude in radians at the center of the  (output)
 *                             projection
 * latitudeOfTrueScale    : Latitude in radians at the origin of the   (output)
 *                             projection
 * falseEasting           : Easting/X at the center of the projection  (output)
 * falseNorthing          : Northing/Y at the center of the projection (output)
 * scaleFactor            : Projection scale factor                    (output)
 */

  return new MapProjection5Parameters(
     CoordinateType::transverseMercator,
     TranMerc_Origin_Long, TranMerc_Origin_Lat, TranMerc_Scale_Factor,
     TranMerc_False_Easting, TranMerc_False_Northing );
}


MSP::CCS::MapProjectionCoordinates* TransverseMercator::convertFromGeodetic(
   MSP::CCS::GeodeticCoordinates* geodeticCoordinates )
{
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

  double c;       /* Cosine of latitude                          */
  double c2;
  double c3;
  double c5;
  double c7;
  double dlam;    /* Delta longitude - Difference in Longitude       */
  double eta;     /* constant - TranMerc_ebs *c *c                   */
  double eta2;
  double eta3;
  double eta4;
  double s;       /* Sine of latitude                        */
  double sn;      /* Radius of curvature in the prime vertical       */
  double t;       /* Tangent of latitude                             */
  double tan2;
  double tan3;
  double tan4;
  double tan5;
  double tan6;
  double t1;      /* Term in coordinate conversion formula - GP to Y */
  double t2;      /* Term in coordinate conversion formula - GP to Y */
  double t3;      /* Term in coordinate conversion formula - GP to Y */
  double t4;      /* Term in coordinate conversion formula - GP to Y */
  double t5;      /* Term in coordinate conversion formula - GP to Y */
  double t6;      /* Term in coordinate conversion formula - GP to Y */
  double t7;      /* Term in coordinate conversion formula - GP to Y */
  double t8;      /* Term in coordinate conversion formula - GP to Y */
  double t9;      /* Term in coordinate conversion formula - GP to Y */
  double tmd;     /* True Meridianal distance                        */
  double tmdo;    /* True Meridianal distance for latitude of origin */
  double temp_Origin;
  double temp_Long;

  double longitude = geodeticCoordinates->longitude();
  double latitude  = geodeticCoordinates->latitude();

  if ((latitude < -MAX_LAT) || (latitude > MAX_LAT))
  {  /* Latitude out of range */
    throw CoordinateConversionException( ErrorMessages::latitude );
  }

  if (longitude > PI)
    longitude -= (2 * PI);
  if ((longitude < (TranMerc_Origin_Long - MAX_DELTA_LONG))
      || (longitude > (TranMerc_Origin_Long + MAX_DELTA_LONG)))
  {
    if (longitude < 0)
      temp_Long = longitude + 2 * PI;
    else
      temp_Long = longitude;
    if (TranMerc_Origin_Long < 0)
      temp_Origin = TranMerc_Origin_Long + 2 * PI;
    else
      temp_Origin = TranMerc_Origin_Long;
    if ((temp_Long < (temp_Origin - MAX_DELTA_LONG))
        || (temp_Long > (temp_Origin + MAX_DELTA_LONG)))
       throw CoordinateConversionException( ErrorMessages::longitude );
  }

  /* 
   *  Delta Longitude
   */
  dlam = longitude - TranMerc_Origin_Long;


  char warning[256];
  warning[0] = '\0';
  if (fabs(dlam) > (9.0 * PI / 180))
  { /* Distortion results if Longitude is > 9 degrees from Central Meridian */
    strcat( warning, MSP::CCS::WarningMessages::longitude );
  }

  if (dlam > PI)
    dlam -= (2 * PI);
  if (dlam < -PI)
    dlam += (2 * PI);
  if (fabs(dlam) < 2.e-10)
    dlam = 0.0;

  s  = sin(latitude);
  c  = cos(latitude);
  c2 = c * c;
  c3 = c2 * c;
  c5 = c3 * c2;
  c7 = c5 * c2;
  t  = tan (latitude);
  tan2 = t * t;
  tan3 = tan2 * t;
  tan4 = tan3 * t;
  tan5 = tan4 * t;
  tan6 = tan5 * t;
  eta  = TranMerc_ebs * c2;
  eta2 = eta * eta;
  eta3 = eta2 * eta;
  eta4 = eta3 * eta;

  /* radius of curvature in prime vertical */
  sn  = sphsn(s);

  /* True Meridianal Distances */
  tmd = sphtmd(latitude, s, c);

  /*  Origin  */
  tmdo = sphtmd(
     TranMerc_Origin_Lat, sin(TranMerc_Origin_Lat), cos(TranMerc_Origin_Lat) );

  /* northing */
  t1 = (tmd - tmdo) * TranMerc_Scale_Factor;
  t2 = sn * s * c * TranMerc_Scale_Factor/ 2.e0;
  t3 = sn * s * c3 * TranMerc_Scale_Factor * 
     (5.e0 - tan2 + 9.e0 * eta + 4.e0 * eta2) /24.e0; 

  t4 = sn * s * c5 * TranMerc_Scale_Factor *
     (61.e0 - 58.e0 * tan2
        + tan4 + 270.e0 * eta - 330.e0 * tan2 * eta + 445.e0 * eta2
        + 324.e0 * eta3 -680.e0 * tan2 * eta2 + 88.e0 * eta4 
        -600.e0 * tan2 * eta3 - 192.e0 * tan2 * eta4) / 720.e0;

  t5 = sn * s * c7 * TranMerc_Scale_Factor *
     (1385.e0 - 3111.e0 * tan2 + 543.e0 * tan4 - tan6) / 40320.e0;

  double dlamSq = dlam * dlam;

  double northing = TranMerc_False_Northing + t1 + 
     (((t5 * dlamSq + t4) * dlamSq + t3) * dlamSq + t2) * dlamSq;

  /* Easting */
  t6 = sn * c  * TranMerc_Scale_Factor;
  t7 = sn * c3 * TranMerc_Scale_Factor * (1.e0 - tan2 + eta ) /6.e0;
  t8 = sn * c5 * TranMerc_Scale_Factor * (
     5.e0 - 18.e0 * tan2 + tan4 + 14.e0 * eta - 58.e0 * tan2 * eta +
     13.e0 * eta2 + 4.e0 * eta3 - 64.e0 * tan2 * eta2 - 24.e0 * tan2 * eta3 )
     / 120.e0;
  t9 = sn * c7 * TranMerc_Scale_Factor *
     ( 61.e0 - 479.e0 * tan2 + 179.e0 * tan4 - tan6 ) /5040.e0;

  double easting = TranMerc_False_Easting + 
     (((t9 * dlamSq + t8) * dlamSq + t7) * dlamSq + t6) * dlam;

  return new MapProjectionCoordinates(
     CoordinateType::transverseMercator, warning, easting, northing );
}


MSP::CCS::GeodeticCoordinates* TransverseMercator::convertToGeodetic(
   MSP::CCS::MapProjectionCoordinates* mapProjectionCoordinates )
{
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

  double c;       /* Cosine of latitude                          */
  double de;      /* Delta easting - Difference in Easting (easting-Fe)    */
  double dlam;    /* Delta longitude - Difference in Longitude       */
  double eta;     /* constant - TranMerc_ebs *c *c                   */
  double eta2;
  double eta3;
  double eta4;
  double ftphi;   /* Footpoint latitude                              */
  int    i;       /* Loop iterator                   */
  double s;       /* Sine of latitude                        */
  double sn;      /* Radius of curvature in the prime vertical       */
  double sr;      /* Radius of curvature in the meridian             */
  double t;       /* Tangent of latitude                             */
  double tan2;
  double tan4;
  double t10;     /* Term in coordinate conversion formula - GP to Y */
  double t11;     /* Term in coordinate conversion formula - GP to Y */
  double t12;     /* Term in coordinate conversion formula - GP to Y */
  double t13;     /* Term in coordinate conversion formula - GP to Y */
  double t14;     /* Term in coordinate conversion formula - GP to Y */
  double t15;     /* Term in coordinate conversion formula - GP to Y */
  double t16;     /* Term in coordinate conversion formula - GP to Y */
  double t17;     /* Term in coordinate conversion formula - GP to Y */
  double tmd;     /* True Meridianal distance                        */
  double tmdo;    /* True Meridianal distance for latitude of origin */

  double easting = mapProjectionCoordinates->easting();
  double northing = mapProjectionCoordinates->northing();

  if ((easting < (TranMerc_False_Easting - TranMerc_Delta_Easting))
      ||(easting > (TranMerc_False_Easting + TranMerc_Delta_Easting)))
  { /* easting out of range  */
    throw CoordinateConversionException( ErrorMessages::easting );
  }

  if ((northing < (TranMerc_False_Northing - TranMerc_Delta_Northing))
      || (northing > (TranMerc_False_Northing + TranMerc_Delta_Northing)))
  { /* northing out of range */
    throw CoordinateConversionException( ErrorMessages::northing );
  }

  /* True Meridianal Distances for latitude of origin */
  tmdo = sphtmd(
     TranMerc_Origin_Lat, sin(TranMerc_Origin_Lat), cos(TranMerc_Origin_Lat) );

  /*  Origin  */
  tmd = tmdo +  (northing - TranMerc_False_Northing) / TranMerc_Scale_Factor; 

  /* First Estimate */
  sr = sphsr(0.e0);
  ftphi = tmd/sr;

  for (i = 0; i < 5 ; i++)
  {
     s     = sin(ftphi);
     c     = cos(ftphi);
     t10   = sphtmd(ftphi, s, c);
     sr    = sphsr( s);
     ftphi = ftphi + (tmd - t10) / sr;
  }

  /* Sine Cosine terms */
  s = sin(ftphi);
  c = cos(ftphi);

  /* Radius of Curvature in the meridian */
  sr = sphsr(s);

  /* Radius of Curvature in the meridian */
  sn = sphsn(s);

  /* Tangent Value  */
  t = s/c; //tan(ftphi);
  tan2 = t * t;
  tan4 = tan2 * tan2;
  eta = TranMerc_ebs * c * c;
  eta2 = eta * eta;
  eta3 = eta2 * eta;
  eta4 = eta3 * eta;

  double sn2 = sn  * sn;
  double sn3 = sn2 * sn;
  double sn5 = sn2 * sn3;
  double sn7 = sn2 * sn5;

  de = easting - TranMerc_False_Easting;
  if (fabs(de) < 0.0001)
    de = 0.0;

  /* Latitude */
  double TranMerc_Scale_Factor2 =
     TranMerc_Scale_Factor * TranMerc_Scale_Factor;
  double TranMerc_Scale_Factor3 =
     TranMerc_Scale_Factor2 * TranMerc_Scale_Factor;
  double TranMerc_Scale_Factor4 = 
     TranMerc_Scale_Factor2 * TranMerc_Scale_Factor2;
  double TranMerc_Scale_Factor5 = 
     TranMerc_Scale_Factor3 * TranMerc_Scale_Factor2;
  double TranMerc_Scale_Factor6 = 
     TranMerc_Scale_Factor4 * TranMerc_Scale_Factor2;
  double TranMerc_Scale_Factor7 = 
     TranMerc_Scale_Factor6 * TranMerc_Scale_Factor;
  double TranMerc_Scale_Factor8 = 
     TranMerc_Scale_Factor6 * TranMerc_Scale_Factor2;

  t10 = t / (2.e0 * sr * sn * TranMerc_Scale_Factor2);

  t11 = t * (5.e0  + 3.e0 * tan2 + eta - 4.e0 * eta2 - 9.e0 * tan2 * eta) /
     (24.e0 * sr * sn3 * TranMerc_Scale_Factor4);

  t12 = t * (61.e0 + 90.e0 * tan2 + 46.e0 * eta + 45.E0 * tan4
             - 252.e0 * tan2 * eta  - 3.e0 * eta2 + 100.e0 
             * eta3 - 66.e0 * tan2 * eta2 - 90.e0 * tan4
             * eta + 88.e0 * eta4 + 225.e0 * tan4 * eta2
             + 84.e0 * tan2* eta3 - 192.e0 * tan2 * eta4)
        / ( 720.e0 * sr * sn5 * TranMerc_Scale_Factor6 );

  double t6 = t * t * t * t * t * t;
  t13 = t * ( 1385.e0 + 3633.e0 * tan2 + 4095.e0 * tan4 + 1575.e0 * t6 ) /
     (40320.e0 * sr * sn7 * TranMerc_Scale_Factor8);

  double de2 = de * de;
  double latitude = ftphi + 
     (((t13 * de2 - t12) * de2 + t11) * de2 - t10) *de2;


  t14 = 1.e0 / (sn * c * TranMerc_Scale_Factor);

  t15 = (1.e0 + 2.e0 * tan2 + eta) /
     (6.e0 * sn3 * c * TranMerc_Scale_Factor3);

  t16 = (5.e0 + 6.e0 * eta + 28.e0 * tan2 - 3.e0 * eta2
     + 8.e0 * tan2 * eta + 24.e0 * tan4 - 4.e0 
     * eta3 + 4.e0 * tan2 * eta2 + 24.e0 * tan2 * eta3) /
     (120.e0 * sn5 * c * TranMerc_Scale_Factor5);

  t17 = (61.e0 +  662.e0 * tan2 + 1320.e0 * tan4 + 720.e0 * t6 ) /
     (5040.e0 * sn7 * c * TranMerc_Scale_Factor7);

  /* Difference in Longitude */
  dlam = (((-t17 * de2 + t16) * de2 - t15) * de2 + t14) *de;

  /* Longitude */
  double longitude = TranMerc_Origin_Long + dlam;

  if(fabs(latitude) > (90.0 * PI / 180.0))
    throw CoordinateConversionException( ErrorMessages::northing );

  if((longitude) > (PI))
  {
     longitude -= (2 * PI);
     if(fabs(longitude) > PI)
        throw CoordinateConversionException( ErrorMessages::easting );
  }
  else if((longitude) < (-PI))
  {
     longitude += (2 * PI);
     if(fabs(longitude) > PI)
        throw CoordinateConversionException( ErrorMessages::easting );
  }

  char warning[256];
  warning[0] = '\0';
  if (fabs(dlam) > (9.0 * PI / 180))
  { /* Distortion will result if longitude is more than 9 degrees from
       the Central Meridian at the equator */
    /* and decreases to 0 degrees at the poles */
    /* As you move towards the poles, distortion becomes less significant */
     strcat( warning, MSP::CCS::WarningMessages::longitude );
  }

  return new GeodeticCoordinates(
     CoordinateType::geodetic, warning, longitude, latitude );
}


/************************************************************************/
/*                              PRIVATE FUNCTIONS     
 *
 */

double TransverseMercator::sphtmd(
   double latitude,
   double sinLat,
   double cosLat )
{
//   double sin2Lat = sin(2.e0 * latitude);
//   double cos2Lat = cos(2.e0 * latitude);
   double sin2Lat = 2.0 * sinLat * cosLat;
   double cos2Lat = 2.0 * cosLat * cosLat - 1.0;
   double sin2LatCubed = sin2Lat * sin2Lat * sin2Lat;
   double sin4Lat = 2.0 * sin2Lat * cos2Lat;
   double sin6Lat = 3.0 * sin2Lat - 4.0 * sin2LatCubed;
   double sin8Lat = cos2Lat * ( 4.0 * sin2Lat - 8.0 * sin2LatCubed);

   return TranMerc_ap * latitude 
      - TranMerc_bp * sin2Lat + TranMerc_cp * sin4Lat
      - TranMerc_dp * sin6Lat + TranMerc_ep * sin8Lat;

// return TranMerc_ap * latitude 
//    - TranMerc_bp * sin(2.e0 * latitude) + TranMerc_cp * sin(4.e0 * latitude) 
//    - TranMerc_dp * sin(6.e0 * latitude) + TranMerc_ep * sin(8.e0 * latitude);
}


double TransverseMercator::sphsn( double  sinLat ) 
{
//   double sinLat = sin(latitude);
   return semiMajorAxis / sqrt( 1.e0 - TranMerc_es * sinLat * sinLat);
}


double TransverseMercator::sphsr( double sinLat ) 
{
//   double sinLat = sin(latitude);
   double denom = sqrt(1.e0 - TranMerc_es * sinLat * sinLat);
   return semiMajorAxis * (1.e0 - TranMerc_es) / denom / denom / denom;
}



// CLASSIFICATION: UNCLASSIFIED
