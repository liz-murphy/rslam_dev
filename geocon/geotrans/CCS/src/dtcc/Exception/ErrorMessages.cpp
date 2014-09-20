// CLASSIFICATION: UNCLASSIFIED

#include "ErrorMessages.h"


using namespace MSP::CCS;


char* ErrorMessages::geoidFileOpenError = "Unable to locate geoid data file\n";
char* ErrorMessages::geoidFileParseError = "Unable to read geoid file\n";

char* ErrorMessages::ellipsoidFileOpenError = "Unable to locate ellipsoid data file: ellips.dat\n";
char* ErrorMessages::ellipsoidFileCloseError = "Unable to close ellipsoid file: ellips.dat\n";
char* ErrorMessages::ellipsoidFileParseError = "Unable to read ellipsoid file: ellips.dat\n";
char* ErrorMessages::ellipsoidOverflow = "Ellipsoid table overflow\n";
char* ErrorMessages::ellipse = "Ellipsoid library not initialized\n";
char* ErrorMessages::invalidEllipsoidCode = "Invalid ellipsoid code\n";

char* ErrorMessages::datumFileOpenError = "Unable to locate datum data file\n";
char* ErrorMessages::datumFileCloseError = "Unable to close datum file\n";
char* ErrorMessages::datumFileParseError = "Unable to read datum file\n";
char* ErrorMessages::datumDomain = "Invalid local datum domain of validity\n";
char* ErrorMessages::datumOverflow = "Datum table overflow";
char* ErrorMessages::datumRotation = "Rotation values must be between -60.0 and 60.0";
char* ErrorMessages::datumSigma = "Standard error values must be positive, or -1 if unknown\n";
char* ErrorMessages::datumType = "Invalid datum type\n";
char* ErrorMessages::invalidDatumCode = "Invalid datum code\n";

char* ErrorMessages::notUserDefined = "Specified code not user defined\n";
char* ErrorMessages::ellipseInUse = "Ellipsoid is in use by a datum\n";

// Parameter error messages
char* ErrorMessages::semiMajorAxis = "Ellipsoid semi-major axis must be greater than zero\n";
char* ErrorMessages::ellipsoidFlattening = "Inverse flattening must be between 250 and 350\n";
char* ErrorMessages::orientation = "Orientation out of range\n";
char* ErrorMessages::originLatitude = "Origin Latitude (or Standard Parallel or Latitude of True Scale) out of range\n";
char* ErrorMessages::originLongitude = "Origin Longitude (or Longitude Down from Pole) out of range\n";
char* ErrorMessages::centralMeridian = "Central Meridian out of range\n";
char* ErrorMessages::scaleFactor = "Scale Factor out of range\n";
char* ErrorMessages::zone = "Invalid Zone\n";
char* ErrorMessages::zoneOverride = "Invalid Zone Override\n";
char* ErrorMessages::standardParallel1 = "Invalid 1st Standard Parallel\n";
char* ErrorMessages::standardParallel2 = "Invalid 2nd Standard Parallel\n";
char* ErrorMessages::standardParallel1_2 = "1st & 2nd Standard Parallels cannot both be zero\n";
char* ErrorMessages::standardParallelHemisphere = "Standard Parallels cannot be equal and opposite latitudes\n";
char* ErrorMessages::precision = "Precision must be between 0 and 5\n";
char* ErrorMessages::bngEllipsoid = "British National Grid ellipsoid must be Airy\n";
char* ErrorMessages::nzmgEllipsoid = "New Zealand Map Grid ellipsoid must be International\n";
char* ErrorMessages::latitude1 = "Latitude 1 out of range\n";
char* ErrorMessages::latitude2 = "Latitude 2 out of range\n";
char* ErrorMessages::latitude1_2 = "Latitude 1 and Latitude 2 cannot be equal\n";
char* ErrorMessages::longitude1 = "Longitude 1 out of range\n";
char* ErrorMessages::longitude2 = "Longitude 2 out of range\n";
char* ErrorMessages::omercHemisphere = "Point 1 and Point 2 cannot be in different hemispheres\n";
char* ErrorMessages::hemisphere = "Invalid Hemisphere\n";
char* ErrorMessages::radius = "Easting/Northing too far from center of projection\n";


// Coordinate error messages
char* ErrorMessages::latitude = "Latitude out of range\n";
char* ErrorMessages::longitude = "Longitude out of range\n";
char* ErrorMessages::easting = "Easting/X out of range\n";
char* ErrorMessages::northing = "Northing/Y out of range\n";
char* ErrorMessages::projection = "Point projects into a circle\n";
char* ErrorMessages::invalidArea = "Coordinates are outside valid area\n";
char* ErrorMessages::bngString = "Invalid British National Grid String\n";
char* ErrorMessages::garsString = "Invalid GARS String\n";
char* ErrorMessages::georefString = "Invalid GEOREF String\n";
char* ErrorMessages::mgrsString = "Invalid MGRS String\n";
char* ErrorMessages::usngString = "Invalid USNG String\n";

char* ErrorMessages::invalidIndex = "Index value outside of valid range\n";
char* ErrorMessages::invalidName = "Invalid name\n";
char* ErrorMessages::invalidType = "Invalid coordinate system type\n";

char* ErrorMessages::longitude_min = "The longitude minute part of the string is greater than 60\n";
char* ErrorMessages::latitude_min = "The latitude minute part of the string is greater than 60\n";

// CLASSIFICATION: UNCLASSIFIED
