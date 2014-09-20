// CLASSIFICATION: UNCLASSIFIED

#include <jni.h>
#include "string.h"
#include "CoordinateConversionService.h"
#include "Accuracy.h"
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
#include "CoordinateSystemParameters.h"
#include "ThrowException.h"


using namespace MSP::CCS;


#ifdef __cplusplus
extern "C" {
#endif

CoordinateSystemParameters* translateFromJNIParameters( JNIEnv *env, jobject parameters )
{
    jclass geodeticParametersClazz = env->FindClass( "geotrans3/parameters/GeodeticParameters" );

    if( env->IsInstanceOf( parameters, geodeticParametersClazz ) )
    {
      jclass clazz = env->GetObjectClass( parameters );

      jfieldID fid = env->GetFieldID( clazz, "heightType", "I");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Height type field ID error." );
          return 0;
      }

      return new GeodeticParameters( CoordinateType::geodetic, ( HeightType::Enum )env->GetIntField( parameters, fid ) );
    }
    else
    {
      jclass equidistantCylindricalParametersClazz = env->FindClass( "geotrans3/parameters/EquidistantCylindricalParameters" );

      if( env->IsInstanceOf( parameters, equidistantCylindricalParametersClazz ) )
      {
        jclass clazz = env->GetObjectClass( parameters );

        // Central Meridian
        jfieldID fid = env->GetFieldID( clazz, "centralMeridian", "D");
        if (fid == NULL)
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
            return 0;
        }
        double centralMeridian = env->GetDoubleField( parameters, fid );

        // Standard Parallel
        fid = env->GetFieldID( clazz, "standardParallel", "D");
        if (fid == NULL)
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: standardParallel field ID error." );
            return 0;
        }
        double standardParallel = env->GetDoubleField( parameters, fid );

        // False Easting
        fid = env->GetFieldID( clazz, "falseEasting", "D");
        if (fid == NULL)
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
            return 0;
        }
        double falseEasting = env->GetDoubleField( parameters, fid );

        // False Northing
        fid = env->GetFieldID( clazz, "falseNorthing", "D");
        if (fid == NULL)
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
            return 0;
        }
        double falseNorthing = env->GetDoubleField( parameters, fid );

        return new EquidistantCylindricalParameters( CoordinateType::equidistantCylindrical, centralMeridian, standardParallel, falseEasting, falseNorthing );
      }
      else
      {
        jclass localCartesianParametersClazz = env->FindClass( "geotrans3/parameters/LocalCartesianParameters" );

        if( env->IsInstanceOf( parameters, localCartesianParametersClazz ) )
        {
          jclass clazz = env->GetObjectClass( parameters );

          // Longitude
          jfieldID fid = env->GetFieldID( clazz, "longitude", "D");
          if (fid == NULL)
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: longitude field ID error." );
              return 0;
          }
          double longitude = env->GetDoubleField( parameters, fid );

          // Latitude
          fid = env->GetFieldID( clazz, "latitude", "D");
          if (fid == NULL)
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: latitude field ID error." );
              return 0;
          }
          double latitude = env->GetDoubleField( parameters, fid );

          // Height
          fid = env->GetFieldID( clazz, "height", "D");
          if (fid == NULL)
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: height field ID error." );
              return 0;
          }
          double height = env->GetDoubleField( parameters, fid );

          // Orientation
          fid = env->GetFieldID( clazz, "orientation", "D");
          if (fid == NULL)
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: orientation field ID error." );
              return 0;
          }
          double orientation = env->GetDoubleField( parameters, fid );

          return new LocalCartesianParameters( CoordinateType::localCartesian, longitude, latitude, height, orientation );
        }
        else
        {
          jclass mapProjection3ParametersClazz = env->FindClass( "geotrans3/parameters/MapProjection3Parameters" );

          if( env->IsInstanceOf( parameters, mapProjection3ParametersClazz ) )
          {
            jclass clazz = env->GetObjectClass( parameters );

            // Coordinate Type
            jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
            if (fid == NULL)
            {
                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                return 0;
            }
            CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

            // Central Meridian
            fid = env->GetFieldID( clazz, "centralMeridian", "D");
            if (fid == NULL)
            {
                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                return 0;
            }
            double centralMeridian = env->GetDoubleField( parameters, fid );

            // False Easting
            fid = env->GetFieldID( clazz, "falseEasting", "D");
            if (fid == NULL)
            {
                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                return 0;
            }
            double falseEasting = env->GetDoubleField( parameters, fid );

            // False Northing
            fid = env->GetFieldID( clazz, "falseNorthing", "D");
            if (fid == NULL)
            {
                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                return 0;
            }
            double falseNorthing = env->GetDoubleField( parameters, fid );

            return new MapProjection3Parameters( coordinateType, centralMeridian, falseEasting, falseNorthing );
          }
          else
          {
            jclass mapProjection4ParametersClazz = env->FindClass( "geotrans3/parameters/MapProjection4Parameters" );

            if( env->IsInstanceOf( parameters, mapProjection4ParametersClazz ) )
            {
              jclass clazz = env->GetObjectClass( parameters );

              // Coordinate Type
              jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                  return 0;
              }
              CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

              // Central Meridian
              fid = env->GetFieldID( clazz, "centralMeridian", "D");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                  return 0;
              }
              double centralMeridian = env->GetDoubleField( parameters, fid );

              // Origin Latitude
              fid = env->GetFieldID( clazz, "originLatitude", "D");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: originLatitude field ID error." );
                  return 0;
              }
              double originLatitude = env->GetDoubleField( parameters, fid );

              // False Easting
              fid = env->GetFieldID( clazz, "falseEasting", "D");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                  return 0;
              }
              double falseEasting = env->GetDoubleField( parameters, fid );

              // False Northing
              fid = env->GetFieldID( clazz, "falseNorthing", "D");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                  return 0;
              }
              double falseNorthing = env->GetDoubleField( parameters, fid );

              return new MapProjection4Parameters( coordinateType, centralMeridian, originLatitude, falseEasting, falseNorthing );
            }
            else
            {
              jclass mapProjection5ParametersClazz = env->FindClass( "geotrans3/parameters/MapProjection5Parameters" );

              if( env->IsInstanceOf( parameters, mapProjection5ParametersClazz ) )
              {
                jclass clazz = env->GetObjectClass( parameters );

                // Coordinate Type
                jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                if (fid == NULL)
                {
                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                    return 0;
                }
                CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                // Central Meridian
                fid = env->GetFieldID( clazz, "centralMeridian", "D");
                if (fid == NULL)
                {
                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                    return 0;
                }
                double centralMeridian = env->GetDoubleField( parameters, fid );

                // Origin Latitude
                fid = env->GetFieldID( clazz, "originLatitude", "D");
                if (fid == NULL)
                {
                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: originLatitude field ID error." );
                    return 0;
                }
                double originLatitude = env->GetDoubleField( parameters, fid );

                // Scale Factor
                fid = env->GetFieldID( clazz, "scaleFactor", "D");
                if (fid == NULL)
                {
                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: scaleFactor field ID error." );
                    return 0;
                }
                double scaleFactor = env->GetDoubleField( parameters, fid );

                // False Easting
                fid = env->GetFieldID( clazz, "falseEasting", "D");
                if (fid == NULL)
                {
                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                    return 0;
                }
                double falseEasting = env->GetDoubleField( parameters, fid );

                // False Northing
                fid = env->GetFieldID( clazz, "falseNorthing", "D");
                if (fid == NULL)
                {
                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                    return 0;
                }
                double falseNorthing = env->GetDoubleField( parameters, fid );

                return new MapProjection5Parameters( coordinateType, centralMeridian, originLatitude, scaleFactor, falseEasting, falseNorthing );
              }
              else
              {
                jclass mapProjection6ParametersClazz = env->FindClass( "geotrans3/parameters/MapProjection6Parameters" );

                if( env->IsInstanceOf( parameters, mapProjection6ParametersClazz ) )
                {
                  jclass clazz = env->GetObjectClass( parameters );

                  // Coordinate Type
                  jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                  if (fid == NULL)
                  {
                      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                      return 0;
                  }
                  CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                  // Central Meridian
                  fid = env->GetFieldID( clazz, "centralMeridian", "D");
                  if (fid == NULL)
                  {
                      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                      return 0;
                  }
                  double centralMeridian = env->GetDoubleField( parameters, fid );

                  // Origin Latitude
                  fid = env->GetFieldID( clazz, "originLatitude", "D");
                  if (fid == NULL)
                  {
                      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: originLatitude field ID error." );
                      return 0;
                  }
                  double originLatitude = env->GetDoubleField( parameters, fid );

                  // Standard Parallel 1
                  fid = env->GetFieldID( clazz, "standardParallel1", "D");
                  if (fid == NULL)
                  {
                      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: standardParallel1 field ID error." );
                      return 0;
                  }
                  double standardParallel1 = env->GetDoubleField( parameters, fid );

                  // Standard Parallel 2
                  fid = env->GetFieldID( clazz, "standardParallel2", "D");
                  if (fid == NULL)
                  {
                      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: standardParallel2 field ID error." );
                      return 0;
                  }
                  double standardParallel2 = env->GetDoubleField( parameters, fid );

                  // False Easting
                  fid = env->GetFieldID( clazz, "falseEasting", "D");
                  if (fid == NULL)
                  {
                      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                      return 0;
                  }
                  double falseEasting = env->GetDoubleField( parameters, fid );

                  // False Northing
                  fid = env->GetFieldID( clazz, "falseNorthing", "D");
                  if (fid == NULL)
                  {
                      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                      return 0;
                  }
                  double falseNorthing = env->GetDoubleField( parameters, fid );

                  return new MapProjection6Parameters( coordinateType, centralMeridian, originLatitude, standardParallel1, standardParallel2, falseEasting, falseNorthing );
                }
                else
                {
                  jclass mercatorStandardParallelParametersClazz = env->FindClass( "geotrans3/parameters/MercatorStandardParallelParameters" );

                  if( env->IsInstanceOf( parameters, mercatorStandardParallelParametersClazz ) )
                  {
                    jclass clazz = env->GetObjectClass( parameters );

                    // Coordinate Type
                    jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                    if (fid == NULL)
                    {
                        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                        return 0;
                    }
                    CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                    // Central Meridian
                    fid = env->GetFieldID( clazz, "centralMeridian", "D");
                    if (fid == NULL)
                    {
                        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                        return 0;
                    }
                    double centralMeridian = env->GetDoubleField( parameters, fid );

                    // Latitude Of True Scale
                    fid = env->GetFieldID( clazz, "standardParallel", "D");
                    if (fid == NULL)
                    {
                        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: standardParallel field ID error." );
                        return 0;
                    }
                    double standardParallel = env->GetDoubleField( parameters, fid );

                    // Scale Factor
                    fid = env->GetFieldID( clazz, "scaleFactor", "D");
                    if (fid == NULL)
                    {
                        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: scaleFactor field ID error." );
                        return 0;
                    }
                    double scaleFactor = env->GetDoubleField( parameters, fid );

                    // False Easting
                    fid = env->GetFieldID( clazz, "falseEasting", "D");
                    if (fid == NULL)
                    {
                        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                        return 0;
                    }
                    double falseEasting = env->GetDoubleField( parameters, fid );

                    // False Northing
                    fid = env->GetFieldID( clazz, "falseNorthing", "D");
                    if (fid == NULL)
                    {
                        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                        return 0;
                    }
                    double falseNorthing = env->GetDoubleField( parameters, fid );

                    return new MercatorStandardParallelParameters( coordinateType, centralMeridian, standardParallel, scaleFactor, falseEasting, falseNorthing );
                  }
                  else
                  {
                    jclass mercatorScaleFactorParametersClazz = env->FindClass( "geotrans3/parameters/MercatorScaleFactorParameters" );

                    if( env->IsInstanceOf( parameters, mercatorScaleFactorParametersClazz ) )
                    {
                      jclass clazz = env->GetObjectClass( parameters );

                      // Coordinate Type
                      jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                      if (fid == NULL)
                      {
                          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                          return 0;
                      }
                      CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                      // Central Meridian
                      fid = env->GetFieldID( clazz, "centralMeridian", "D");
                      if (fid == NULL)
                      {
                          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                          return 0;
                      }
                      double centralMeridian = env->GetDoubleField( parameters, fid );

                      // Scale Factor
                      fid = env->GetFieldID( clazz, "scaleFactor", "D");
                      if (fid == NULL)
                      {
                          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: scaleFactor field ID error." );
                          return 0;
                      }
                      double scaleFactor = env->GetDoubleField( parameters, fid );

                      // False Easting
                      fid = env->GetFieldID( clazz, "falseEasting", "D");
                      if (fid == NULL)
                      {
                          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                          return 0;
                      }
                      double falseEasting = env->GetDoubleField( parameters, fid );

                      // False Northing
                      fid = env->GetFieldID( clazz, "falseNorthing", "D");
                      if (fid == NULL)
                      {
                          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                          return 0;
                      }
                      double falseNorthing = env->GetDoubleField( parameters, fid );

                      return new MercatorScaleFactorParameters( coordinateType, centralMeridian, scaleFactor, falseEasting, falseNorthing );
                    }
                    else
                    {
                      jclass neysParametersClazz = env->FindClass( "geotrans3/parameters/NeysParameters" );

                      if( env->IsInstanceOf( parameters, neysParametersClazz ) )
                      {
                        jclass clazz = env->GetObjectClass( parameters );

                        // Coordinate Type
                        jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                        if (fid == NULL)
                        {
                            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                            return 0;
                        }
                        CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                        // Central Meridian
                        fid = env->GetFieldID( clazz, "centralMeridian", "D");
                        if (fid == NULL)
                        {
                            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                            return 0;
                        }
                        double centralMeridian = env->GetDoubleField( parameters, fid );

                        // Origin Latitude
                        fid = env->GetFieldID( clazz, "originLatitude", "D");
                        if (fid == NULL)
                        {
                            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: originLatitude field ID error." );
                            return 0;
                        }
                        double originLatitude = env->GetDoubleField( parameters, fid );

                        // Standard Parallel 1
                        fid = env->GetFieldID( clazz, "standardParallel1", "D");
                        if (fid == NULL)
                        {
                            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: standardParallel1 field ID error." );
                            return 0;
                        }
                        double standardParallel1 = env->GetDoubleField( parameters, fid );

                        // False Easting
                        fid = env->GetFieldID( clazz, "falseEasting", "D");
                        if (fid == NULL)
                        {
                            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                            return 0;
                        }
                        double falseEasting = env->GetDoubleField( parameters, fid );

                        // False Northing
                        fid = env->GetFieldID( clazz, "falseNorthing", "D");
                        if (fid == NULL)
                        {
                            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                            return 0;
                        }
                        double falseNorthing = env->GetDoubleField( parameters, fid );

                        return new NeysParameters( coordinateType, centralMeridian, originLatitude, standardParallel1, falseEasting, falseNorthing );
                      }
                      else
                      {
                        jclass obliqueMercatorParametersClazz = env->FindClass( "geotrans3/parameters/ObliqueMercatorParameters" );

                        if( env->IsInstanceOf( parameters, obliqueMercatorParametersClazz ) )
                        {
                          jclass clazz = env->GetObjectClass( parameters );

                          // Coordinate Type
                          jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                              return 0;
                          }
                          CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                          // Origin Latitude
                          fid = env->GetFieldID( clazz, "originLatitude", "D");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: originLatitude field ID error." );
                              return 0;
                          }
                          double originLatitude = env->GetDoubleField( parameters, fid );

                          // Longitude 1
                          fid = env->GetFieldID( clazz, "longitude1", "D");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: longitude1 field ID error." );
                              return 0;
                          }
                          double longitude1 = env->GetDoubleField( parameters, fid );

                          // Latitude 1
                          fid = env->GetFieldID( clazz, "latitude1", "D");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: latitude1 field ID error." );
                              return 0;
                          }
                          double latitude1 = env->GetDoubleField( parameters, fid );

                          // Longitude 2
                          fid = env->GetFieldID( clazz, "longitude2", "D");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: longitude2 field ID error." );
                              return 0;
                          }
                          double longitude2 = env->GetDoubleField( parameters, fid );

                          // Latitude 2
                          fid = env->GetFieldID( clazz, "latitude2", "D");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: latitude2 field ID error." );
                              return 0;
                          }
                          double latitude2 = env->GetDoubleField( parameters, fid );

                          // False Easting
                          fid = env->GetFieldID( clazz, "falseEasting", "D");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                              return 0;
                          }
                          double falseEasting = env->GetDoubleField( parameters, fid );

                          // False Northing
                          fid = env->GetFieldID( clazz, "falseNorthing", "D");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                              return 0;
                          }
                          double falseNorthing = env->GetDoubleField( parameters, fid );

                          // Scale Factor
                          fid = env->GetFieldID( clazz, "scaleFactor", "D");
                          if (fid == NULL)
                          {
                              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: scaleFactor field ID error." );
                              return 0;
                          }
                          double scaleFactor = env->GetDoubleField( parameters, fid );

                          return new ObliqueMercatorParameters( coordinateType, originLatitude, longitude1, latitude1, longitude2, latitude2, falseEasting, falseNorthing, scaleFactor );
                        }
                        else
                        {
                          jclass polarStereographicStandardParallelParametersClazz = env->FindClass( "geotrans3/parameters/PolarStereographicStandardParallelParameters" );

                          if( env->IsInstanceOf( parameters, polarStereographicStandardParallelParametersClazz ) )
                          {
                            jclass clazz = env->GetObjectClass( parameters );

                            // Coordinate Type
                            jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                            if (fid == NULL)
                            {
                                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                                return 0;
                            }
                            CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                            // Central Meridian
                            fid = env->GetFieldID( clazz, "centralMeridian", "D");
                            if (fid == NULL)
                            {
                                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                                return 0;
                            }
                            double centralMeridian = env->GetDoubleField( parameters, fid );

                            // Standard Parallel
                            fid = env->GetFieldID( clazz, "standardParallel", "D");
                            if (fid == NULL)
                            {
                                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: standardParallel field ID error." );
                                return 0;
                            }
                            double standardParallel = env->GetDoubleField( parameters, fid );

                            // False Easting
                            fid = env->GetFieldID( clazz, "falseEasting", "D");
                            if (fid == NULL)
                            {
                                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                                return 0;
                            }
                            double falseEasting = env->GetDoubleField( parameters, fid );

                            // False Northing
                            fid = env->GetFieldID( clazz, "falseNorthing", "D");
                            if (fid == NULL)
                            {
                                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                                return 0;
                            }
                            double falseNorthing = env->GetDoubleField( parameters, fid );

                            return new PolarStereographicStandardParallelParameters( coordinateType, centralMeridian, standardParallel, falseEasting, falseNorthing );
                          }
                          else
                          {
                            jclass polarStereographicScaleFactorParametersClazz = env->FindClass( "geotrans3/parameters/PolarStereographicScaleFactorParameters" );

                            if( env->IsInstanceOf( parameters, polarStereographicScaleFactorParametersClazz ) )
                            {
                              jclass clazz = env->GetObjectClass( parameters );

                              // Coordinate Type
                              jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                              if (fid == NULL)
                              {
                                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                                  return 0;
                              }
                              CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                              // Central Meridian
                              fid = env->GetFieldID( clazz, "centralMeridian", "D");
                              if (fid == NULL)
                              {
                                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: centralMeridian field ID error." );
                                  return 0;
                              }
                              double centralMeridian = env->GetDoubleField( parameters, fid );

                              // Scale Factor
                              fid = env->GetFieldID( clazz, "scaleFactor", "D");
                              if (fid == NULL)
                              {
                                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: scaleFactor field ID error." );
                                  return 0;
                              }
                              double scaleFactor = env->GetDoubleField( parameters, fid );

                              // Hemisphere
                              fid = env->GetFieldID( clazz, "hemisphere", "C");
                              if (fid == NULL)
                              {
                                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: hemisphere field ID error." );
                                  return 0;
                              }
                              char hemisphere = (char)env->GetCharField( parameters, fid );

                              // False Easting
                              fid = env->GetFieldID( clazz, "falseEasting", "D");
                              if (fid == NULL)
                              {
                                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseEasting field ID error." );
                                  return 0;
                              }
                              double falseEasting = env->GetDoubleField( parameters, fid );

                              // False Northing
                              fid = env->GetFieldID( clazz, "falseNorthing", "D");
                              if (fid == NULL)
                              {
                                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: falseNorthing field ID error." );
                                  return 0;
                              }
                              double falseNorthing = env->GetDoubleField( parameters, fid );

                              return new PolarStereographicScaleFactorParameters( coordinateType, centralMeridian, scaleFactor, hemisphere, falseEasting, falseNorthing );
                            }
                            else
                            {
                              jclass utmParametersClazz = env->FindClass( "geotrans3/parameters/UTMParameters" );

                              if( env->IsInstanceOf( parameters, utmParametersClazz ) )
                              {
                                jclass clazz = env->GetObjectClass( parameters );

                                // Coordinate Type
                                jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                                if (fid == NULL)
                                {
                                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                                    return 0;
                                }
                                CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( parameters, fid );

                                // Zone
                                fid = env->GetFieldID( clazz, "zone", "J");
                                if (fid == NULL)
                                {
                                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: zone field ID error." );
                                    return 0;
                                }
                                long zone = env->GetLongField( parameters, fid );

                                // Override
                                fid = env->GetFieldID( clazz, "override", "J");
                                if (fid == NULL)
                                {
                                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: override field ID error." );
                                    return 0;
                                }
                                long override = env->GetLongField( parameters, fid );

                                return new UTMParameters( coordinateType, zone, override );
                              }
                              else
                              {
                                jclass clazz = env->GetObjectClass( parameters );

                                // Coordinate Type
                                jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
                                if (fid == NULL)
                                {
                                    throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                                    return 0;
                                }

                                return new CoordinateSystemParameters( ( CoordinateType::Enum )env->GetIntField( parameters, fid ) );
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
}


CoordinateTuple* translateFromJNICoordinates( JNIEnv *env, jobject coordinates )
{
  jclass geodeticCoordinatesClazz = env->FindClass( "geotrans3/coordinates/GeodeticCoordinates" );

  if( env->IsInstanceOf( coordinates, geodeticCoordinatesClazz ) )
  { 
    jclass clazz = env->GetObjectClass( coordinates );

    // Longitude
    jfieldID fid = env->GetFieldID( clazz, "longitude", "D");
    if (fid == NULL)
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Longitude field ID error." );
        return 0;
    }
    double longitude = env->GetDoubleField( coordinates, fid );

    // Latitude
    fid = env->GetFieldID( clazz, "latitude", "D");
    if (fid == NULL)
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Latitude field ID error." );
        return 0;
    }
    double latitude = env->GetDoubleField( coordinates, fid );

    // Height
    fid = env->GetFieldID( clazz, "height", "D");
    if (fid == NULL)
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Height field ID error." );
        return 0;
    }
    double height = env->GetDoubleField( coordinates, fid );

    return new GeodeticCoordinates( CoordinateType::geodetic, longitude, latitude, height ); 
  }
  else
  {
    jclass cartesianCoordinatesClazz = env->FindClass( "geotrans3/coordinates/CartesianCoordinates" );

    if( env->IsInstanceOf( coordinates, cartesianCoordinatesClazz ) )
    {
      jclass clazz = env->GetObjectClass( coordinates );

      // Coordinate Type
      jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
          return 0;
      }
      CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( coordinates, fid );

      // X
      fid = env->GetFieldID( clazz, "x", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: x field ID error." );
          return 0;
      }
      double x = env->GetDoubleField( coordinates, fid );

      // Y
      fid = env->GetFieldID( clazz, "y", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: y field ID error." );
          return 0;
      }
      double y = env->GetDoubleField( coordinates, fid );

      // Z
      fid = env->GetFieldID( clazz, "z", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: z field ID error." );
          return 0;
      }
      double z = env->GetDoubleField( coordinates, fid );

      return new CartesianCoordinates( coordinateType, x, y, z ); 
    }
    else
    {
      jclass mapProjectionCoordinatesClazz = env->FindClass( "geotrans3/coordinates/MapProjectionCoordinates" );

      if( env->IsInstanceOf( coordinates, mapProjectionCoordinatesClazz ) )
      {
        jclass clazz = env->GetObjectClass( coordinates );

        // Coordinate Type
        jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
        if (fid == NULL)
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
            return 0;
        }
        CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( coordinates, fid );
        
        // Easting
        fid = env->GetFieldID( clazz, "easting", "D");
        if (fid == NULL)
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: easting field ID error." );
            return 0;
        }
        double easting = env->GetDoubleField( coordinates, fid );

        // Northing
        fid = env->GetFieldID( clazz, "northing", "D");
        if (fid == NULL)
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: northing field ID error." );
            return 0;
        }
        double northing = env->GetDoubleField( coordinates, fid );

        return new MapProjectionCoordinates( coordinateType, easting, northing ); 
      }
      else
      {
        jclass stringCoordinatesClazz = env->FindClass( "geotrans3/coordinates/StringCoordinates" );

        if( env->IsInstanceOf( coordinates, stringCoordinatesClazz ) )
        {
          jclass clazz = env->GetObjectClass( coordinates );

          // Coordinate Type
          jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
          if (fid == NULL)
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
              return 0;
          }
          CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( coordinates, fid );

          // Coordinate String
          fid = env->GetFieldID( clazz, "coordinateString", "Ljava/lang/String;");
          if (fid == NULL)
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateString field ID error." );
              return 0;
          }
          jstring jStr = (jstring)env->GetObjectField( coordinates, fid );
          const char* str = env->GetStringUTFChars( jStr, NULL );
          if (str == NULL)
          {
              throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Out of memory.");
              return 0;
          }

          // Precision
          fid = env->GetFieldID( clazz, "precision", "I");
          if (fid == NULL)
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: precision field ID error." );
              return 0;
          }
          Precision::Enum precision = ( Precision::Enum )env->GetIntField( coordinates, fid );

          switch( coordinateType )
          {
            case CoordinateType::britishNationalGrid:
            {
              BNGCoordinates* bngCoordinates = new BNGCoordinates( coordinateType, str, precision );
              env->ReleaseStringUTFChars( jStr, str );
              return bngCoordinates;
            }
            case CoordinateType::georef:
            {
              GEOREFCoordinates* georefCoordinates = new GEOREFCoordinates( coordinateType, str, precision );
              env->ReleaseStringUTFChars( jStr, str );
              return georefCoordinates;
            }
            case CoordinateType::globalAreaReferenceSystem:
            {
              GARSCoordinates* garsCoordinates = new GARSCoordinates( coordinateType, str, precision );
              env->ReleaseStringUTFChars( jStr, str );
              return garsCoordinates;
            }
            case CoordinateType::militaryGridReferenceSystem:
            case CoordinateType::usNationalGrid:
            {
              MGRSorUSNGCoordinates* mgrsorUSNGCoordinates = new MGRSorUSNGCoordinates( coordinateType, str, precision );
              env->ReleaseStringUTFChars( jStr, str );
              return mgrsorUSNGCoordinates;
            }
          }
        }
        else
        {
          jclass upsCoordinatesClazz = env->FindClass( "geotrans3/coordinates/UPSCoordinates" );

          if( env->IsInstanceOf( coordinates, upsCoordinatesClazz ) )
          {
            jclass clazz = env->GetObjectClass( coordinates );

            // Coordinate Type
            jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
            if (fid == NULL)
            {
                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                return 0;
            }
            CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( coordinates, fid );

            // Hemisphere
            fid = env->GetFieldID( clazz, "hemisphere", "C");
            if (fid == NULL)
            {
                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: hemisphere field ID error." );
                return 0;
            }
            char hemisphere = (char)env->GetCharField( coordinates, fid );

            // Easting
            fid = env->GetFieldID( clazz, "easting", "D");
            if (fid == NULL)
            {
                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: easting field ID error." );
                return 0;
            }
            double easting = env->GetDoubleField( coordinates, fid );

            // Northing
            fid = env->GetFieldID( clazz, "northing", "D");
            if (fid == NULL)
            {
                throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: northing field ID error." );
                return 0;
            }
            double northing = env->GetDoubleField( coordinates, fid );

            return new UPSCoordinates( coordinateType, hemisphere, easting, northing ); 
          }
          else
          {
            jclass utmCoordinatesClazz = env->FindClass( "geotrans3/coordinates/UTMCoordinates" );

            if( env->IsInstanceOf( coordinates, utmCoordinatesClazz ) )
            {
              jclass clazz = env->GetObjectClass( coordinates );

              // Coordinate Type
              jfieldID fid = env->GetFieldID( clazz, "coordinateType", "I");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: coordinateType field ID error." );
                  return 0;
              }
              CoordinateType::Enum coordinateType = ( CoordinateType::Enum )env->GetIntField( coordinates, fid );

              // Zone
              fid = env->GetFieldID( clazz, "zone", "J");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: zone field ID error." );
                  return 0;
              }
              long zone = env->GetLongField( coordinates, fid );

              // Hemisphere
              fid = env->GetFieldID( clazz, "hemisphere", "C");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: hemisphere field ID error." );
                  return 0;
              }
              char hemisphere = (char)env->GetCharField( coordinates, fid );

              // Easting
              fid = env->GetFieldID( clazz, "easting", "D");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: easting field ID error." );
                  return 0;
              }
              double easting = env->GetDoubleField( coordinates, fid );

              // Northing
              fid = env->GetFieldID( clazz, "northing", "D");
              if (fid == NULL)
              {
                  throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: northing field ID error." );
                  return 0;
              }
              double northing = env->GetDoubleField( coordinates, fid );

              return new UTMCoordinates( coordinateType, zone, hemisphere, easting, northing ); 
            }
          }
        }
      }
    }
  }

  return 0;
}


Accuracy* translateFromJNIAccuracy( JNIEnv *env, jobject _accuracy )
{
    jclass clazz = env->GetObjectClass( _accuracy );

    // CE90
    jfieldID fid = env->GetFieldID( clazz, "ce90", "D" );
    if (fid == NULL)
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: CE90 field ID error." );
        return 0;
    }
    double ce90 = env->GetDoubleField( _accuracy, fid );

    // LE90
    fid = env->GetFieldID( clazz, "le90", "D" );
    if (fid == NULL)
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: LE90 field ID error." );
        return 0;
    }
    double le90 = env->GetDoubleField( _accuracy, fid );

    // SE90
    fid = env->GetFieldID(  clazz, "se90", "D");
    if (fid == NULL)
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: SE90 field ID error." );
        return 0;
    }
    double se90 = env->GetDoubleField( _accuracy, fid );

    return new Accuracy( ce90, le90, se90 ); 
}


jobject translateToJNIAccuracy( JNIEnv *env, Accuracy* _accuracy )
{
  jclass cls = env->FindClass( "geotrans3/coordinates/Accuracy" );
  if( cls == NULL )
  {
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy class not found." );
      return NULL;
  }

  jmethodID cid = env->GetMethodID( cls, "<init>", "(DDD)V" );
  if( cid == NULL )
  {
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy method id not found." );
      return NULL;
  }

  jobject jAccuracy = env->NewObject( cls, cid, (jdouble)_accuracy->circularError90(), (jdouble)_accuracy->linearError90(), (jdouble)_accuracy->sphericalError90() );
  if( jAccuracy == NULL )
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy object could not be created." );

  return jAccuracy;
}


jobject translateToJNICoordinates( JNIEnv *env, CoordinateTuple* coordinates )
{
  jobject jCoordinates;// = 0;
  jclass cls;
  jmethodID cid;

    switch( coordinates->coordinateType() )
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
      case CoordinateType::newZealandMapGrid:
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
        MapProjectionCoordinates _coordinates = ( *dynamic_cast< MapProjectionCoordinates* >( coordinates ) );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/MapProjectionCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjectionCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;DD)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjectionCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, (jint)coordinates->coordinateType(), jWarningStr, (jdouble)_coordinates.easting(), (jdouble)_coordinates.northing() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjectionCoordinates object could not be created." );
        break;
      }
      case CoordinateType::britishNationalGrid:
      {
        BNGCoordinates _coordinates = ( *dynamic_cast< BNGCoordinates* >( coordinates ) );
        jstring jStr = env->NewStringUTF( _coordinates.BNGString() );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/BNGCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: BNGCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;Ljava/lang/String;I)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: BNGCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, (jint)coordinates->coordinateType(), jWarningStr, jStr, (jint)_coordinates.precision() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: BNGCoordinates object could not be created." );

        break;
      }
      case CoordinateType::georef:
      {
        GEOREFCoordinates _coordinates = ( *dynamic_cast< GEOREFCoordinates* >( coordinates ) );
        jstring jStr = env->NewStringUTF( _coordinates.GEOREFString() );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/GEOREFCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GEOREFCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;Ljava/lang/String;I)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GEOREFCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, (jint)coordinates->coordinateType(), jWarningStr, jStr, (jint)_coordinates.precision() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GEOREFCoordinates object could not be created." );

        break;
      }
      case CoordinateType::globalAreaReferenceSystem:
      {
        GARSCoordinates _coordinates = ( *dynamic_cast< GARSCoordinates* >( coordinates ) );
        jstring jStr = env->NewStringUTF( _coordinates.GARSString() );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/GARSCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GARSCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;Ljava/lang/String;I)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GARSCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, (jint)coordinates->coordinateType(), jWarningStr, jStr, (jint)_coordinates.precision() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GARSCoordinates object could not be created." );

        break;
      }
      case CoordinateType::militaryGridReferenceSystem:
      case CoordinateType::usNationalGrid:
      {
        MGRSorUSNGCoordinates _coordinates = ( *dynamic_cast< MGRSorUSNGCoordinates* >( coordinates ) );
        jstring jStr = env->NewStringUTF( _coordinates.MGRSString() );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/MGRSorUSNGCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MGRSorUSNGCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;Ljava/lang/String;I)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MGRSorUSNGCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, (jint)coordinates->coordinateType(), jWarningStr, jStr, (jint)_coordinates.precision() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MGRSorUSNGCoordinates object could not be created." );

        break;
      }
      case CoordinateType::geocentric:
      case CoordinateType::localCartesian:
      {
        CartesianCoordinates _coordinates = ( *dynamic_cast< CartesianCoordinates* >( coordinates ) );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/CartesianCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: CartesianCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;DDD)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: CartesianCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, (jint)coordinates->coordinateType(), jWarningStr, (jdouble)_coordinates.x(), (jdouble)_coordinates.y(), (jdouble)_coordinates.z() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: CartesianCoordinates object could not be created." );

        break;
      }
      case CoordinateType::geodetic:
      {
        GeodeticCoordinates _coordinates = ( *dynamic_cast< GeodeticCoordinates* >( coordinates ) );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/GeodeticCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GeodeticCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;DDD)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GeodeticCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, coordinates->coordinateType(), jWarningStr, _coordinates.longitude(), _coordinates.latitude(), _coordinates.height() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GeodeticCoordinates object could not be created." );

        break;
      }
      case CoordinateType::universalPolarStereographic:
      {
        UPSCoordinates _coordinates = ( *dynamic_cast< UPSCoordinates* >( coordinates ) );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/UPSCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UPSCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;CDD)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UPSCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, (jint)coordinates->coordinateType(), jWarningStr, (jchar)_coordinates.hemisphere(), (jdouble)_coordinates.easting(), (jdouble)_coordinates.northing() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UPSCoordinates object could not be created." );

        break;
      }
      case CoordinateType::universalTransverseMercator:
      {
        UTMCoordinates _coordinates = ( *dynamic_cast< UTMCoordinates* >( coordinates ) );
        jstring jWarningStr = env->NewStringUTF( _coordinates.warningMessage() );

        cls = env->FindClass( "geotrans3/coordinates/UTMCoordinates" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UTMCoordinates class not found." );
            return NULL;
        }

        cid = env->GetMethodID( cls, "<init>", "(ILjava/lang/String;JCDD)V" );
        if( cid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UTMCoordinates method id not found." );
            return NULL;
        }

        jCoordinates = env->NewObject( cls, cid, (jint)coordinates->coordinateType(), jWarningStr, (jlong)_coordinates.zone(), (jchar)_coordinates.hemisphere(), (jdouble)_coordinates.easting(), (jdouble)_coordinates.northing() );
        if( jCoordinates == NULL )
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UTMCoordinates object could not be created." );

        break;
      }
      default:
        break;
    }

  return jCoordinates;
}

#ifdef __cplusplus
}
#endif

// CLASSIFICATION: UNCLASSIFIED
