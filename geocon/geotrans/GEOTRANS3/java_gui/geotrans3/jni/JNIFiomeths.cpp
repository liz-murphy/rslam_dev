// CLASSIFICATION: UNCLASSIFIED

#include <jni.h>
#include "fiomeths.h"
#include "ThrowException.h"
#include "CoordinateConversionService.h"
#include "CoordinateConversionException.h"
#include "CoordinateSystemParameters.h"
#include "JNICCSObjectTranslator.h"
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


using namespace MSP::CCS;


#ifdef __cplusplus
extern "C" {
#endif


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIFiomeths_jniFiomethsCreate(JNIEnv *env, jobject obj, jstring _fileName)
{
  // Input file name
  const char* fileName = env->GetStringUTFChars( _fileName, NULL );
  if( fileName == NULL )
  {
      throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid output file name.");
      return 0;
  }

  try
  {
    Fiomeths* fiomeths = new Fiomeths( fileName );

    env->ReleaseStringUTFChars( _fileName, fileName );

    return (jlong)fiomeths; 
  }
  catch(CoordinateConversionException e)
  {
    throwException(env, "geotrans3/exception/CoordinateConversionException", e.getMessage());
  }

  return 0;
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIFiomeths_jniFiomethsCreateExampleFile(JNIEnv *env, jobject obj)
{
  try
  {
    Fiomeths* fiomeths = new Fiomeths();

    return (jlong)fiomeths; 
  }
  catch(CoordinateConversionException e)
  {
    throwException(env, "geotrans3/exception/CoordinateConversionException", e.getMessage());
  }

  return 0;
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniFiomethsDestroy(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;

  if( fiomeths )
  {
    delete fiomeths;
    fiomeths = 0;
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniWriteExampleCoord(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;

  if( fiomeths )
  {
    try
    {
      fiomeths->writeExampleCoord();
    }
    catch(CoordinateConversionException e)
    {
      throwException(env, "geotrans3/exception/CoordinateConversionException", e.getMessage());
    }
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniCloseInputFile(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->closeInputFile();
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniCloseOutputFile(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->closeOutputFile();
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniConvertFile(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    try
    {
      fiomeths->convertFile();
    }
    catch(CoordinateConversionException e)
    {
      throwException(env, "geotrans3/exception/CoordinateConversionException", e.getMessage());
    }
  }
}


JNIEXPORT jstring JNICALL Java_geotrans3_jni_JNIFiomeths_jniGetDatumCode(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  jstring jStr = 0;
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    try
    {
      const char* datumCode = fiomeths->getDatumCode();

      if( !datumCode )
        throwException(env, "geotrans3/exception/CoordinateConversionException", "Invalid datum code.");
      else
      {
        jStr = env->NewStringUTF( datumCode );
        if (jStr == NULL)
            throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Out of memory.");
      }
    }
    catch(CoordinateConversionException e)
    {
      throwException(env, "geotrans3/exception/CoordinateConversionException", e.getMessage());
    }
  }

  return jStr;
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNIFiomeths_jniGetCoordinateSystemParameters(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  jobject jParameters = 0;
  try
  {
    Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
    if( fiomeths )
    {
      jclass cls;
      jmethodID mid;

      MSP::CCS::CoordinateSystemParameters* coordinateSystemParameters = fiomeths->getCoordinateSystemParameters();

      switch( coordinateSystemParameters->coordinateType() )
      {
        case CoordinateType::eckert4:
        case CoordinateType::eckert6:
        case CoordinateType::millerCylindrical:
        case CoordinateType::mollweide:
        case CoordinateType::sinusoidal:
        case CoordinateType::vanDerGrinten:
        {
          MapProjection3Parameters* parameters = ( dynamic_cast< MapProjection3Parameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/MapProjection3Parameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection3Parameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection3Parameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection3Parameters object could not be created." );
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
          MapProjection4Parameters* parameters = ( dynamic_cast< MapProjection4Parameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/MapProjection4Parameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection4Parameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection4Parameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->originLatitude(), (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection4Parameters object could not be created." );
          break;
        }
        case CoordinateType::lambertConformalConic1Parallel:
        case CoordinateType::transverseCylindricalEqualArea:
        case CoordinateType::transverseMercator:
        {
          MapProjection5Parameters* parameters = ( dynamic_cast< MapProjection5Parameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/MapProjection5Parameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection5Parameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection5Parameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->originLatitude(), (jdouble)parameters->scaleFactor(), 
                                       (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection5Parameters object could not be created." );
          break;
        }
        case CoordinateType::albersEqualAreaConic:
        case CoordinateType::lambertConformalConic2Parallels:
        {
          MapProjection6Parameters* parameters = ( dynamic_cast< MapProjection6Parameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/MapProjection6Parameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection6Parameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection6Parameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->originLatitude(), (jdouble)parameters->standardParallel1(),
                                        (jdouble)parameters->standardParallel2(), (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MapProjection6Parameters object could not be created." );
          break;
        }
        case CoordinateType::equidistantCylindrical:
        {
          EquidistantCylindricalParameters* parameters = ( dynamic_cast< EquidistantCylindricalParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/EquidistantCylindricalParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: EquidistantCylindricalParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: EquidistantCylindricalParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->standardParallel(), (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: EquidistantCylindricalParameters object could not be created." );
          break;
        }
        case CoordinateType::geodetic:
        {
          GeodeticParameters* parameters = ( dynamic_cast< GeodeticParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/GeodeticParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GeodeticParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(II)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GeodeticParameters method id not found." );
              return NULL;
          } 

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jint)parameters->heightType() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: GeodeticParameters object could not be created." );
          break;
        }
        case CoordinateType::localCartesian:
        {
          LocalCartesianParameters* parameters = ( dynamic_cast< LocalCartesianParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/LocalCartesianParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: LocalCartesianParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: LocalCartesianParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->longitude(), (jdouble)parameters->latitude(), (jdouble)parameters->height(), (jdouble)parameters->orientation() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: LocalCartesianParameters object could not be created." );
          break;
        }
        case CoordinateType::mercatorStandardParallel:
        {
          MercatorStandardParallelParameters* parameters = ( dynamic_cast< MercatorStandardParallelParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/MercatorStandardParallelParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MercatorStandardParallelParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MercatorStandardParallelParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->standardParallel(), (jdouble)parameters->scaleFactor(), 
                                       (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MercatorStandardParallelParameters object could not be created." );
          break;
        }
        case CoordinateType::mercatorScaleFactor:
        {
          MercatorScaleFactorParameters* parameters = ( dynamic_cast< MercatorScaleFactorParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/MercatorScaleFactorParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MercatorScaleFactorParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MercatorScaleFactorParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->scaleFactor(), 
                                       (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: MercatorScaleFactorParameters object could not be created." );
          break;
        }
        case CoordinateType::neys:
        {
          NeysParameters* parameters = ( dynamic_cast< NeysParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/NeysParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: NeysParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: NeysParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->originLatitude(), (jdouble)parameters->standardParallel1(), 
                                        (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: NeysParameters object could not be created." );
          break;
        }
        case CoordinateType::obliqueMercator:
        {
          ObliqueMercatorParameters* parameters = ( dynamic_cast< ObliqueMercatorParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/ObliqueMercatorParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ObliqueMercatorParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDDDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ObliqueMercatorParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->originLatitude(), (jdouble)parameters->longitude1(), (jdouble)parameters->latitude1(), 
                                        (jdouble)parameters->longitude2(), (jdouble)parameters->latitude2(), (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing(), (jdouble)parameters->scaleFactor() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ObliqueMercatorParameters object could not be created." );
          break;
        }
        case CoordinateType::polarStereographicStandardParallel:
        {
          PolarStereographicStandardParallelParameters* parameters = ( dynamic_cast< PolarStereographicStandardParallelParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/PolarStereographicStandardParallelParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: PolarStereographicStandardParallelParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: PolarStereographicStandardParallelParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->standardParallel(), (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: PolarStereographicStandardParallelParameters object could not be created." );
          break;
        }
        case CoordinateType::polarStereographicScaleFactor:
        {
          PolarStereographicScaleFactorParameters* parameters = ( dynamic_cast< PolarStereographicScaleFactorParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/PolarStereographicScaleFactorParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: PolarStereographicScaleFactorParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IDDCDD)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: PolarStereographicScaleFactorParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jdouble)parameters->centralMeridian(), (jdouble)parameters->scaleFactor(), (jchar)parameters->hemisphere(), (jdouble)parameters->falseEasting(), (jdouble)parameters->falseNorthing() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: PolarStereographicScaleFactorParameters object could not be created." );
          break;
        }
        case CoordinateType::universalTransverseMercator:
        {
          UTMParameters* parameters = ( dynamic_cast< UTMParameters* >( coordinateSystemParameters ) );

          cls = env->FindClass( "geotrans3/parameters/UTMParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UTMParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(IJJ)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UTMParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)parameters->coordinateType(), (jlong)parameters->zone(), (jlong)parameters->override() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: UTMParameters object could not be created." );
          break;
        }
        default:

          cls = env->FindClass( "geotrans3/parameters/CoordinateSystemParameters" );
          if( cls == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: CoordinateSystemParameters class not found." );
              return NULL;
          }

          mid = env->GetMethodID( cls, "<init>", "(I)V" );
          if( mid == NULL )
          {
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: CoordinateSystemParameters method id not found." );
              return NULL;
          }

          jParameters = env->NewObject( cls, mid, (jint)coordinateSystemParameters->coordinateType() );
          if( jParameters == NULL )
              throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: CoordinateSystemParameters object could not be created." );
          break;
      }
    }
  }
  catch(CoordinateConversionException e)
  {
    throwException(env, "geotrans3/exception/CoordinateConversionException", e.getMessage());
  }

  return jParameters;
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIFiomeths_jniGetNumErrors(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    return fiomeths->getNumErrors();
  }
  else 
    return 0;
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIFiomeths_jniGetNumProcessed(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    return fiomeths->getNumProcessed();
  }
  else 
    return 0;
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIFiomeths_jniGetNumWarnings(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    return fiomeths->getNumWarnings();
  }
  else 
    return 0;
}


JNIEXPORT jdouble JNICALL Java_geotrans3_jni_JNIFiomeths_jniGetElapsedTime(JNIEnv *env, jobject obj, jlong fiomethsPtr)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    return fiomeths->getElapsedTime();
  }
  else 
    return 0;
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniSetOutputFilename(JNIEnv *env, jobject obj, jlong fiomethsPtr, jstring _filename, jstring _targetDatumCode, jobject targetParameters)
{
  try
  {
    const char *filename;

    filename = env->GetStringUTFChars( _filename, NULL );
    if(filename == NULL)
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid output file name.");
        return;
    }

    // Target datum code
    const char* targetDatumCode = env->GetStringUTFChars( _targetDatumCode, NULL );
    if( targetDatumCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid target datum code.");
        return;
    }

    // Target parameters
    MSP::CCS::CoordinateSystemParameters* targetCoordinateSystemParameters = translateFromJNIParameters( env, targetParameters );

    Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
    if( fiomeths )
    {
      fiomeths->setOutputFilename(filename, targetDatumCode, targetCoordinateSystemParameters);
    }

    env->ReleaseStringUTFChars( _filename, filename );

    env->ReleaseStringUTFChars( _targetDatumCode, targetDatumCode );

    delete targetCoordinateSystemParameters;
    targetCoordinateSystemParameters = 0;
  }
  catch(CoordinateConversionException e)
  {
    throwException(env, "geotrans3/exception/CoordinateConversionException", e.getMessage());
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniSetUseNSEW(JNIEnv *env, jobject obj, jlong fiomethsPtr, jboolean useNSEW)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->setUseNSEW( useNSEW != 0 );
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniSetUseMinutes(JNIEnv *env, jobject obj, jlong fiomethsPtr, jboolean useMinutes)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->setUseMinutes( useMinutes != 0 );
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniSetUseSeconds(JNIEnv *env, jobject obj, jlong fiomethsPtr, jboolean useSeconds)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->setUseSeconds( useSeconds != 0 );
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniSetLatLongPrecision(JNIEnv *env, jobject obj, jlong fiomethsPtr, jint _precision)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->setLatLongPrecision( _precision );
  }
}

// BAEts29174 - Set output format for file operation
JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniSetSeparator(JNIEnv *env, jobject obj, jlong fiomethsPtr, jchar sepChar)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->setSeparator( sepChar );
  }
}

JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniSetLonRange(JNIEnv *env, jobject obj, jlong fiomethsPtr, jint longRange)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->setLonRange( longRange );
  }
}

JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniShowLeadingZeros(JNIEnv *env, jobject obj, jlong fiomethsPtr, jboolean lz)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->showLeadingZeros( lz );
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIFiomeths_jniSetGeodeticCoordinateOrder(JNIEnv *env, jobject obj, jlong fiomethsPtr, jboolean outputLatitudeLongitudeOrder)
{
  Fiomeths* fiomeths = ( Fiomeths* )fiomethsPtr;
  if( fiomeths )
  {
    fiomeths->setGeodeticCoordinateOrder( outputLatitudeLongitudeOrder != 0 );
  }
}

#ifdef __cplusplus
}
#endif

// CLASSIFICATION: UNCLASSIFIED
