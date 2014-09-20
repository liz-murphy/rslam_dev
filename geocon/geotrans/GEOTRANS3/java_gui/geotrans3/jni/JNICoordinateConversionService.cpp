// CLASSIFICATION: UNCLASSIFIED

#include <jni.h>
#include "string.h"
#include "CoordinateConversionService.h"
#include "Accuracy.h"
#include "DatumType.h"
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
#include "DatumLibrary.h"
#include "EllipsoidLibrary.h"
#include "CoordinateSystemParameters.h"
#include "CoordinateConversionException.h"
#include "ThrowException.h"
#include "JNICCSObjectTranslator.h"


using namespace MSP::CCS;


#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniCreate(JNIEnv *env, jobject jobj, jstring _sourceDatumCode, jobject sourceParameters, jstring _targetDatumCode, jobject targetParameters)
{
  //initialize the return value to null
  jlong return_val = 0;
  MSP::CCS::CoordinateSystemParameters* sourceCoordinateSystemParameters = 0;
  MSP::CCS::CoordinateSystemParameters* targetCoordinateSystemParameters = 0;
  const char* sourceDatumCode;
  const char* targetDatumCode;
  try
  {
    // Source datum code
    sourceDatumCode = env->GetStringUTFChars( _sourceDatumCode, NULL );
    if( sourceDatumCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid source datum code.");
        return return_val;
    }

    // Target datum code
    targetDatumCode = env->GetStringUTFChars( _targetDatumCode, NULL );
    if( targetDatumCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid target datum code.");
        return return_val;
    }

	// Source parameters
    sourceCoordinateSystemParameters = translateFromJNIParameters( env, sourceParameters );

    // Target parameters
    targetCoordinateSystemParameters = translateFromJNIParameters( env, targetParameters );

    //The constructor can throw an exception, need to cleanup in exception handler
	CoordinateConversionService* coordinateConversionService = new CoordinateConversionService(sourceDatumCode, sourceCoordinateSystemParameters, targetDatumCode, targetCoordinateSystemParameters);

	return_val = (jlong)coordinateConversionService;

  }
  catch( CoordinateConversionException e )
  {
    throwException(env, "geotrans3/exception/CoordinateConversionException", e.getMessage());
  }

  //cleanup memory before returning
  env->ReleaseStringUTFChars( _sourceDatumCode, sourceDatumCode );

  delete sourceCoordinateSystemParameters;

  env->ReleaseStringUTFChars( _targetDatumCode, targetDatumCode );

  delete targetCoordinateSystemParameters;

  return return_val;
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniDestroy(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr)
{
  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
  {
    delete coordinateConversionService;
  }
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniConvertSourceToTarget(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr, jobject _sourceCoordinates, jobject _sourceAccuracy, jobject _targetCoordinates, jobject _targetAccuracy)
{
  jobject convertResults = NULL;

  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
  {
    // Source coordinates
    MSP::CCS::CoordinateTuple* sourceCoordinateTuple = translateFromJNICoordinates( env, _sourceCoordinates );

    // Source accuracy
    MSP::CCS::Accuracy* sourceAccuracy = translateFromJNIAccuracy( env, _sourceAccuracy );

    // Target coordinates
    MSP::CCS::CoordinateTuple* targetCoordinateTuple = translateFromJNICoordinates( env, _targetCoordinates );

    // Target accuracy
    MSP::CCS::Accuracy* targetAccuracy = translateFromJNIAccuracy( env, _targetAccuracy );

    if( sourceCoordinateTuple && sourceAccuracy && targetCoordinateTuple && targetAccuracy )
    {
      try
      {
       coordinateConversionService->convertSourceToTarget( sourceCoordinateTuple, sourceAccuracy, *targetCoordinateTuple, *targetAccuracy );

       jobject jniTargetCoordinateTuple = translateToJNICoordinates( env, targetCoordinateTuple );

       jobject jniTargetAccuracy = translateToJNIAccuracy( env, targetAccuracy );

        // Create ConvertResults
        jclass cls = env->FindClass( "geotrans3/coordinates/ConvertResults" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertResults class not found." );
            return NULL;
        }

        jmethodID mid = env->GetMethodID( cls, "<init>", "(Lgeotrans3/coordinates/CoordinateTuple;Lgeotrans3/coordinates/Accuracy;)V" );
        if( mid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertResults method id not found." );
            return NULL;
        }

        convertResults = env->NewObject( cls, mid, jniTargetCoordinateTuple, jniTargetAccuracy );
        if( convertResults == NULL )
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertResults object could not be created." );
      }
      catch( CoordinateConversionException e )
      {
        throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
      }
    }
    else
      throwException( env, "geotrans3/exception/CoordinateConversionException", "jniConvertSourceToTarget: Invalid coordinates\n" );

    delete sourceCoordinateTuple;
    delete sourceAccuracy;
    delete targetCoordinateTuple;
    delete targetAccuracy;

  }

  return convertResults;
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniConvertTargetToSource(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr, jobject _targetCoordinates, jobject _targetAccuracy, jobject _sourceCoordinates, jobject _sourceAccuracy)
{
  jobject convertResults = NULL;

  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
  {
    // Target coordinates
    MSP::CCS::CoordinateTuple* targetCoordinateTuple = translateFromJNICoordinates( env, _targetCoordinates );

    // Target accuracy
    MSP::CCS::Accuracy* targetAccuracy = translateFromJNIAccuracy( env, _targetAccuracy );

    // Source coordinates
    MSP::CCS::CoordinateTuple* sourceCoordinateTuple = translateFromJNICoordinates( env, _sourceCoordinates );

    // Source accuracy
    MSP::CCS::Accuracy* sourceAccuracy = translateFromJNIAccuracy( env, _sourceAccuracy );

    if( targetCoordinateTuple && targetAccuracy && sourceCoordinateTuple && sourceAccuracy )
    {
      try
      {
        coordinateConversionService->convertTargetToSource( targetCoordinateTuple, targetAccuracy, *sourceCoordinateTuple, *sourceAccuracy );

        jobject jniSourceCoordinateTuple = translateToJNICoordinates( env, sourceCoordinateTuple );

        jobject jniSourceAccuracy = translateToJNIAccuracy( env, sourceAccuracy );


        // Create ConvertResults
        jclass cls = env->FindClass( "geotrans3/coordinates/ConvertResults" );
        if( cls == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertResults class not found." );
            return NULL;
        }

        jmethodID mid = env->GetMethodID( cls, "<init>", "(Lgeotrans3/coordinates/CoordinateTuple;Lgeotrans3/coordinates/Accuracy;)V" );
        if( mid == NULL )
        {
            throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertResults method id not found." );
            return NULL;
        }

        convertResults = env->NewObject( cls, mid, jniSourceCoordinateTuple, jniSourceAccuracy );
        if( convertResults == NULL )
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertResults object could not be created." );
      }
      catch( CoordinateConversionException e )
      {
        throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
      }
    }
    else
      throwException( env, "geotrans3/exception/CoordinateConversionException", "jniConvertTargetToSource: Invalid coordinates\n" );

    delete targetCoordinateTuple;
    delete targetAccuracy;
    delete sourceCoordinateTuple;
    delete sourceAccuracy;

  }

  return convertResults;
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniGetDatumLibrary(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr) 
{
  DatumLibrary* datumLibrary = 0;
  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
    datumLibrary = coordinateConversionService->getDatumLibrary();

  return (jlong)datumLibrary; 
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniGetEllipsoidLibrary(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr)
{
  EllipsoidLibrary* ellipsoidLibrary = 0;
  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
    ellipsoidLibrary = coordinateConversionService->getEllipsoidLibrary();

  return (jlong)ellipsoidLibrary; 
}


JNIEXPORT jdouble JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniGetServiceVersion(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr)
{
  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
    return coordinateConversionService->getServiceVersion();
  else
    return 0;
}


JNIEXPORT jstring JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniGetDatum(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr, jint direction)
{
  jstring jStr = NULL;
  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
  {
    const char* datumCode = coordinateConversionService->getDatum( ( SourceOrTarget::Enum )direction );

    jStr = env->NewStringUTF(datumCode);
    if (jStr == NULL)
      throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid datum code.");
  }

  return jStr;
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniGetCoordinateSystem(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr, jint direction)
{
  jobject jParameters = 0;
  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;
  if( coordinateConversionService )
  {
    jclass cls;
    jmethodID mid;

    MSP::CCS::CoordinateSystemParameters* coordinateSystemParameters = coordinateConversionService->getCoordinateSystem( ( SourceOrTarget::Enum )direction );

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

  return jParameters;
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniConvertSourceToTargetCollection(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr,
                                                                                        jobject _sourceCoordinates, jobject _sourceAccuracy, 
                                                                                        jobject _targetCoordinates, jobject _targetAccuracy)
{
  jobject convertCollectionResults = 0;

  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
  {
    jclass jVectorClass = env->FindClass( "java/util/Vector" );

    jmethodID sizeMid = env->GetMethodID( jVectorClass, "size", "()I" );
    if( sizeMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: size method id not found." );
        return NULL;
    }

    jmethodID getMid = env->GetMethodID( jVectorClass, "get", "(I)Ljava/lang/Object;" );
    if( getMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: get method id not found." );
        return NULL;
    }


    // Create source coordinates vector
    jint numSourceCoordinates = env->CallIntMethod( _sourceCoordinates, sizeMid );
    std::vector<CoordinateTuple*> sourceCoordinateVector;
    sourceCoordinateVector.reserve( numSourceCoordinates );

    int i;
    for( i = 0; i < numSourceCoordinates; i++ )
    {
      jobject coordinate = env->CallObjectMethod( _sourceCoordinates, getMid, i );

      CoordinateTuple* sourceCoordinateTuple = translateFromJNICoordinates( env, coordinate );

      sourceCoordinateVector.push_back( sourceCoordinateTuple );
    }


    // Create source accuracy vector
    jint numSourceAccuracies = env->CallIntMethod( _sourceAccuracy, sizeMid );
    std::vector<Accuracy*> sourceAccuracyVector;
    sourceAccuracyVector.reserve( numSourceAccuracies );

    for( i = 0; i < numSourceAccuracies; i++ )
    {
      jobject accuracy = env->CallObjectMethod( _sourceAccuracy, getMid, i );

      Accuracy* sourceAccuracy = new Accuracy();
      jclass clazz = env->GetObjectClass( accuracy );

      // CE90
      jfieldID fid = env->GetFieldID( clazz, "ce90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ce90 field ID error." );
          return 0;
      }
      sourceAccuracy->setCircularError90( env->GetDoubleField( accuracy, fid ) );

      // LE90
      fid = env->GetFieldID( clazz, "le90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: le90 field ID error." );
          return 0;
      }
      sourceAccuracy->setLinearError90( env->GetDoubleField( accuracy, fid ) );

      // SE90
      fid = env->GetFieldID( clazz, "se90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: se90 field ID error." );
          return 0;
      }
      sourceAccuracy->setSphericalError90( env->GetDoubleField( accuracy, fid ) );

      sourceAccuracyVector.push_back( sourceAccuracy );
    }


    // Convert
    std::vector<CoordinateTuple*> targetCoordinateVector;
    targetCoordinateVector.reserve( numSourceCoordinates );
    std::vector<Accuracy*> targetAccuracyVector;
    targetAccuracyVector.reserve( numSourceCoordinates );

    coordinateConversionService->convertSourceToTargetCollection( sourceCoordinateVector, sourceAccuracyVector,
                                                                  targetCoordinateVector, targetAccuracyVector );


    jmethodID initMid = env->GetMethodID( jVectorClass, "<init>", "()V" );
    if( initMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: init method id not found." );
        return NULL;
    }

    jmethodID addElementMid = env->GetMethodID( jVectorClass, "addElement", "(Ljava/lang/Object;)V" );
    if( addElementMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: addElement method id not found." );
        return NULL;
    }


    // Create target coordinates vector
    jobject jTargetCoordinateVectorObj = env->NewObject( jVectorClass, initMid );

    int num = targetCoordinateVector.size();

    for( i = 0; i < num; i++ )
    {
      // Create CoordinateTuple parameter
      jobject jCoordinates = translateToJNICoordinates( env, targetCoordinateVector[i] );

      env->CallBooleanMethod( jTargetCoordinateVectorObj, addElementMid, jCoordinates );
    }


    // Create target accuracy vector
    jobject jTargetAccuracyVectorObj = env->NewObject( jVectorClass, initMid );

    num = targetAccuracyVector.size();

    for( i = 0; i < num; i++ )
    {
      // Create Accuracy parameter
      jclass acls = env->FindClass( "geotrans3/coordinates/Accuracy" );
      if( acls == NULL )
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy class not found." );
          return NULL;
      }

      jmethodID cid = env->GetMethodID( acls, "<init>", "(DDD)V" );
      if( cid == NULL )
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy method id not found." );
          return NULL;
      }

      Accuracy targetAccuracy = *targetAccuracyVector[i];
      jobject jAccuracy = env->NewObject( acls, cid, targetAccuracy.circularError90(), targetAccuracy.linearError90(), targetAccuracy.sphericalError90() );
      if( jAccuracy == NULL )
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy object could not be created." );
          return NULL;
      }

      env->CallBooleanMethod( jTargetAccuracyVectorObj, addElementMid, jAccuracy );
    }


    // Create ConvertCollectionResults
    jclass cls = env->FindClass( "geotrans3/coordinates/ConvertCollectionResults" );
    if( cls == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertCollectionResults class not found." );
        return NULL;
    }

    jmethodID mid = env->GetMethodID( cls, "<init>", "(Ljava/util/Vector;Ljava/util/Vector;)V" );
    if( mid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertCollectionResults method id not found." );
        return NULL;
    }

    convertCollectionResults = env->NewObject( cls, mid, jTargetCoordinateVectorObj, jTargetAccuracyVectorObj );
    if( convertCollectionResults == NULL )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertCollectionResults object could not be created." );
      return NULL;
    }


    // Release memory
    for( i = 0; i < numSourceCoordinates; i++ )
    {
      delete sourceCoordinateVector[i];
    }
    sourceCoordinateVector.clear();

    for( i = 0; i < numSourceAccuracies; i++ )
    {
      delete sourceAccuracyVector[i];
    }
    sourceAccuracyVector.clear();

    for( i = 0; i < numSourceCoordinates; i++ )
    {
      delete targetCoordinateVector[i];
    }
    targetCoordinateVector.clear();
  }

  return convertCollectionResults;
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniConvertTargetToSourceCollection(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr,
                                                                                        jobject _targetCoordinates, jobject _targetAccuracy, 
                                                                                        jobject _sourceCoordinates, jobject _sourceAccuracy)
{
  jobject convertCollectionResults = 0;

  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
  {
    jclass jVectorClass = env->FindClass( "java/util/Vector" );

    jmethodID sizeMid = env->GetMethodID( jVectorClass, "size", "()I" );
    if( sizeMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: size method id not found." );
        return NULL;
    }

    jmethodID getMid = env->GetMethodID( jVectorClass, "get", "(I)Ljava/lang/Object;" );
    if( getMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: get method id not found." );
        return NULL;
    }


    // Create target coordinates vector
    jint numTargetCoordinates = env->CallIntMethod( _targetCoordinates, sizeMid );
    std::vector<CoordinateTuple*> targetCoordinateVector;
    targetCoordinateVector.reserve( numTargetCoordinates );

    int i;
    for( i = 0; i < numTargetCoordinates; i++ )
    {
      jobject coordinate = env->CallObjectMethod( _targetCoordinates, getMid, i );

      CoordinateTuple* targetCoordinateTuple = translateFromJNICoordinates( env, coordinate );

      targetCoordinateVector.push_back( targetCoordinateTuple );
    }


    // Create target accuracy vector
    jint numTargetAccuracies = env->CallIntMethod( _targetAccuracy, sizeMid );
    std::vector<Accuracy*> targetAccuracyVector;
    targetAccuracyVector.reserve( numTargetAccuracies );

    for( i = 0; i < numTargetAccuracies; i++ )
    {
      jobject accuracy = env->CallObjectMethod( _targetAccuracy, getMid, i );

      Accuracy* targetAccuracy = new Accuracy();
      jclass clazz = env->GetObjectClass( accuracy );

      // CE90
      jfieldID fid = env->GetFieldID( clazz, "ce90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ce90 field ID error." );
          return 0;
      }
      targetAccuracy->setCircularError90( env->GetDoubleField( accuracy, fid ) );

      // LE90
      fid = env->GetFieldID( clazz, "le90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: le90 field ID error." );
          return 0;
      }
      targetAccuracy->setLinearError90( env->GetDoubleField( accuracy, fid ) );

      // SE90
      fid = env->GetFieldID( clazz, "se90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: se90 field ID error." );
          return 0;
      }
      targetAccuracy->setSphericalError90( env->GetDoubleField( accuracy, fid ) );

      targetAccuracyVector.push_back( targetAccuracy );
    }


    // Convert
    std::vector<CoordinateTuple*> sourceCoordinateVector;
    sourceCoordinateVector.reserve( numTargetCoordinates );
    std::vector<Accuracy*> sourceAccuracyVector;
    sourceAccuracyVector.reserve( numTargetCoordinates );

    coordinateConversionService->convertTargetToSourceCollection( targetCoordinateVector, targetAccuracyVector,
                                                                  sourceCoordinateVector, sourceAccuracyVector );


    jmethodID initMid = env->GetMethodID( jVectorClass, "<init>", "()V" );
    if( initMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: init method id not found." );
        return NULL;
    }

    jmethodID addElementMid = env->GetMethodID( jVectorClass, "addElement", "(Ljava/lang/Object;)V" );
    if( addElementMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: addElement method id not found." );
        return NULL;
    }


    // Create source coordinates vector
    jobject jSourceCoordinateVectorObj = env->NewObject( jVectorClass, initMid );

    int num = sourceCoordinateVector.size();

    for( i = 0; i < num; i++ )
    {
      // Create CoordinateTuple parameter
      jobject jCoordinates = translateToJNICoordinates( env, sourceCoordinateVector[i] );

      env->CallBooleanMethod( jSourceCoordinateVectorObj, addElementMid, jCoordinates );
    }


    // Create source accuracy vector
    jobject jSourceAccuracyVectorObj = env->NewObject( jVectorClass, initMid );

    num = sourceAccuracyVector.size();

    for( i = 0; i < num; i++ )
    {
      // Create Accuracy parameter
      jclass acls = env->FindClass( "geotrans3/coordinates/Accuracy" );
      if( acls == NULL )
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy class not found." );
          return NULL;
      }

      jmethodID cid = env->GetMethodID( acls, "<init>", "(DDD)V" );
      if( cid == NULL )
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy method id not found." );
          return NULL;
      }

      Accuracy sourceAccuracy = *sourceAccuracyVector[i];
      jobject jAccuracy = env->NewObject( acls, cid, sourceAccuracy.circularError90(), sourceAccuracy.linearError90(), sourceAccuracy.sphericalError90() );
      if( jAccuracy == NULL )
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy object could not be created." );
          return NULL;
      }

      env->CallBooleanMethod( jSourceAccuracyVectorObj, addElementMid, jAccuracy );
    }


    // Create ConvertCollectionResults
    jclass cls = env->FindClass( "geotrans3/coordinates/ConvertCollectionResults" );
    if( cls == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertCollectionResults class not found." );
        return NULL;
    }

    jmethodID mid = env->GetMethodID( cls, "<init>", "(Ljava/util/Vector;Ljava/util/Vector;)V" );
    if( mid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertCollectionResults method id not found." );
        return NULL;
    }

    convertCollectionResults = env->NewObject( cls, mid, jSourceCoordinateVectorObj, jSourceAccuracyVectorObj );
    if( convertCollectionResults == NULL )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertCollectionResults object could not be created." );
      return NULL;
    }


    // Release memory
    for( i = 0; i < numTargetCoordinates; i++ )
    {
      delete targetCoordinateVector[i];
    }
    targetCoordinateVector.clear();

    for( i = 0; i < numTargetAccuracies; i++ )
    {
      delete targetAccuracyVector[i];
    }
    targetAccuracyVector.clear();

    for( i = 0; i < numTargetCoordinates; i++ )
    {
      delete sourceCoordinateVector[i];
    }
    sourceCoordinateVector.clear();
  }

  return convertCollectionResults;
}

/*JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNICoordinateConversionService_jniConvertVector(JNIEnv *env, jobject jobj, jlong coordinateConversionPtr,
                                                                                        jstring _sourceDatumCode, jobject sourceParameters, 
                                                                                        jstring _targetDatumCode, jobject targetParameters,
                                                                                        jobject sourceCoordinatesVector, jobject _sourceAccuracyVector)
{
  jobject convertVectorResults = 0;
  CoordinateConversionService* coordinateConversionService = ( CoordinateConversionService* )coordinateConversionPtr;

  if( coordinateConversionService )
  {
    const char *sourceDatumCode;
    const char *targetDatumCode;

    sourceDatumCode = env->GetStringUTFChars( _sourceDatumCode, NULL );
    if( sourceDatumCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Out of memory.");
        return 0;
    }
    targetDatumCode = env->GetStringUTFChars( _targetDatumCode, NULL );
    if( targetDatumCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Out of memory.");
        return 0;
    }

    CoordinateSystemParameters* sourceCoordinateSystemParameters = translateFromJNIParameters( env, coordinateConversionService, SourceOrTarget::source, sourceParameters/*, &sourceCoordinateSystemParameters*/ /*);

    CoordinateSystemParameters* targetCoordinateSystemParameters = translateFromJNIParameters( env, coordinateConversionService, SourceOrTarget::source, targetParameters/*, &targetCoordinateSystemParameters */ /*);


    jclass jVectorClass = env->FindClass( "java/util/Vector" );

    jmethodID sizeMid = env->GetMethodID( jVectorClass, "size", "()I" );
    if( sizeMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: size method id not found." );
        return NULL;
    }

    jmethodID getMid = env->GetMethodID( jVectorClass, "get", "(I)Ljava/lang/Object;" );
    if( getMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: get method id not found." );
        return NULL;
    }

    // Create source coordinates vector
    jint numSourceCoordinates = env->CallIntMethod( sourceCoordinatesVector, sizeMid );
    std::vector<CoordinateTuple*> sourceCoordinateVector;
    sourceCoordinateVector.reserve( numSourceCoordinates );

    int i;
    for( i = 0; i < numSourceCoordinates; i++ )
    {
      jobject coordinate = env->CallObjectMethod( sourceCoordinatesVector, getMid, i );

      CoordinateTuple* sourceCoordinateTuple = translateFromJNICoordinates( env, coordinateConversionService, coordinate );

      sourceCoordinateVector.push_back( sourceCoordinateTuple );
    }


    // Create source accuracy vector
    jint numSourceAccuracies = env->CallIntMethod( _sourceAccuracyVector, sizeMid );
    std::vector<Accuracy*> sourceAccuracyVector;
    sourceAccuracyVector.reserve( numSourceAccuracies );

    for( i = 0; i < numSourceAccuracies; i++ )
    {
      jobject accuracy = env->CallObjectMethod( _sourceAccuracyVector, getMid, i );

      Accuracy* sourceAccuracy = new Accuracy();
      jclass clazz = env->GetObjectClass( accuracy );

      // CE90
      jfieldID fid = env->GetFieldID( clazz, "ce90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ce90 field ID error." );
          return 0;
      }
      sourceAccuracy->setCircularError90( env->GetDoubleField( accuracy, fid ) );

      // LE90
      fid = env->GetFieldID( clazz, "le90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: le90 field ID error." );
          return 0;
      }
      sourceAccuracy->setLinearError90( env->GetDoubleField( accuracy, fid ) );

      // SE90
      fid = env->GetFieldID( clazz, "se90", "D");
      if (fid == NULL)
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: se90 field ID error." );
          return 0;
      }
      sourceAccuracy->setSphericalError90( env->GetDoubleField( accuracy, fid ) );

      sourceAccuracyVector.push_back( sourceAccuracy );
    }


    // Convert
    std::vector<CoordinateTuple*> targetCoordinateVector;
    targetCoordinateVector.reserve( numSourceCoordinates );
    std::vector<Accuracy> targetAccuracyVector;
    targetAccuracyVector.reserve( numSourceCoordinates );
    std::vector<long> conversionStatusVector;
    conversionStatusVector.reserve( numSourceCoordinates );

    coordinateConversionService->convertVector( sourceDatumCode, sourceCoordinateSystemParameters,
                                                targetDatumCode, targetCoordinateSystemParameters,
                                                sourceCoordinateVector, sourceAccuracyVector,
                                                &targetCoordinateVector, &targetAccuracyVector, &conversionStatusVector);


    jmethodID initMid = env->GetMethodID( jVectorClass, "<init>", "()V" );
    if( initMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: init method id not found." );
        return NULL;
    }

    jmethodID addElementMid = env->GetMethodID( jVectorClass, "addElement", "(Ljava/lang/Object;)V" );
    if( addElementMid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: addElement method id not found." );
        return NULL;
    }


    // Create target coordinates vector
    jobject jTargetCoordinateVectorObj = env->NewObject( jVectorClass, initMid );

    int num = targetCoordinateVector.size();

    for( i = 0; i < num; i++ )
    {
      // Create CoordinateTuple parameter
      jobject jCoordinates = translateToJNICoordinates( env, coordinateConversionService, targetCoordinateVector[i] );

      env->CallBooleanMethod( jTargetCoordinateVectorObj, addElementMid, jCoordinates );
    }


    // Create target accuracy vector
    jobject jTargetAccuracyVectorObj = env->NewObject( jVectorClass, initMid );

    num = targetAccuracyVector.size();

    for( i = 0; i < num; i++ )
    {
      // Create Accuracy parameter
      jclass acls = env->FindClass( "geotrans3/coordinates/Accuracy" );
      if( acls == NULL )
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy class not found." );
          return NULL;
      }

      jmethodID cid = env->GetMethodID( acls, "<init>", "(DDD)V" );
      if( cid == NULL )
      {
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy method id not found." );
          return NULL;
      }

      Accuracy targetAccuracy = targetAccuracyVector[i];
      jobject jAccuracy = env->NewObject( acls, cid, targetAccuracy.circularError90(), targetAccuracy.linearError90(), targetAccuracy.sphericalError90() );
      if( jAccuracy == NULL )
          throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Accuracy object could not be created." );

      env->CallBooleanMethod( jTargetAccuracyVectorObj, addElementMid, jAccuracy );
    }


    // Create conversion status vector
    jobject jConversionStatusVectorObj = env->NewObject( jVectorClass, initMid );

    num = conversionStatusVector.size();

    for( i = 0; i < num; i++ )
    {
      // Create Conversion Status parameter
      long conversionStatus = conversionStatusVector[i];

      env->CallBooleanMethod( jConversionStatusVectorObj, addElementMid, conversionStatus );
    }

    
    // Create ConvertVectorResults
    jclass cls = env->FindClass( "geotrans3/coordinates/ConvertVectorResults" );
    if( cls == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertVectorResults class not found." );
        return NULL;
    }

    jmethodID mid = env->GetMethodID( cls, "<init>", "(Ljava/util/Vector;Ljava/util/Vector;Ljava/util/Vector;)V" );
    if( mid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertVectorResults method id not found." );
        return NULL;
    }

    convertVectorResults = env->NewObject( cls, mid, jTargetCoordinateVectorObj, jTargetAccuracyVectorObj, jConversionStatusVectorObj );
    if( convertVectorResults == NULL )
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: ConvertVectorResults object could not be created." );


    // Release memory
    delete sourceCoordinateSystemParameters;
    sourceCoordinateSystemParameters = 0;

    delete targetCoordinateSystemParameters;
    targetCoordinateSystemParameters = 0;

    env->ReleaseStringUTFChars( _sourceDatumCode, sourceDatumCode );
    env->ReleaseStringUTFChars( _targetDatumCode, targetDatumCode );

    for( i = 0; i < numSourceCoordinates; i++ )
    {
      delete sourceCoordinateVector[i];
    }
    sourceCoordinateVector.clear();

    for( i = 0; i < numSourceAccuracies; i++ )
    {
      delete sourceAccuracyVector[i];
    }
    sourceAccuracyVector.clear();

    for( i = 0; i < numSourceCoordinates; i++ )
    {
      delete targetCoordinateVector[i];
    }
    targetCoordinateVector.clear();
  }

  return convertVectorResults;
}*/

#ifdef __cplusplus
}
#endif

// CLASSIFICATION: UNCLASSIFIED
