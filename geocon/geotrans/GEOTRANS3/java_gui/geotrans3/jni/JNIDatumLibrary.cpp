// CLASSIFICATION: UNCLASSIFIED

#include <jni.h>
#include "string.h"
#include "JNIDatumLibrary.h"
#include "DatumLibrary.h"
#include "CoordinateConversionException.h"
#include "ThrowException.h"


using namespace MSP::CCS;


#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_geotrans3_jni_JNIDatumLibrary_jniDefineDatum(JNIEnv *env, jobject jobj, 
          jlong datumLibraryPtr, jint _datumType, jstring _datumCode, jstring _datumName, jstring _ellipsoidCode, 
          jdouble _deltaX, jdouble _deltaY, jdouble _deltaZ, jdouble _sigmaX, jdouble _sigmaY, jdouble _sigmaZ, 
          jdouble _westLon, jdouble _eastLon, jdouble _southLat, jdouble _northLat, 
          jdouble _rotationX, jdouble _rotationY, jdouble _rotationZ, jdouble _scaleFactor)
{
  DatumLibrary* datumLibrary = ( DatumLibrary* )datumLibraryPtr;

  if( datumLibrary )
  {
    const char* datumCode = env->GetStringUTFChars( _datumCode, NULL );
    if( datumCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid datum code.");
        return;
    }

    const char* datumName = env->GetStringUTFChars( _datumName, NULL );
    if( datumName == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid datum name.");
        return;
    }

    const char* ellipsoidCode = env->GetStringUTFChars( _ellipsoidCode, NULL );
    if( ellipsoidCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid ellipsoid code.");
        return;
    }

    try
    {
      datumLibrary->defineDatum( _datumType, datumCode, datumName, ellipsoidCode, 
            _deltaX, _deltaY, _deltaZ, _sigmaX, _sigmaY, _sigmaZ, 
            _westLon, _eastLon, _southLat, _northLat, 
            _rotationX, _rotationY, _rotationZ, _scaleFactor);
    }
    catch( CoordinateConversionException e )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
    }

    env->ReleaseStringUTFChars( _datumCode, datumCode );
    env->ReleaseStringUTFChars( _datumName, datumName );
    env->ReleaseStringUTFChars( _ellipsoidCode, ellipsoidCode );
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIDatumLibrary_jniRemoveDatum(JNIEnv *env, jobject jobj, jlong datumLibraryPtr, jstring _datumCode)
{
  DatumLibrary* datumLibrary = ( DatumLibrary* )datumLibraryPtr;

  if( datumLibrary )
  {
    const char* datumCode = env->GetStringUTFChars( _datumCode, NULL );
    if( datumCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid datum code.");
        return;
    }
    
    try
    {
      datumLibrary->removeDatum( datumCode );
    }
    catch( CoordinateConversionException e )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
    }

    env->ReleaseStringUTFChars( _datumCode, datumCode );
  }
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIDatumLibrary_jniGetDatumCount(JNIEnv *env, jobject jobj, jlong datumLibraryPtr)
{
  long count = 0;

  DatumLibrary* datumLibrary = ( DatumLibrary* )datumLibraryPtr;

  if( datumLibrary )
  {
    datumLibrary->getDatumCount( &count );
  }

  return count;
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIDatumLibrary_jniGetDatumIndex(JNIEnv *env, jobject jobj, jlong datumLibraryPtr, jstring _datumCode)
{
  long datumIndex = 0;
  DatumLibrary* datumLibrary = ( DatumLibrary* )datumLibraryPtr;

  if( datumLibrary )
  {
    const char *datumCode = env->GetStringUTFChars( _datumCode, NULL) ;
    if( datumCode == NULL )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid datum code." );
      return datumIndex;
    }

    try
    {
      datumLibrary->getDatumIndex( datumCode, &datumIndex );
    }
    catch( CoordinateConversionException e )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
    }

    env->ReleaseStringUTFChars( _datumCode, datumCode );
  }

  return datumIndex;
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNIDatumLibrary_jniGetDatumInfo(JNIEnv *env, jobject jobj, jlong datumLibraryPtr, jlong index)
{
  DatumLibrary* datumLibrary = ( DatumLibrary* )datumLibraryPtr;

  jobject info = 0;

  if( datumLibrary )
  {
    char code[10];
    char name[256];
    char ellipsoidCode[3];

    jclass cls;
    jmethodID cid;
    
    try
    {
      datumLibrary->getDatumInfo( (long)index, code, name, ellipsoidCode );
    }
    catch( CoordinateConversionException e )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
    }

    cls = env->FindClass( "geotrans3/misc/Info" );
    if( cls == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Info class not found." );
        return NULL;
    }

    cid = env->GetMethodID( cls, "<init>", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V" );
    if( cid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Info method id not found." );
        return NULL;
    }

    jstring codejStr = env->NewStringUTF( code );
    jstring namejStr = env->NewStringUTF( name );
    jstring ellipsoidCodejStr = env->NewStringUTF( ellipsoidCode );

    info = env->NewObject( cls, cid, codejStr, namejStr, ellipsoidCodejStr );
    if( info == NULL )
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Info object could not be created." );
  }

  return info;
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNIDatumLibrary_jniGetDatumValidRectangle(JNIEnv *env, jobject jobj, jlong datumLibraryPtr, jlong index)
{
  DatumLibrary* datumLibrary = ( DatumLibrary* )datumLibraryPtr;

  jobject aoi = 0;

  if( datumLibrary )
  {
    double westLongitude, eastLongitude, southLatitude, northLatitude;

    jclass cls;
    jmethodID cid;
    
    try
    {
      datumLibrary->getDatumValidRectangle( (long)index, &westLongitude, &eastLongitude, &southLatitude, &northLatitude );
    }
    catch( CoordinateConversionException e )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
    }

    cls = env->FindClass( "geotrans3/misc/AOI" );
    if( cls == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: AOI class not found." );
        return NULL;
    }

    cid = env->GetMethodID( cls, "<init>", "(DDDD)V" );
    if( cid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: AOI method id not found." );
        return NULL;
    }

    aoi = env->NewObject( cls, cid, westLongitude, eastLongitude, southLatitude, northLatitude );
    if( aoi == NULL )
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: AOI object could not be created." );
  }

  return aoi;
}

#ifdef __cplusplus
}
#endif

// CLASSIFICATION: UNCLASSIFIED
