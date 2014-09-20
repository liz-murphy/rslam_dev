// CLASSIFICATION: UNCLASSIFIED

#include <jni.h>
#include "string.h"
#include "JNIEllipsoidLibrary.h"
#include "EllipsoidLibrary.h"
#include "CoordinateConversionException.h"
#include "ThrowException.h"


using namespace MSP::CCS;


#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_geotrans3_jni_JNIEllipsoidLibrary_jniDefineEllipsoid(JNIEnv *env, jobject jobj, jlong ellipsoidLibraryPtr, jstring _ellipsoidCode, jstring _ellipsoidName, 
                                                                               jdouble semiMajorAxis, jdouble flattening)
{
  EllipsoidLibrary* ellipsoidLibrary = ( EllipsoidLibrary* )ellipsoidLibraryPtr;

  if( ellipsoidLibrary )
  {
    const char* ellipsoidCode = env->GetStringUTFChars( _ellipsoidCode, NULL );
    if( ellipsoidCode == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid ellipsoid code.");
        return;
    }

    const char* ellipsoidName = env->GetStringUTFChars( _ellipsoidName, NULL );
    if( ellipsoidName == NULL )
    {
        throwException(env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid ellipsoid name.");
        return;
    }

    try
    {
      ellipsoidLibrary->defineEllipsoid( ellipsoidCode, ellipsoidName, semiMajorAxis, flattening );
    }
    catch( CoordinateConversionException e )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
    }
    
    env->ReleaseStringUTFChars( _ellipsoidCode, ellipsoidCode );
    env->ReleaseStringUTFChars( _ellipsoidName, ellipsoidName );
  }
}


JNIEXPORT void JNICALL Java_geotrans3_jni_JNIEllipsoidLibrary_jniRemoveEllipsoid(JNIEnv *env, jobject jobj, jlong ellipsoidLibraryPtr, jstring _ellipsoidCode)
{
  EllipsoidLibrary* ellipsoidLibrary = ( EllipsoidLibrary* )ellipsoidLibraryPtr;

  if( ellipsoidLibrary )
  {
    const char *ellipsoidCode = env->GetStringUTFChars( _ellipsoidCode, NULL) ;
    if( ellipsoidCode == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid ellipsoid code." );
        return;
    }

    try
    {
      ellipsoidLibrary->removeEllipsoid( ellipsoidCode );
    }
    catch( CoordinateConversionException e )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
    }

    env->ReleaseStringUTFChars( _ellipsoidCode, ellipsoidCode );
  }
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIEllipsoidLibrary_jniGetEllipsoidCount(JNIEnv *env, jobject jobj, jlong ellipsoidLibraryPtr)
{
  long count = 0;

  EllipsoidLibrary* ellipsoidLibrary = ( EllipsoidLibrary* )ellipsoidLibraryPtr;

  if( ellipsoidLibrary )
  {
    ellipsoidLibrary->getEllipsoidCount( &count );
  }

  return count;
}


JNIEXPORT jlong JNICALL Java_geotrans3_jni_JNIEllipsoidLibrary_jniGetEllipsoidIndex(JNIEnv *env, jobject jobj, jlong ellipsoidLibraryPtr, jstring _ellipsoidCode)
{
  long ellipsoidIndex = 0;

  EllipsoidLibrary* ellipsoidLibrary = ( EllipsoidLibrary* )ellipsoidLibraryPtr;

  if( ellipsoidLibrary )
  {
    const char* ellipsoidCode = env->GetStringUTFChars( _ellipsoidCode, NULL) ;
    if( ellipsoidCode == NULL )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Invalid ellipsoid code." );
      return ellipsoidIndex;
    }

    try
    {
      ellipsoidLibrary->getEllipsoidIndex( ellipsoidCode, &ellipsoidIndex );
    }
    catch( CoordinateConversionException e )
    {
      throwException( env, "geotrans3/exception/CoordinateConversionException", e.getMessage() );
    }

    env->ReleaseStringUTFChars( _ellipsoidCode, ellipsoidCode );
  }

  return ellipsoidIndex;
}


JNIEXPORT jobject JNICALL Java_geotrans3_jni_JNIEllipsoidLibrary_jniGetEllipsoidInfo(JNIEnv *env, jobject jobj, jlong ellipsoidLibraryPtr, jlong index)
{
  EllipsoidLibrary* ellipsoidLibrary = ( EllipsoidLibrary* )ellipsoidLibraryPtr;

  jobject info = 0;

  if( ellipsoidLibrary )
  {
    char code[3];
    char name[30];

    jclass cls;
    jmethodID cid;
    
    try
    {
      ellipsoidLibrary->getEllipsoidInfo( (long)index, code, name );
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

    cid = env->GetMethodID( cls, "<init>", "(Ljava/lang/String;Ljava/lang/String;)V" );
    if( cid == NULL )
    {
        throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Info method id not found." );
        return NULL;
    }

    jstring codejStr = env->NewStringUTF( code );
    jstring namejStr = env->NewStringUTF( name );

    info = env->NewObject( cls, cid, codejStr, namejStr );
    if( info == NULL )
      throwException( env, "geotrans3/exception/CoordinateConversionException", "JNI Exception: Info object could not be created." );
  }

  return info;
}


#ifdef __cplusplus
}
#endif

// CLASSIFICATION: UNCLASSIFIED
