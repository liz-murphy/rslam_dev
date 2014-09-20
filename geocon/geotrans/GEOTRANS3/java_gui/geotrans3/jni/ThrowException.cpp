// CLASSIFICATION: UNCLASSIFIED

#include <jni.h>
#include "ThrowException.h"



void throwException(JNIEnv* env, const char *name, const char *msg)
{
  jclass cls = NULL;

  env->ExceptionClear();

  cls = env->FindClass( name );
  if( cls != NULL )
    env->ThrowNew ( cls, msg );

  env->DeleteLocalRef( cls );
}




// CLASSIFICATION: UNCLASSIFIED
