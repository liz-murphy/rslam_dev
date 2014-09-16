// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include "utils/PrintMessage.h"

#include <string>
#include <vector>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

namespace rslam {

std::string StringFormat( const char *fmt, ... ) {
  va_list args;
  va_start( args, fmt );

  if( !fmt ) {
    return "";
  }

  int nResult = INT_MAX;
  int nLength = 2048;
  char *pBuffer = NULL;
  while( nResult >= nLength ) {
    nLength *= 2;

    delete[] pBuffer;
    pBuffer = new char[nLength + 1];

    memset( pBuffer, 0, nLength + 1 );
    nResult = vsnprintf( pBuffer, nLength, fmt, args );
  }
  std::string s( pBuffer );
  delete[] pBuffer;

  va_end(args);
  return s;
}
} // end rslam namespace
