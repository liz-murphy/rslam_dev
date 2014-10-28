// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//  \file PrintMessage.cpp  Printing utility functions.

#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

/** @todo Replace all calls to PrintMessage with GLog */
#ifndef PrintMessage
/// PrintMessage is a macro for efficiency. Calls amount to an
/// extra if, and they can be turned off completely if necessary.
#ifdef SILENCE_RSLAM_PRINT_HANDLER
# define PrintMessage( nErrorLevel, ... )
#else
# define PrintMessage( nErrorLevel, ... )               \
  LOG(nErrorLevel) << rslam::StringFormat(__VA_ARGS__);
#endif


/** @todo Replace all calls to StreamMessage with straight GLog */
#ifndef StreamMessage
#  define StreamMessage(nErrorLevel) LOG(nErrorLevel)
#endif
#endif

namespace rslam {
std::string StringFormat( const char *fmt, ... );

}
