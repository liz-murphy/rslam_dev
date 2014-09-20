// CLASSIFICATION: UNCLASSIFIED

#ifndef WarningMessages_H
#define WarningMessages_H


/**
 * Defines all warning messages which may be returned by the ccs. 
 * 
 * @author comstam
 */

#include "DtccApi.h"

namespace MSP
{
  namespace CCS
  {
    class MSP_DTCC_API WarningMessages
    {  
      public:

        // Coordinate warning messages
        static char* longitude;
        static char* latitude;

        static char* datum;

    };
  }
}
	
#endif 

// CLASSIFICATION: UNCLASSIFIED
