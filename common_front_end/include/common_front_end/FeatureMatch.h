// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef _FEATURE_MATCH_H_
#define _FEATURE_MATCH_H_

// TODO: write SSE version
#include <common_front_end/FeatureImage.h>
#include <vector>
#include <slam_map/Measurement.h>

///
/// \brief FindBestFeatureInRow
/// \param[in] x - search starting point
/// \param[in] y - search starting point
/// \param[in] descriptor - feature descriptor to match
/// \param[in] descsize   - length of the descriptor
/// \param[in] search_width - search area
/// \param[in] search_image - search image
/// \param[out] match_score - best matching score
/// \param[out] match_flag - matching flag
/// \return matched feature if any
///
Feature* FindBestFeatureInRow(
      const float          x,
      const float          y,
      const unsigned char* descriptor,
      const unsigned int   descsize,
      const int            search_width,
      const FeatureImage&  search_image,
      float&               match_score,
      MatchFlag&           match_flag);

///
/// \brief FindBestFeatureInRow
/// \param[in] x - search starting point
/// \param[in] y - search starting point
/// \param[in] descriptor - feature descriptor to match
/// \param[in] descsize   - length of the descriptor
/// \param[in] search_width - search area
/// \param[in] search_height - search area
/// \param[in] search_image - search image
/// \param[in] featureOptions
/// \param[out] match_score - best matching score
/// \param[out] match_flag - matching flag
/// \return matched feature if any
///
Feature* FindBestFeatureInRegion(
    const float          x,
    const float          y,
    const void*          descriptor,
    const unsigned int   descsize,
    const int            search_width,
    const int            search_height,
    const FeatureImage&  search_image,
    const FeatureHandler::Options& featureOptions,
    float&               match_score,
    MatchFlag&           match_flag);


#endif
