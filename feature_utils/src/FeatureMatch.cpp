// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <feature_utils/FeatureMatch.h>
#include <opencv2/features2d/features2d.hpp>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <vector>
#include <feature_utils/FeatureParams.h>
#include <feature_utils/MatchFlags.h>

///////////////////////////////////////////////////////////////////////////////
Feature*  FindBestFeatureInRow(const float           x,
                               const float           y,
                               const unsigned char* descriptor,
                               const unsigned int,  /* descsize,*/
                               const int            search_width,
                               const FeatureImage&  search_image,
                               float&               match_score,
                               MatchFlag&           match_flag)
{
  Feature* featOut          = NULL;
  const int image_width     = search_image.Width();
  const int image_height    = search_image.Height();
  const int search_row      = (int)round(y);
  const float min_disparity = 0.0; // this is a hack -- what about good features at infinity?
  const float max_error_y   = 1.0; // this is a hack
  int best_score            = 512; // this is a hack
  float dbest_score         = DBL_MAX;
  float left_boundary, right_boundary;

  if (search_width < 0) {
    // if SearchWidth is negative, search to the left
    left_boundary  = x + search_width;
    right_boundary = x - min_disparity;
  } else {
    // otherwise search to the right
    left_boundary  = x + min_disparity;
    right_boundary = x + search_width;
  }

  // Whoa, subtle bug!  Say a feature projects to x=209.8,
  // check search boundaries and re-adjust limits if needed
  if (left_boundary < 0) {
    left_boundary = 0.0;
  }
  if (right_boundary >= image_width) {
    right_boundary = image_width - 1.0;
  }

  // features may have subpixel precision but for storage
  // the "y" coordinate was rounded. We should  check in one/two extra row(s)
  // assuming  0.5 pixel noise in the measurements.
  //    int nTop = roundf(y);
  //    int nBot = roundf(y);

  int top_boudnary = search_row - 1;
  int bot_boundary = search_row + 1;

  if (top_boudnary < 0)            { top_boudnary = 0; }
  if (bot_boundary >= image_height) { bot_boundary = image_height-1; }

  /*
    if( roundf(y) == y ) {
    nTop = inRow - 1;
    nBot = inRow + 1;
    } else {
    if( y < inRow ){
    nTop = inRow - 1;
    nBot = inRow;
    }else{
    nTop = inRow;
    nBot = inRow + 1;
    }
    }
  */

#ifndef ANDROID
  cv::Hamming hsse;
  cv::L2<float> sl2;
#endif
  for (int nRow = top_boudnary;  nRow <= bot_boundary; ++nRow ) {
    const std::vector< FeaturePtr >& Features = search_image.RowVectorRef(nRow);

    for (auto it = Features.begin(); it != Features.end(); ++it ) {
      if ((*it)->x > left_boundary &&
          (*it)->x < right_boundary &&
          !(*it)->used ) {
        if (fabs(y - (*it)->y) <= max_error_y) {
#ifndef ANDROID
          if(FeatureParams::getParams()->getFeatureDescriptor() == feature_utils::Feature_SURF) {
            float score = sl2(
                reinterpret_cast<const float*>(descriptor),
                reinterpret_cast<const float*>(&(*it)->descriptor[0]),
                (*it)->descsize/4);
            if( score < dbest_score ) {
              featOut = (*it).get();
              dbest_score = score;
            }
          }
          else { // Hamming distance
            int score = hsse(descriptor,
                             &(*it)->descriptor[0], (*it)->descsize);
            if( score < best_score ) {
              featOut    = (*it).get();
              best_score = score;
            }
          }
#endif
        }
      }
    }
  }

  if(FeatureParams::getParams()->getFeatureDescriptor() == feature_utils::Feature_SURF) {
    match_score = (float)dbest_score;
    if( best_score == DBL_MAX ){
      match_flag = NoFeaturesToMatch;  // no features found in search window
    }
    else if( dbest_score > FeatureParams::getParams()->getFeatureMatchingThreshold() ) {
      match_flag  = NoMatchOnLine;
    }
    else{
      match_flag = GoodMatch;
    }
  }
  else{
    match_score = best_score;
    if( best_score == 512 ){
      match_flag = NoFeaturesToMatch;  // no features found in search window
    }
    else if( best_score > FeatureParams::getParams()->getFeatureMatchingThreshold() ) {
      match_flag  = NoMatchOnLine;
    }
    else{
      match_flag = GoodMatch;
    }
  }
  return featOut; 
}

///////////////////////////////////////////////////////////////////////////////
Feature* FindBestFeatureInRegion(
    const float                    x,
    const float                    y,
    const void*                    descriptor,
    const unsigned int ,           //descsize,
    const int                      search_width,
    const int                      search_height,
    const FeatureImage&            search_image,
    float&                         match_score,
    MatchFlag&                     match_flag
                                 )
{

  Feature* featOut = NULL;

  const int image_width  = search_image.Width();
  const int image_height = search_image.Height();
  const int search_col   = round(x);
  const int search_row   = round(y);
  int top_boundary       = search_row - search_height;
  int bot_boundary       = search_row + search_height;
  int left_boundary      = search_col - search_width;
  int right__boundary    = search_col + search_width;
  double nScore;

  PrintMessage( 1, "    Find feature search dims [%d %d]\n",search_width,search_height);
  PrintMessage( 1, "    Find feature search area [%d %d %d %d]\n",top_boundary,bot_boundary,left_boundary,right__boundary);

  double nBestScore = DBL_MAX;
  double nNextBestScore = DBL_MAX;

  // check search boundaries and re-adjust limits if needed
  if( left_boundary < 0 ) left_boundary = 0;
  if( top_boundary < 0 )  top_boundary  = 0;

  if( right__boundary >= image_width ) right__boundary = image_width - 1;
  if( bot_boundary   >= image_height ) bot_boundary    = image_height - 1;

#ifndef ANDROID
  cv::Hamming hsse; // TODO
#endif
  cv::L2<float> sl2;

  for (int v = top_boundary; v <= bot_boundary; ++v) {
    const std::vector< FeaturePtr >& Features = search_image.RowVectorRef(v);
    for (auto it = Features.begin(); it != Features.end(); ++it) {
      if ((*it)->x >= left_boundary &&
          (*it)->x <= right__boundary &&
          !(*it)->used) {
        if (FeatureParams::getParams()->getFeatureDescriptor() == feature_utils::Feature_SURF) { // Returns 0 on equal
          nScore = sl2((float*)descriptor,
                       (float *)&(*it)->descriptor[0],
                       (*it)->descsize/4);
        } else { // Hamming distance
#ifndef ANDROID
          nScore = hsse((const unsigned char *)descriptor,
                        (const unsigned char *)&(*it)->descriptor[0],
                        (*it)->descsize);
#endif
        }
        if (nScore < nBestScore) {
          PrintMessage( 1, "    Find feature in region [%.3f %.3f], testing [%.3f %.3f] score: %.2f descSize: %u ]\n",
                        x, y, (*it)->x, (*it)->y, nScore, (*it)->descsize/4 );
          featOut   = (*it).get();
          nNextBestScore = nBestScore;
          nBestScore = nScore;
        }
      }
    }
  }

  match_score = nBestScore;

  if( nBestScore == 512 ){
    match_flag = NoFeaturesToMatch; // no features found in search window
  }
  else if( nBestScore > FeatureParams::getParams()->getFeatureMatchingThreshold() ) { // this number is super important!
    match_flag  = NoMatchInRegion; // matched feature is not good enough
  }
  else if( nBestScore * FeatureParams::getParams()->getMatchErrorFactor() >= nNextBestScore ) {
    match_flag  = AmbiguousMatch;
  }
  else{
    // we found a match!
    match_flag = GoodMatch;
  }

  return featOut;
}
