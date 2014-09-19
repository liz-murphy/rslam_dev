// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <utils/PrintMessage.h>
#include <features/Features.h>
#include <common_front_end/PatchMatch.h>
//#include <common_front_end/CommonFrontEndCVars.h>
#include <common_front_end/CommonFrontEndConfig.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <vector>

//using namespace rslam;

////////////////////////////////////////////////////////////////////////////////
float ScorePatchesNCC(const unsigned char* patch_a,
                      const unsigned char* patch_b,
                      const unsigned int   patch_width,
                      const unsigned int   patch_height)
{
  float mean_a  = 0.0f;
  float mean_b  = 0.0f;

  unsigned int size = patch_width*patch_height;

  for (unsigned int ii = 0; ii < size; ++ii) {
    mean_a += patch_a[ii];
    mean_b += patch_b[ii];
  }

  mean_a /= size;
  mean_b /= size;

  float denA = 0.0f;
  float denB = 0.0f;
  float num = 0.0f;

  for (unsigned int ii = 0; ii < size; ++ii) {
    float fA = patch_a[ii] - mean_a;
    float fB = patch_b[ii] - mean_b;
    num  += fA*fB;
    denA += fA*fA;
    denB += fB*fB;
  }

  float den = sqrt(denA*denB);

  return num/den;
}

////////////////////////////////////////////////////////////////////////////////
float ScorePatchesSAD(const unsigned char* patch_a,
                      const unsigned char* patch_b,
                      const unsigned int   patch_width,
                      const unsigned int   patch_height)
{
  unsigned int score = 0;
  unsigned int size=patch_width*patch_height;
  for (unsigned int ii = 0; ii < size; ++ii) {
    score += abs(patch_a[ii] - patch_b[ii]);
  }
  return (float)score/size;
}

////////////////////////////////////////////////////////////////////////////////
// SAD with mean subtraction
float ScorePatchesMS(const unsigned char* patch_a,
                     const unsigned char* patch_b,
                     const unsigned int   patch_width,
                     const unsigned int   patch_height
                     )
{

  int   size  = (patch_width*patch_height);
  float mean_a  = 0.0;
  float meab_b  = 0.0;

  for (int ii = 0; ii < size; ++ii) {
    mean_a += patch_a[ii];
    meab_b += patch_b[ii];
  }

  mean_a /= size;
  meab_b /= size;

  float score = 0.0;

  for (int ii = 0; ii < size; ++ii) {
    score += fabs(((float)patch_a[ii]-mean_a) - ((float)patch_b[ii]-meab_b));
  }

  return score/size;
}

////////////////////////////////////////////////////////////////////////////////
// SAD with mean subtraction
float ScorePatchesMeanSAD(const unsigned char* patch_a,
                          const unsigned char* patch_b,
                          const unsigned int   patch_width,
                          const unsigned int   patch_height)
{

  int size  = patch_width*patch_height;
  int mean  = 0;

  for (int ii = 0; ii < size; ++ii) {
    mean += (patch_a[ii] - patch_b[ii]);
  }

  mean /= size;
  int sad   = 0;

  for (int ii = 0; ii < size; ++ii) {
    sad += abs( patch_a[ii] - patch_b[ii] - mean );
  }

  return (float)sad/size;
}



///////////////////////////////////////////////////////////////////////////////
Feature* FindBestPatchInRow(
    const float x,                  //< Input:
    const float y,                  //< Input:
    const unsigned char* pRefPatch, //< Input:
    const unsigned int nPatchSize,  //< Input:
    const int nSearchWidth,  		//< Input: Search width delta
    FeatureImage& rSearchImage,     //< Input: Image we search in
    float& fMatchScore,             //< Output: Match score
    MatchFlag& eFlag                //< Output: Match flag [GOOD, BAD or NO match]
    )
{

  Feature*    featOut       = NULL;
  const int   nImageWidth   = rSearchImage.Width();
  const int   nImageHeight  = rSearchImage.Height();
  const int   inRow         = (int)round(y);
  const float fMinDisparity = 0.0f; // this is a hack -- what about good features at infinity?
  const float fMaxErrorY    = 1.0f; // this is a hack
  float       fBestScore    = FLT_MAX; // this is a hack
  float       fSecondBest   = FLT_MAX;
  float       fScore;
  float       fLeft;
  float       fRight;

  if( nSearchWidth < 0 ) {
    // if SearchWidth is negative, search to the left
    fLeft  = x + nSearchWidth;
    fRight = x - fMinDisparity;
  }
  else {
    // otherwise search to the right
    fLeft  = x + fMinDisparity;
    fRight = x + nSearchWidth;
  }

  // Whoa, subtle bug!  Say a feature projects to x=209.8,

  // check search boundaries and re-adjust limits if needed
  if( fLeft < 0.0f ){
    fLeft = 0.0f;
  }
  if( fRight >= nImageWidth ){
    fRight = nImageWidth - 1.0f;
  }

  // features may have subpixel precision but for storage
  // the "y" coordinate was rounded. We should  check in one/two extra row(s)
  // assuming  0.5 pixel noise in the measurements.
  //int nTop = roundf(y);
  //int nBot = roundf(y);

  int nTop = inRow - 2;
  int nBot = inRow + 2;

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
  if( nTop < 0 )            { nTop = 0; }
  if( nBot >= nImageHeight) { nBot = nImageHeight-1; }

  int verbose = 0;

  unsigned char pPatch[144]; // mem for patch matching
  assert( nPatchSize <= 12 ); // patch not too big

  std::vector< FeaturePtr >::iterator it;
  for( int nRow = nTop;  nRow <= nBot; ++nRow ) {
    std::vector< FeaturePtr >& Features = rSearchImage.RowVectorRef(nRow);
    PrintMessage(verbose,"Row: %d #feats: %d \n", nRow, Features.size() );
    for( it = Features.begin(); it != Features.end(); ++it ) {
      PrintMessage(verbose,"    Feat: [%.2f %.2f]", (*it)->x, (*it)->y);
      if( (*it)->x >= fLeft && (*it)->x <= fRight && !(*it)->used ){
        if( fabs(y - (*it)->y) <= fMaxErrorY ) {

          PrintMessage(verbose,"  -- testing");
          //=======================================================
          // Compute coordinates at the appropiate scale for
          // correct patch matching
          //=======================================================
          float px = (*it)->x;
          float py = (*it)->y;

          assert(0); // this code is old -- use LoadPatch in the feature image...
          LoadInterpolatedPatch(
                rSearchImage.ImageData(),
                rSearchImage.Width(),
                rSearchImage.Height(),
                px,
                py,
                pPatch,
                nPatchSize,
                nPatchSize
                );

          fScore = ScorePatchesMeanSAD( pRefPatch, pPatch, nPatchSize, nPatchSize );

          PrintMessage(verbose,"  -- score: %.3f",fScore);

          if( fScore < fBestScore ) {
            fSecondBest = fBestScore;
            featOut     = (*it).get();
            fBestScore  = fScore;
          }else if( fScore < fSecondBest ){
            fSecondBest = fScore;
          }
        }
      }
      PrintMessage(verbose,"\n");
    }
  }
  fMatchScore = fBestScore;
  if( fBestScore == FLT_MAX ){
    eFlag = NoFeaturesToMatch;
  }
  else if( fBestScore > CommonFrontEndConfig::getConfig()->match_error_threshold) {
    eFlag  = NoMatchOnLine;
  }
  else if( fBestScore*CommonFrontEndConfig::getConfig()->match_error_factor >= fSecondBest ) {
    eFlag  = NoMatchOnLine;
  }
  else{
    eFlag = GoodMatch;
  }
  return featOut;
}


////////////////////////////////////////////////////////////////////////////////
template<unsigned int PatchSize>
Feature* FindBestPatchInRegion(
    const PatchHomography<PatchSize>& predicted_h,    //< Input: where and what shape patch to match
    const unsigned char*              referece_patch,
    const int                         search_width,
    const int                         search_height,
    const FeatureImage&               search_image,
    float&                            match_score,
    MatchFlag&                        match_flag
    )
{
  Feature* feat_out       = nullptr;
  Feature* second_feat    = nullptr;
  const int image_width   = search_image.Width();
  const int image_height  = search_image.Height();
  const int search_col    = round( predicted_h.CenterPixel()[0] );
  const int search_row    = round( predicted_h.CenterPixel()[1] );
  int top_boundary        = search_row - search_height;
  int bot_boundary        = search_row + search_height;
  int left_boundary       = search_col - search_width;
  int right_boundary      = search_col + search_width;
  float score;

  float best_score  = FLT_MAX;
  float second_best_score = FLT_MAX;

  // check search boundaries and re-adjust limits if needed
  if (left_boundary < 0)              { left_boundary  = 0; }
  if (top_boundary < 0)               { top_boundary   = 0; }
  if (right_boundary >= image_width)  { right_boundary = image_width - 1;  }
  if (bot_boundary   >= image_height) { bot_boundary   = image_height - 1; }


  std::vector<unsigned char> patch( PatchSize*PatchSize );
  unsigned char* patch_ptr = &patch[0]; // mem for patch matching

  // compute the homography outside the for loops.
  // The only thing that changes is the position
  // while the scale and orientation remain the same.
  PatchHomography<PatchSize> homography;
  unsigned int sampling_level;
  float        scale_factor_at_level;
  predicted_h.GetSamplingHomographyAndLevel(
        search_image.GetLevelFactor(),
        search_image.GetNumLevels(),
        homography, sampling_level );
  scale_factor_at_level = search_image.GetFactorAtLevel(sampling_level);

  std::vector< FeaturePtr >::const_iterator it;
  for (int row = top_boundary; row <= bot_boundary; ++row) {
    const std::vector< FeaturePtr >& Features = search_image.RowVectorRef(row);
    for( it = Features.begin(); it != Features.end(); ++it ) {
      if( (*it)->x >= left_boundary &&
          (*it)->x <= right_boundary &&
          !(*it)->used ) {

        const float x = ((*it)->x + 0.5)*scale_factor_at_level - 0.5;
        const float y = ((*it)->y + 0.5)*scale_factor_at_level - 0.5;
        homography.SetTranslation(x, y);   // re-center H

        // load patch at patch scale
        if(search_image.LoadPatchAtLevel( homography,
                                          sampling_level,
                                          patch_ptr )) {

          score = ScorePatchesMeanSAD(referece_patch, patch_ptr,
                                       PatchSize, PatchSize);
        }else{
          continue;
        }

        if (score < best_score) {
          second_best_score = best_score;
          second_feat = feat_out;
          feat_out    = (*it).get();
          best_score  = score;
        } else if (score < second_best_score ) {
          second_best_score = score;
          second_feat = (*it).get();;
        }
      }
    }
  }

  //std::cout << "Window size [ " << nSearchWidth << " " << nSearchHeight << "] - Feat tested: " << uFeatTested << std::endl;

  match_score = best_score;
  if (best_score == FLT_MAX) {
    match_flag = NoFeaturesToMatch;
  } else if (best_score > CommonFrontEndConfig::getConfig()->match_error_threshold) {
    match_flag  = NoMatchInRegion;
  } else if (best_score*CommonFrontEndConfig::getConfig()->match_error_factor >= second_best_score
             && second_feat
             && fabs(feat_out->x - second_feat->x) > 2.0
             && fabs(feat_out->y - second_feat->y) > 2.0 ) {
    // only flag ambiguous if the feature is really at a different location
    // else ESM will fix its location...
    match_flag  = AmbiguousMatch;
  }
  else{
    match_flag = GoodMatch;
  }
  return feat_out;
}


///////////////////////////////////////////////////////////////////////////////
// Specialization, commented out for speed
template Feature* FindBestPatchInRegion<CANONICAL_PATCH_SIZE>(
    const PatchHomography<CANONICAL_PATCH_SIZE>& PredictedH,//< Input: where and what shape patch to match
const unsigned char* pRefPatch,                      //< Input:
const int nSearchWidth,           //< Input: Search width delta
const int nSearchHeight,          //< Input: Search height delta
const FeatureImage& rSearchImage,       //< Input: Image we search in
float& fMatchScore,
MatchFlag& eFlag
);
