// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef _PATCH_MATCH_H_
#define _PATCH_MATCH_H_

// TODO: write SSE version
#include <feature_utils/FeatureImage.h>
#include <vector>
#include <feature_utils/MatchFlags.h>

///////////////////////////////////////////////////////////////////////////////
Feature* FindBestPatchInRow(
      const float x,                  ///< Input:
      const float y,                  ///< Input:
      const unsigned char* pPatch,    ///< Input:
      const unsigned int nPatchSize,  ///< Input:
      const int nSearchWidth,  		///< Input: Search width delta
      FeatureImage& SearchImage,      ///< Input: Image we search in
      float& fMatchScore,             ///< Output: Match score
      MatchFlag& eFlag                ///< Output: Match flag
      );

///////////////////////////////////////////////////////////////////////////////
template<unsigned int PatchSize>
Feature* FindBestPatchInRegion(
        const PatchHomography<PatchSize>& PredictedH,//< Input: where and what shape patch to match
        const unsigned char* pRefPatch,                      //< Input:
        const int nSearchWidth,           //< Input: Search width delta
        const int nSearchHeight,          //< Input: Search height delta
        const FeatureImage& rSearchImage,       //< Input: Image we search in
        float& fMatchScore,
        MatchFlag& eFlag
        );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool LoadPatch(
        const unsigned char* pImage,      //< Input:
        const unsigned int nImageWidth,   //< Input:
        const unsigned int nImageHeight,  //< Input:
        const unsigned int nPatchPosX,    //< Input:
        const unsigned int nPatchPosY,    //< Input:
        unsigned char* pPatch,			  //< Output:
        const unsigned int nPatchWidth,   //< Input:
        const unsigned int nPatchHeight   //< Input:
        );

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//float _interp(
//        float x,						//< Input: X coordinate
//        float y,						//< Input: Y coordinate
//        const unsigned char* pImage,	//< Input: Pointer to Image
//        int ImageWidth,					//< Input: Image width
//        int ImageHeight					//< Input: Image height
//        );


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool LoadInterpolatedPatch(
        const unsigned char* pImage,     //< Input:
        const unsigned int nImageWidth,  //< Input:
        const unsigned int nImageHeight, //< Input:
        const float nPatchPosX,    		//< Input:
        const float nPatchPosY,    		//< Input:
        unsigned char* pPatch,           //< Output:
        const unsigned int nPatchWidth,
        const unsigned int nPatchHeight
        );


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float ScorePatchesNCC(const unsigned char* patch_a,
                      const unsigned char* patch_b,
                      const unsigned int   patch_width,
                      const unsigned int   patch_height);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float ScorePatchesSAD(const unsigned char* patch_a,       //< Input:
                      const unsigned char* patch_b,       //< Input:
                      const unsigned int patch_width,
                      const unsigned int patch_height);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float ScorePatchesMeanSAD(const unsigned char* patch_a,        //< Input:
        const unsigned char* patch_b,        //< Input:
        const unsigned int   patch_width,
        const unsigned int   patch_height
        );

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// SAD with mean subtraction
float ScorePatchesMS(const unsigned char* patch_a,       //< Input:
        const unsigned char* patch_b,       //< Input:
        const unsigned int   patch_width,
        const unsigned int   patch_height
        );

#endif
