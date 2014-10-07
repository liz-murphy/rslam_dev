// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <common_front_end/PatchMatch.h>
#include <common_front_end/CommonFrontEndParamsConfig.h>
#include <common_front_end/CommonFrontEndConfig.h>
#include <common_front_end/FeatureMatch.h>
#include <slam_map/PatchHomography.h>
#include <utils/PrintMessage.h>
#include <utils/MathTypes.h>
#include <utils/Utils.h>

#include <algorithm>

#include <ros/ros.h>

//#define DEBUG_HELPERS 0

template<unsigned int PatchSize=CANONICAL_PATCH_SIZE>
struct FeatureMatches
{
  std::vector<Feature*>                    vFeatures;
  std::vector<Scalar>                      vErrors;
  std::vector<MatchFlag>                   vFlags;
  std::vector<PatchHomography<PatchSize> > vH;
  // std::vector<std::vector<unsigned char>>       vPatches;   // patches for every matched feature
};

/// \brief GetBestSimMatch
/// \param id
/// \param search_image
/// \param roi_top
/// \param roi_bottom
/// \param roi_left
/// \param roi_right
/// \param match_score
/// \param match_flag
/// \return
///
inline Feature* GetBestSimMatch(const int feature_id,
                                const FeatureImage& search_image,
                                 int roi_top,
                                 int roi_bottom,
                                 int roi_left,
                                 int roi_right,
                                float&    match_score,
                                MatchFlag& match_flag)
{
  Feature* match = nullptr;

  //[TEST] remove this
  roi_top = 0;
  roi_left = 0;
  roi_bottom = search_image.Height();
  roi_right = search_image.Width();

  if (feature_id >= 0) {
    for (int row = roi_top; row <= roi_bottom; ++row) {
      const std::vector<FeaturePtr>& features = search_image.RowVectorRef(row);
      for (const FeaturePtr& pf : features) {
        if( pf->x < (float)roi_left || pf->x > roi_right ) continue;
        if (pf->id == feature_id) {
          match = pf.get();
          break;
        }
      }
      if (match) {
        break;
      }
    }
  }

  match_score = 0.0;

  if (match) {
    match_flag = GoodMatch;
  }else{
    match_flag = NoFeaturesToMatch;
  }

  return match;
}

///////////////////////////////////////////////////////////////////////////////
inline Feature* GetBestTrack2dMatch(const int            nId,
                                   const FeatureImage&  rSearchImage,
                                   float&               fMatchScore,
                                   MatchFlag&           eFlag)
{

  Feature* pMatch = nullptr;

  if (nId >= 0) {
    for (int row = 0; row < rSearchImage.Height(); ++row) {
      const std::vector<FeaturePtr>& vFeatures = rSearchImage.RowVectorRef(row);
      for (const FeaturePtr& pf : vFeatures) {
        if (pf->id == nId) {
          pMatch = pf.get();
          break;
        }
      }
      if (pMatch) {
        break;
      }
    }
  }

  fMatchScore = 0.0;

  if (pMatch) {
    eFlag = GoodMatch;
  }else{
    eFlag = NoFeaturesToMatch;
  }

  return pMatch;

}


////////////////////////////////////////////////////////////////////////////////
// wrapper around feature based and patch based calls
template<unsigned int PatchSize>
inline Feature* FindBestMatchInRegion(
    const int                        feature_id,      //< Input: only valid/used for track2d
    const PatchHomography<PatchSize> &H,              //< Input: where and what shape patch to match
    const unsigned char              *descriptor,     //< Input:
    const unsigned int               descsize,        //< Input:
    const unsigned char              *pPatch,         //< Input: Reference patch
    const int                        search_width,    //< Input: Search width delta
    const int                        search_height,   //< Input: Search height delta
    const FeatureImage               &search_image,    //< Input: Image we search in
    const FeatureHandler::Options    &feature_options,
    float                            &match_score,
    MatchFlag                        &match_flag
    ) {
  if (CommonFrontEndConfig::getConfig()->feature_descriptor== common_front_end::CommonFrontEndParams_TRACK_2D) {
    return GetBestTrack2dMatch( feature_id, search_image, match_score, match_flag );
  } 
  if( CommonFrontEndConfig::getConfig()->feature_descriptor== common_front_end::CommonFrontEndParams_SIMULATION ) {

    int search_col  = round( H.CenterPixel()[0] );
    int search_row  = round( H.CenterPixel()[1] );
    int roi_top     = search_row - search_height;
    int roi_bottom  = search_row + search_height;
    int roi_left    = search_col - search_width;
    int roi_right   = search_col + search_width;
    int image_width = search_image.Width();
    int image_height = search_image.Height();
    if (roi_left < 0)               { roi_left  = 0; }
    if (roi_top < 0)                { roi_top  = 0; }
    if (roi_right >= image_width)   { roi_right = image_width - 1;  }
    if (roi_bottom >= image_height) { roi_bottom = image_height - 1; }

    return GetBestSimMatch( feature_id,
                            search_image,
                            roi_top,
                            roi_bottom,
                            roi_left,
                            roi_right,
                            match_score,
                            match_flag);


  } else if(CommonFrontEndConfig::getConfig()->feature_descriptor== common_front_end::CommonFrontEndParams_PATCH ) {
    return FindBestPatchInRegion<PatchSize>( H,
                                             pPatch,
                                             search_width,
                                             search_height,
                                             search_image,
                                             match_score,
                                             match_flag );
  }
  else{
    // else match feature descriptors
    return FindBestFeatureInRegion( H.CenterPixel()[0],
                                    H.CenterPixel()[1],
                                    descriptor,
                                    descsize,
                                    search_width,
                                    search_height,
                                    search_image,
                                    feature_options,
                                    match_score,
                                    match_flag );
  }
}


///////////////////////////////////////////////////////////////////////////////
inline Feature* FindBestMatchInRow(
    const float x,                  //< Input:
    const float y,                  //< Input:
    const unsigned char* descriptor,//< Input:
    const unsigned int descsize,    //< Input:
    const unsigned char* pPatch,    //< Input:
    const unsigned int patchsize,   //< Input:
    const int nSearchWidth,         //< Input: Search width delta
    FeatureImage& SearchImage,      //< Input: Image we search in
    float& fMatchScore,             //< Output: Match score
    MatchFlag& eFlag                //< Output: Match flag [GOOD, BAD or NO match]
    )
{
  if(CommonFrontEndConfig::getConfig()->feature_detector == common_front_end::CommonFrontEndParams_PATCH ){
    return FindBestPatchInRow( x, y, pPatch, patchsize,
                               nSearchWidth, SearchImage, fMatchScore, eFlag );
  }
  return FindBestFeatureInRow( x, y, descriptor, descsize,
                               nSearchWidth, SearchImage, fMatchScore, eFlag );
}


///////////////////////////////////////////////////////////////////////////////
/// Given rHRef defining a patch warp into an nxn patch in the ref image,
/// compute homography H for loading an nxn patch in level 0 of search image.
template<unsigned int PatchSize=CANONICAL_PATCH_SIZE>
inline bool GetHomographyIfOnEpipolarLine(
    const PatchHomography<PatchSize>         &rHRef,
    const Eigen::Vector3t                    &rRefRay_sp,
    const Eigen::Vector2t                    &rSearchPix,
    const calibu::CameraModelGeneric<Scalar> &rSearchCam,
    const calibu::CameraModelGeneric<Scalar> &rRefCam,
    const Sophus::SE3t                       &Tsr,
    PatchHomography<PatchSize>               &H           //< Output:
    )
{
  const Eigen::Vector3t rSearchRay_sp = rSearchCam.Unproject( rSearchPix );
  const Eigen::Vector3t Xsp = RayRayIntersect( rRefRay_sp, Tsr.translation(), rSearchRay_sp );
  const Eigen::Vector2t xsp = rSearchCam.Project( Xsp );
  const double dReprojError = (rSearchPix - xsp).norm();
  if (dReprojError > 1.5) {
    return false;
  }
  const float fRho_ref = 1.0 / (Tsr.inverse()*Xsp)[2]; //  inv z-depth from ref cam, in ref Coords
  //     H = rHRef.TransferPatchSimilarityHomography( rSearchPix, rSearchCam,rRefCam, Tsr, fRho_ref );
  H = rHRef.TransferPatchHomography( rSearchCam,rRefCam, Tsr, fRho_ref );

  return true;
}

///////////////////////////////////////////////////////////////////////////////
///
/// \brief FindBestPatchOnEpipolarLine - Goal here is to use known epi-polar
/// geometry to arp rRefFeature to the search camera patch and then
/// score that match.
/// \param[in] ref_feature, Feature in the reference camera
/// \param[in] ref_ray_sp, ray from ref cam IN SEARCH FRAME
/// \param[in] ref_homography, Homography to extract reference patch
/// \param[in] Tsr, Reference to Search camera transform
/// \param[in] ref_cam, Camera the reference patch came from
/// \param[in] search_cam, Camera we are looking to match in
/// \param[in] ref_patch, Pointer to reference patch data
/// \param[in] patch_size, Patch size
/// \param[in] search_image, Image we search in
/// \param[out] match_score, matching score (SAD)
/// \param[out] match_flag, flag indicating matching status
/// \param[out] match_homography,  homography for best match
/// \return pointer to matched feature (nullptr is no match is found)
///
template<unsigned int PatchSize=CANONICAL_PATCH_SIZE>
inline Feature* FindBestPatchOnEpipolarLine(
    const Feature                             &ref_feature,
    const Eigen::Vector3t                     &ref_ray_sp,
    const PatchHomography<PatchSize>          &ref_homography,
    const Sophus::SE3Group<Scalar>            &Tsr,
    const calibu::CameraModelGeneric<Scalar>  &ref_cam,
    const calibu::CameraModelGeneric<Scalar>  &search_cam,
    const unsigned char*                      ref_patch,
    const unsigned int                        patch_size,
    FeatureImage                              &search_image,
    float                                     &match_score,
    MatchFlag                                 &match_flag,
    PatchHomography<PatchSize>                &match_homography,
    const FeatureHandler::Options             &feature_options)
{

  bool is_simulation = feature_options.feature_detector == SIMULATION;

  Eigen::Vector2tArray pts =
      GetEpiPolarLine(ref_cam, search_cam, Tsr,
                      Eigen::Vector2t(ref_feature.x,ref_feature.y) );


  // get region to search in
  float xmax = patch_size/2;
  float xmin = search_image.Width() - patch_size/2;
  float ymax = patch_size/2;
  float ymin=search_image.Height() - patch_size/2;
  for (const Eigen::Vector2t& p : pts) {
    if (p[0] > xmax) { xmax = p[0]; }
    if (p[0] < xmin) { xmin = p[0]; }
    if (p[1] > ymax) { ymax = p[1]; }
    if (p[1] < ymin) { ymin = p[1]; }
  }
  xmax = std::min( xmax, search_image.Width()-patch_size/2.0f );
  xmin = std::max( xmin, patch_size/2.0f );
  ymax = std::min( ymax, search_image.Height()-patch_size/2.0f );
  ymin = std::max( ymin, patch_size/2.0f );

  Feature* match  = nullptr;

  const int pad = 1;
  const int verb = 1;
  float best_score  = FLT_MAX;
  float second_best = FLT_MAX;
  float score;

  std::vector<unsigned char> patch(patch_size*patch_size);
  unsigned char* pPatch = &patch[0];

  for (unsigned int row = round(ymin)-pad; row <= round(ymax)+pad; row++) {
    const std::vector<FeaturePtr>& vFeatures = search_image.RowVectorRef(row);

    for (const FeaturePtr& pf : vFeatures) {
      if (pf->x > xmax+pad || pf->x < xmin-pad) {
        continue;
      }

      const Eigen::Vector2t rSearchPix = Eigen::Vector2t(pf->x,pf->y);

      PrintMessage( verb, "        Checking [%.2f, %.2f, s(%.2f)] "
                    "and [%.2f, %.2f, s(%.2f)]  ",
                    ref_feature.x, ref_feature.y, ref_feature.scale,
                    pf->x, pf->y, pf->scale ); fflush(stdout);

      // should always enforce epipolar geometry as
      // GetHomographyIfOnEpipolarLine returns junk H if not on epipolar line
      PatchHomography<PatchSize> H;
      bool bRes =
          GetHomographyIfOnEpipolarLine<PatchSize>(ref_homography,
                                                   ref_ray_sp,
                                                   rSearchPix,
                                                   search_cam,
                                                   ref_cam,
                                                   Tsr,
                                                   H);
      if (!bRes) {
        PrintMessage( verb,
                      "             Epipolar line check fail, skipping\n" );
        fflush(stdout);
        continue;
      }

      if (is_simulation) {

        if( pf->id == ref_feature.id){
          match = pf.get();
          match_homography = H;
          best_score = 0.0;
          break;
        }

      } else {

        if (search_image.LoadPatch( H, pPatch )) {
          score = ScorePatchesMeanSAD(ref_patch, pPatch, patch_size, patch_size);
          PrintMessage( verb, "     score = %.2f ", score );
        } else {
          continue;
        }

        if (score < best_score) {
          second_best = best_score;
          match = pf.get();
          match_homography = H;
          best_score  = score;
          PrintMessage( verb, "***" );
        } else if (score < second_best) {
          // only record second best if the feature is actually from a different
          // location:
          if (match && fabs(match->x - pf->x) > 2) {
            second_best = score;
          }
        }
      }

      PrintMessage( verb, "\n" );
    }

    if (match && is_simulation) {
      break;
    }
  }

  match_score = best_score;
  if( best_score == FLT_MAX ){
    match_flag = NoFeaturesToMatch;
  }
  else if( best_score > CommonFrontEndConfig::getConfig()->match_error_threshold ) {
    match_flag  = NoMatchInRegion;
  }
  else if( best_score*CommonFrontEndConfig::getConfig()->match_error_factor >= second_best ) {
    match_flag  = AmbiguousMatch;
  }
  else{
    match_flag = GoodMatch;
  }


  return match;
}

///////////////////////////////////////////////////////////////////////////////
///
template<unsigned int PatchSize=CANONICAL_PATCH_SIZE>
///
/// \brief FindEpipolarMatch, Look for best match to rRefFeature in
/// search_image, with warping, and epi-polar geometry check -- e.g, the
/// matched feature must be on the epi-polar line. Also does jealous matching.
/// \param ref_feature,
/// \param ref_homography
/// \param ref_patch
/// \param rig
/// \param ref_cam_id
/// \param search_cam_id
/// \param ref_image
/// \param search_image
/// \param match_score
/// \param match_flag
/// \param match_homography
/// \param featureOptions
/// \return
///
inline Feature* FindEpipolarMatch(
    Feature                            &ref_feature,
    PatchHomography<PatchSize>         &ref_homography,
    const unsigned char                *ref_patch,
    const calibu::CameraRigT<Scalar>   &rig,
    const unsigned int                 ref_cam_id,
    const unsigned int                 search_cam_id,
    FeatureImage                       &ref_image,
    FeatureImage                       &search_image,
    float                              &match_score,
    MatchFlag                          &match_flag,
    PatchHomography<PatchSize>         &match_homography,
    const FeatureHandler::Options&     feature_options
    )
{
  int verb = 1;
  Feature* match;
  const calibu::CameraModelGeneric<Scalar>& ref_cam = rig.cameras[ref_cam_id].camera;
  const calibu::CameraModelGeneric<Scalar>& search_cam = rig.cameras[search_cam_id].camera;
  std::vector<unsigned char> patch(PatchSize*PatchSize);
  unsigned char* match_patch = &patch[0];

  // pre-compute ray coming out of ref camera, but in search camera frame
  const Sophus::SE3t Tsr = rig.cameras[search_cam_id].T_wc.inverse() *
                           rig.cameras[ref_cam_id].T_wc;
  Eigen::Vector2t pt = Eigen::Vector2t(ref_feature.x, ref_feature.y);
  const Eigen::Vector3t rRefRay_sp = Tsr.so3()*ref_cam.Unproject(pt);

  // OK, need to determine the transfer function between reference image and search image:
  //PatchHomography<float,PatchSize> H;
  match = FindBestPatchOnEpipolarLine<PatchSize>(ref_feature,
                                                 rRefRay_sp,
                                                 ref_homography,
                                                 Tsr,
                                                 ref_cam,
                                                 search_cam,
                                                 ref_patch,
                                                 PatchSize,
                                                 search_image,
                                                 match_score,
                                                 match_flag,
                                                 match_homography,
                                                 feature_options);

  if (match_flag != GoodMatch) {
    ref_feature.used = true; // not sure this is wise
    PrintMessage(verb, "    Epipolar Search Failed: '%s'\n",
                 MatchStr(match_flag));
    //        if( eMatchFlag == NoMatchOnLine ) {
    //            PrintMessage( g_frontend_cvars.m_uDebugStartNewLandmarks,
    //                          "    Bad multi-view match [%.2f %.2f] score: %.2f, skipping\n",
    //                          pMatch->x, pMatch->y, fMatchScore  );
    //        } else {
    //            PrintMessage( g_frontend_cvars.m_uDebugStartNewLandmarks,"    No multi-view match, skipping\n" );
    //        }
    return NULL; // failed to match in this rSearchImage
  }

  // we have an initial match
  PrintMessage( verb,
                "    Initial match (berofe reverse validation) from cam[%d] "
                "at [%.2f. %.2f %.2f] to cam[%d] at [%.2f. %.2f %.2f] "
                "score: %0.2f\n",
                ref_cam_id, ref_feature.x, ref_feature.y, ref_feature.scale,
                search_cam_id, match->x, match->y, match->scale, match_score );

  // validate match by doing reverse matching
  //    printf( "    Starting from [%.0f,%.0f], hoping to reverse match to [%.0f,%.0f]\n",
  //            pMatch->x, pMatch->y, rRefFeature.x, rRefFeature.y ); fflush(stdout);

  if (CommonFrontEndConfig::getConfig()->do_jealous_matching) {
    // re-compute H that we found in first epi-polar search:
    pt << match->x, match->y;
    search_image.LoadPatch( match_homography, match_patch );

    const Eigen::Vector3t rSearchRay = Tsr.inverse().so3()*ref_cam.Unproject( pt );
    PatchHomography<PatchSize> reverseH;
    Feature* pReverseMatch = FindBestPatchOnEpipolarLine(*match,
                                                         rSearchRay,
                                                         match_homography,
                                                         Tsr.inverse(),
                                                         search_cam,
                                                         ref_cam,
                                                         match_patch,
                                                         PatchSize,
                                                         ref_image,
                                                         match_score,
                                                         match_flag,
                                                         reverseH,
                                                         feature_options);

    //        assert( rHRef.matrix() == reverseH.matrix() );

    if( match_flag != GoodMatch ) {
      ref_feature.used = true;
      match->used = true;

      // There should be a match. If not, something went really wrong
      if( match_flag == NoFeaturesToMatch ){
        PrintMessage( 0, "CRITICAL ERROR. No match found during match validation.\n");
      }
      return NULL;
    }

    // if reverse matching fails, flag both features and move on to a new feature
    if( pReverseMatch->x != ref_feature.x ) {
      ref_feature.used = true;
      match->used = true;
      PrintMessage( 1,
                    "    Reverse-match FAIL (got %.2f instead of %.2f), skipping\n",
                    pReverseMatch->x, ref_feature.x );
      match_flag = ReverseMatchFail;
      return NULL;
    }
  }

  // we have a match!
  PrintMessage( verb,
                "    Initial match from cam[%d] at [%.2f. %.2f  s(%.2f)] to cam[%d] at [%.2f. %.2f] score: %0.2f\n",
                ref_cam_id, ref_feature.x, ref_feature.y, ref_feature.scale,
                search_cam_id, match->x, match->y, match_score);

  // now do subpixel refinement if active
  if( CommonFrontEndConfig::getConfig()->do_subpixel_refinement ){
    float fRefinedX = match->x;
    float fRefinedY = match->y;
    double dError = search_image.RefineSubPixelXY((unsigned char*)ref_patch,
                                                  match_homography,
                                                  fRefinedX,
                                                  fRefinedY );

    PrintMessage( verb,
                  "    ESM RMS error %f, refined match location in cam[%d] from u=%.3f to u=%.3f and v=%.3f to v=%.3f\n",
                  dError, search_cam_id, match->x, fRefinedX,match->y, fRefinedY );

    if( fabs(fRefinedX - match->x) > CommonFrontEndConfig::getConfig()->esm_subpixel_threshold*match->scale ||
        fabs(fRefinedY - match->y) > CommonFrontEndConfig::getConfig()->esm_subpixel_threshold*match->scale) {
      ref_feature.used = true;
      match->used = true;
      match_flag = LowSubPixOnLine;
      PrintMessage( verb,"    ESM pixel offset error above threshold, skipping\n" );
      return NULL;
    }

    if( dError > CommonFrontEndConfig::getConfig()->esm_threshold ){
      ref_feature.used = true;
      match->used = true;
      PrintMessage( verb,"    ESM error above threshold, skipping\n" );
      match_flag = BadScoreAfterESM;
      return NULL;
    }

    //        if( DEBUG_HELPERS ){
    //            fRefinedY = rRefFeature.y; // hack for rectified stereo
    //        }

    match->x = fRefinedX;
    match->y = fRefinedY;
    match_score = (float)dError;
    match_homography.SetTranslation( fRefinedX, fRefinedY );
  }
  match->scale = match_homography.scale();

  return match;
}

///////////////////////////////////////////////////////////////////////////////

template<unsigned int PatchSize=CANONICAL_PATCH_SIZE>
bool Create3DPatchHomography(
    const calibu::CameraModelGeneric<Scalar> &cam,
    const Sophus::SE3Group<Scalar>           &Tvs,
    const Eigen::Vector2t                    &pt,
    const Scalar                             homography_width, // width of the square patch the user wants to track
    PatchHomography<PatchSize>               &projected_h,     ///< Output:
    Sophus::SO3Group<Scalar>                 &orientation_v,   ///< Output:
    Scalar                                   &radius           ///< Output:
    )
{

  double hs = homography_width/2.0;

  Eigen::Vector2t t( pt(0)     , pt(1) - hs );
  Eigen::Vector2t l( pt(0) - hs, pt(1)      );
  Eigen::Vector2t b( pt(0)     , pt(1) + hs );
  Eigen::Vector2t r( pt(0) + hs, pt(1)      );


  const Eigen::Vector3t fw_s = (cam.Unproject( pt ).normalized());
  const Eigen::Vector3t fw_camera(0,0,1);
  double angle = fabs(acos(fw_s.dot(fw_camera)));
  if( angle > 85.0/180.0 * M_PI ){
    return false;
  }

  const Eigen::Vector3t t3d = (cam.Unproject( t ).normalized());
  const Eigen::Vector3t l3d = (cam.Unproject( l ).normalized());
  const Eigen::Vector3t b3d = (cam.Unproject( b ).normalized());
  const Eigen::Vector3t r3d = (cam.Unproject( r ).normalized());
  const Eigen::Vector3t c3d = fw_s;

  radius = ((b3d-t3d).norm() + (l3d-r3d).norm())/4.0;

  Eigen::Vector3t rt = l3d-r3d;
  Eigen::Vector3t fw = -c3d;          // center of cam to lm
  Eigen::Vector3t dn = fw.cross(rt);
  rt = dn.cross(fw);
  fw = fw/fw.norm();
  rt = rt/rt.norm();
  dn = dn/dn.norm();

  Eigen::Matrix3t orientation_mat_c;
  orientation_mat_c.col(0) = fw;
  orientation_mat_c.col(1) = rt;
  orientation_mat_c.col(2) = dn;

  Sophus::SO3t orientation_c = Sophus::SO3t(orientation_mat_c);

  // now re-load patch based on 3D world patch:
  // top-left in patch frame r (we assume +x is the patch surface normal)
  const Eigen::Vector3t xptl( 0,  radius, -radius ); // top left in patch frame
  const Eigen::Vector3t xptr( 0, -radius, -radius ); // top right in patch frame
  const Eigen::Vector3t xpbr( 0, -radius,  radius ); // bottom right in patch frame
  const Eigen::Vector3t xpbl( 0,  radius,  radius ); // bottom left in patch frame

  // move from patch frame, p, to camera frame and project
  const Eigen::Vector3t xvtl( orientation_c * xptl + c3d );
  const Eigen::Vector3t xvtr( orientation_c * xptr + c3d );
  const Eigen::Vector3t xvbr( orientation_c * xpbr + c3d );
  const Eigen::Vector3t xvbl( orientation_c * xpbl + c3d );

  const Eigen::Vector2t tl = cam.Project(xvtl);
  const Eigen::Vector2t tr = cam.Project(xvtr);
  const Eigen::Vector2t br = cam.Project(xvbr);
  const Eigen::Vector2t bl = cam.Project(xvbl);


#ifndef IGNORE_PATCH_WARPING
  projected_h = PatchHomography<CANONICAL_PATCH_SIZE>( tl, tr, br, bl );
#else
  projected_h = PatchHomography<CANONICAL_PATCH_SIZE>(pt[0], pt[1],
      PatchHomography<CANONICAL_PATCH_SIZE>::GetQuadrilateralScale(
        tl,tr,bl,br));
#endif

  orientation_v = Tvs.so3() * orientation_c;
  return true;
}


///////////////////////////////////////////////////////////////////////////////
/// Search over all images besides uCamera and try to find a match.
template<unsigned int PatchSize=CANONICAL_PATCH_SIZE>
inline bool GetMultiViewMatchesIfPossible(
    const calibu::CameraRigT<Scalar> &rig,               //< Input: Mullti-view camera rig
    const unsigned                   ref_cam_id,         //< Input: Id of camera feature is in
    FeatureImageVector               &images,            //< Input: vector of FeatureImages
    Feature                          *feature,           //< Input: Feature from nRefCam to triangulate
    FeatureMatches<PatchSize>        &matches,           //< Output: vector of features that match
    Sophus::SO3Group<Scalar>         &patch_orientation, //< Output: Orientation of patch
    Scalar                           &radius,            //< Output: Radius of patch
    const FeatureHandler::Options    &feature_options)
{
  matches.vFeatures.clear();
  matches.vErrors.clear();
  matches.vFlags.clear();
  matches.vH.clear();
  // rMatches.vPatches.clear();
  const size_t uNumCam = rig.cameras.size();
  matches.vFeatures.reserve(uNumCam);
  matches.vErrors.reserve(uNumCam);
  matches.vFlags.reserve(uNumCam);
  matches.vH.reserve(uNumCam);
  // rMatches.vPatches.reserve(uNumCam);
  FeatureImage& reference_image = *images[ref_cam_id];

  const unsigned int patch_size = CANONICAL_PATCH_SIZE;
  std::vector<unsigned char> patch(patch_size*patch_size);
  unsigned char* pPatch = &patch[0];
  float match_score;
  MatchFlag match_flag;

  // figure out the patch homography
  const calibu::CameraModelGeneric<Scalar>& cam = rig.cameras[ref_cam_id].camera;
  const Sophus::SE3t Tvs = rig.cameras[ref_cam_id].T_wc;
  // feature->scale here is (1/levelFactor)^pyramidLevel (so for levelFactor=0.5, scale will be
  // 1, 2, 4 for pyramid level 0, 1, 2. dHomographyWidth is then the width of the homography in pixels
  // as measured on the 0th level of the pyramid
  Scalar homography_width = feature->scale*(CANONICAL_PATCH_SIZE-1);
  PatchHomography<CANONICAL_PATCH_SIZE> reference_homography;
  const Eigen::Vector2t pt(feature->x, feature->y);

  // this function may return false if the homography is unsuitable
  if (Create3DPatchHomography(cam, Tvs, pt, homography_width,
                              reference_homography, patch_orientation,
                              radius) == false) {
    feature->used = true;
    return false;
  }

  reference_image.LoadPatch(reference_homography, pPatch);

  //PatchHomography<Scalar,CANONICAL_PATCH_SIZE> HRef( pFeature->x, pFeature->y, pFeature->scale );

  PatchHomography<CANONICAL_PATCH_SIZE> match_homography;

  for (size_t search_cam_id = 0; search_cam_id < rig.cameras.size(); ++search_cam_id) {
    if( search_cam_id == ref_cam_id ){
      matches.vFeatures.push_back(feature);  // ref feature goes here (e.g., slot nRefCam holds refFeature)
      matches.vErrors.push_back(0.0);
      matches.vFlags.push_back(GoodMatch);
      // rMatches.pPatch = pPatch; // static, will not go away, will get copied into LM
      matches.vH.push_back(reference_homography); // is this correct?
      continue;
    }
    // find match on the epi-polar line in every camera that can see the feature:
    FeatureImage& search_image = *images[search_cam_id];
    Feature* match =
        FindEpipolarMatch<PatchSize>(*feature,
                                     reference_homography,
                                     pPatch,
                                     rig,
                                     ref_cam_id,
                                     search_cam_id,
                                     reference_image,
                                     search_image,
                                     match_score,
                                     match_flag,
                                     match_homography,
                                     feature_options);

    matches.vFeatures.push_back(match); // if null, then no match.  Epipolar geometry will be good
    matches.vErrors.push_back(match_score);
    matches.vFlags.push_back(match_flag);
    matches.vH.push_back(match_homography);
  }
  return true;
}
