// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.


#include <common_front_end/MatchHelpers.h>  // for FindBestMatchInRow and FindBestMatchInRegion
#include <common_front_end/Triangulation.h>
#include <miniglog/logging.h>

#include <slam_map/SlamMap.h>
#include <local_map/LocalMap.h>

//#include <sparse_tracking/TrackingCVars.h>
#include <common_front_end/CommonFrontEndConfig.h>
#include <sparse_tracking/TrackingConfig.h>
#include <sparse_tracking/MatchInTime.h>

#include <utils/PatchUtils.h>
#include <utils/MathTypes.h>
#include <utils/PrintMessage.h>
#include <utils/ESM.h>
#include <utils/PoseHelpers.h>

//#define DEBUG_HELPERS 0

using namespace Eigen;

typedef PatchHomography<CANONICAL_PATCH_SIZE> Homography;

///////////////////////////////////////////////////////////////////////////////
///
/// \brief _MatchLandmarkInTime - Populate a new measurement,
/// assuming we were able to track one.
/// \param[in] landmark we are trying to match
/// \param[in] homographies predicted in each camera
/// \param[in] images to search
/// \param[in] feature_options
/// \param[in/out] z new measurement to create
/// \param[out] matches pointers to matched features
/// \return success
///
inline bool  _MatchLandmarkInTime(
    const Landmark                &landmark,
    const std::vector<Homography> &homographies,
    const FeatureImageVector      &images,
    const FeatureHandler::Options &feature_options,
    MultiViewMeasurement          &z,
    std::vector<Feature*>         &matches,
    float                         search_radius)
{
  bool success = false;
  MatchFlag match_flag;
  float match_score;

  // Try to find the landmark in each image:
  for (size_t cam_id = 0; cam_id < images.size(); cam_id++) {

    // default values
    z.SetPixelInCam(cam_id,-1.0,-1.0);
    z.SetMatchingError(cam_id, -1.0);

    // get reference to search image
    const FeatureImage& search_image = *images[cam_id].get();
    // get predicted homography
    PatchHomography<CANONICAL_PATCH_SIZE> H = homographies[cam_id];
    const Eigen::Vector2t hx = H.CenterPixel();

    LOG(TrackingConfig::getConfig()->matchintime_debug_level)
        << "\tMIT: Predicted [ << " << hx[0] << ", " << hx[1]
        << "] in camera-" << cam_id;

    // if predicted outside the image try next camera (if any)
    if (H.GetState() ==
        PatchHomography<CANONICAL_PATCH_SIZE>::eOutsideFOV) {
      LOG(TrackingConfig::getConfig()->matchintime_debug_level) <<  "\tOut of Bounds";
      z.SetFlag( cam_id, OutsideFOV );
      continue;
    }

    // now match the feature
    Feature* pMIT = FindBestMatchInRegion(landmark.id().track2d_id, H,
                                          landmark.descriptor(),
                                          landmark.desc_size(),
                                          landmark.patch(),
                                          search_radius,
                                          search_radius,
                                          search_image,
                                          feature_options,
                                          match_score,
                                          match_flag);

    // record the result of the initial match
    z.SetFlag(cam_id, match_flag);
    z.SetScale(cam_id, H.scale());
    matches[cam_id] = pMIT;

    if (pMIT) {
      LOG(TrackingConfig::getConfig()->matchintime_debug_level)
          << "\tMIT: Initial match " <<  MatchStr(match_flag)
          << " [ " << pMIT->x << ", " << pMIT->y << ", " << pMIT->scale << "] "
          << "(predicted [" << hx[0] << ", " << hx[1] << "]) "
          << "in camera-" << cam_id << " (score " << match_score << ")";

      // we found a match ( either good or bad, just save matching info )
      H.SetTranslation(pMIT->x, pMIT->y);
      z.SetPatchHomography(cam_id, H);
      z.SetPixelInCam(cam_id, pMIT->x, pMIT->y);
      z.SetMatchingError(cam_id, match_score);
    } else {
      // no matches were found record prediction
      z.SetPixelInCam(cam_id, hx[0], hx[1]);
      z.SetPatchHomography(cam_id, H);
    }

    // If the match is bad try next camera (if any)
    if (match_flag != GoodMatch) {
      if (pMIT == nullptr) {
        // In this case no match was found
        LOG(TrackingConfig::getConfig()->matchintime_debug_level)
            << "\tNo match found int camera: " << cam_id;
      } else {
        PrintMessage(TrackingConfig::getConfig()->matchintime_debug_level,
                     "    Bad temporal-match [%.2f %.2f] in camera: %d "
                     "score: %.2f [%s], skipping\n",
                     pMIT->x, pMIT->y, cam_id, match_score,
                     MatchStr(match_flag));
      }
      continue;
    }

    // if refinement is active do it
    float refined_x = pMIT->x;
    float refined_y = pMIT->y;
    float scale     = H.scale();

    if (CommonFrontEndConfig::getConfig()->do_subpixel_refinement) {

      double error =
          search_image.RefineSubPixelXY(landmark.patch(), H,
                                        refined_x, refined_y);

      PrintMessage(TrackingConfig::getConfig()->matchintime_debug_level,
                   "    MIT: ESM (error %f) match refined by [%.2f, %.2f] "
                   "from [%.2f, %.2f] to [%.2f, %.2f]\n",
                   error, pMIT->x - refined_x, pMIT->y - refined_y,
                   pMIT->x, pMIT->y, refined_x, refined_y);

      // re-center after refinement
      H.SetTranslation(refined_x, refined_y);
      z.SetPatchHomography(cam_id, H);

      //  record refined location and error
      z.SetPixelInCam(cam_id, refined_x, refined_y);
      z.SetMatchingError(cam_id , error);

      // ESM error can help to spot bad matches
      Scalar scaled_error = CommonFrontEndConfig::getConfig()->esm_subpixel_threshold*scale;
      if (fabs(refined_x - pMIT->x) > scaled_error ||
          fabs(refined_y - pMIT->y) > scaled_error) {
        PrintMessage(TrackingConfig::getConfig()->matchintime_debug_level,
                     "    ESM Error above threshold, skipping\n");
        z.SetFlag(cam_id, LowSubPixInRegion);
        continue; // try next camera (if any)
      }
      if (error > CommonFrontEndConfig::getConfig()->esm_threshold) {
        z.SetFlag(cam_id, BadScoreAfterESM);
        continue; // try next camera (if any)
      }
    }

    // good
    // THIS IS SEGFAULTING
    /*PrintMessage(TrackingConfig::getConfig()->matchintime_debug_level,
                 "    MIT: SUCCESS -- LM '%s' match at [%.2f, %.2f] "
                 "(predicted [%.2f, %.2f]) in camera-%d\n",
                 landmark.id(), refined_x, refined_y, hx[0], hx[1], cam_id);
*/
    z.SetScale(cam_id, H.scale());
    z.PatchVector(cam_id).resize(powi(CANONICAL_PATCH_SIZE,2));
    search_image.LoadPatch(z.GetPatchHomography(cam_id),
                           &(z.PatchVector(cam_id)[0]));

    // indicate that we are using this feature --
    //so it doesn't get matched or used for a new landmark
    pMIT->used = true; // Is this working? feature images are const!
    success = true;
  }
  return success;
}

//////////////////////////////////////////////////////////////////////////////////
/// \brief ComputeReprojectionUncertainty
/// \param rCam
/// \param rPoseCov
/// \param dRange
/// \return
/// Compute the 2x2 covarinace of reprojection error given a camera with a noisy
/// pose modeled by the covarince rPoseCov.  This uses the unscented transform for now
/// TODO: compute w Jacobians.
Eigen::Matrix2t ComputeReprojectionUncertainty(
    const calibu::CameraModelGeneric<Scalar> &rCam,      //< Input: camera model
    const Sophus::SE3Group<Scalar>           &rPose,     //< Input: camera pose
    const Eigen::Matrix6t                    &rPoseCov,  //< Input: Covariance on camera model pose
    const Eigen::Vector4t                    &rX         //< Input: 3d homogenous point ready for proj.
    )
{
  Eigen::Matrix<Scalar,2,7> vPixels;
  vPixels.col(0) = rCam.Project( rPose.matrix3x4()*rX );
  //    printf( "pt %d, projected [%.2f, %.2f, %.2f]  to [%.2f, %.2f]\n",
  //            0, x[0], x[1], x[2], vPixels.col(0)[0], vPixels.col(0)[1] ); fflush(stdout);
  const Eigen::Matrix<Scalar,6,6> SigmaPoints = rPoseCov.llt().matrixU();
  //    std::cout << rPoseCov << std::endl;
  //    std::cout << SigmaPoints << std::endl;
  for( int ii = 0; ii < 6; ii++ ){ // compute sigma points
    const Sophus::SE3t TSigmaPoint = Sophus::SE3t::exp( SigmaPoints.col(ii) ); // perturb pose
    const Eigen::Vector3t pt = (TSigmaPoint*rPose).matrix3x4()*rX;
    vPixels.col(ii+1) = rCam.Project( pt );
    //        Vector6d t = (TSigmaPoint*rPose).log();
    //        printf( "pt %d, projected [%.2f, %.2f, %.2f]  to [%.2f, %.2f] from [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] \n",
    //                ii+1, x[0], x[1], x[2], vPixels.col(ii+1)[0], vPixels.col(ii+1)[1],
    //                t[0], t[1], t[2], t[3], t[4], t[5] ); fflush(stdout);
  }
  vPixels.row(0) = vPixels.row(0).array() - vPixels.row(0).mean();
  vPixels.row(1) = vPixels.row(1).array() - vPixels.row(1).mean();
  return vPixels * vPixels.transpose();
}

/*
///////////////////////////////////////////////////////////////////////////////
/// Predict a landmark in all of the cameras in the rig. Also get reprojection
/// std. dev. wrt pose uncertainty so we can search only where we need to.
/// TODO this should be a part of TrackingWorkingSet
template<typename Scalar>
void PredictMeasurements(
        const LandmarkContainer<Scalar>&          rLmkContainer,//< Input: landmark container we wish to predict
        const calibu::CameraRigT<Scalar>&         rRig,         //< Input: our camera rig
        const vector< Sophus::SE3Group<Scalar> >& vTsw,         //< Input: vector of camera poses in lifted frame
        const Eigen::Matrix<Scalar,6,6>&          rPoseCov,     //< Input: Covariance on camera model pose
        vector< Eigen::Matrix<Scalar,2,1> >&      vPixels,      //< Output: expected pixel location
        vector< Eigen::Matrix<Scalar,2,1> >&      vPixelStdDevs //< Output: x and y pixel std. dev. given pose Cov
        )
{
    assert( rRig.cameras.size() == vTsw.size() );

    vPixels.resize( rRig.cameras.size() );
    vPixelStdDevs.resize( rRig.cameras.size() );
    for( size_t nCam = 0; nCam < rRig.cameras.size(); nCam++ ){
        const calibu::CameraModelGeneric<Scalar>& cam = rRig.cameras[nCam].camera;

        // first, find out search region based on pose uncertainty
        const Eigen::Matrix<Scalar,4,1> x = rLmkContainer.m_dXwp;
        const Eigen::Matrix<Scalar,2,2> R = ComputeReprojectionUncertainty( cam, vTsw[nCam], rPoseCov, x );
        const Scalar w = cam.Width();
        const Scalar h = cam.Height();
        const double xstd = min( R(0,0), w );
        const double ystd = min( R(1,1), h );
        vPixelStdDevs[nCam] = Eigen::Matrix<Scalar,2,1>(xstd,ystd);
        // too uncertain? track from old pixel pos then.
        if( xstd > w/2 || ystd > h/2 ){
            vPixels[nCam] = _GetPrevMeasurement( rLmkContainer, nCam );
        }
        else{
            vPixels[nCam] = cam.Project( vTsw[nCam].matrix3x4()*rLmkContainer.m_dXwp );
        }

        std::cout << "CamPose: " << vTsw[nCam].log().transpose() << std::endl;

        Eigen::Matrix<Scalar,2,1> p = _GetPrevMeasurement( rLmkContainer, nCam );
        printf("Predicted %.2f, %.2f --> %.2f, %.2f\n", p[0], p[1],
                vPixels[nCam][0], vPixels[nCam][1] ); fflush(stdout);
    }
}
*/
void PrintPose(const std::string& sStr,
               const Sophus::SE3t& Tab)
{
  std::cout << sStr << rslam::T2Cart(Tab.matrix()).transpose() << std::endl;
}

Sophus::SE3t _RDF( const Sophus::SE3t& Tab )
{
  Sophus::SE3t Mvr;
  Mvr.so3() = calibu::RdfRobotics.inverse();
  return Tab*Mvr; // undo rdf business
}

int MatchInTime(const ReferenceFrameId                &frame_id,
                const FeatureImageVector              &images,
                const LocalMap                        &working_set,
                const FeatureHandler::Options         &feature_options,
                std::vector<MultiViewMeasurement>     &new_measurements,
                std::vector< std::vector<Feature*> >  &feature_matches,
                float                                 search_radius) {
  PrintMessage( TrackingConfig::getConfig()->matchintime_debug_level, "<MatchInTime>\n" );

  // Reset output vars
  new_measurements.clear();
  feature_matches.clear();
  feature_matches.reserve(working_set.landmarks.size());
  std::vector<Feature*> matches(images.size());
  new_measurements.resize(working_set.landmarks.size(),
                          MultiViewMeasurement(images.size()));

  MeasurementId zid;
  int z_idx = 0;
  int num_matches = 0;
  for (const auto& pair : working_set.landmarks) {
    zid.landmark_id = pair.first;
    zid.frame_id    = frame_id;
    const std::shared_ptr<LandmarkContainer>& lm_container = pair.second;

    // IMPORTANT!
    // For debugging we always add a measurement of a landmark so we
    // can check the reasons of tracking failure. Landmarks need to
    // have a state variable now. This is to avoid including landmarks
    // in the working set that haven't been succesfully tracked in the last
    // n frames.

    MultiViewMeasurement& z = new_measurements[z_idx++];
    z.set_id(zid);

    Landmark lm;
    /** @todo JGM This is probably why MIT takes _so_ long */
    if (!working_set.map->GetLandmark(lm_container->id, &lm)) {
      LOG(ERROR) << "Landmark not found in working set during MatchInTime";
      continue;
    }

    PrintMessage(TrackingConfig::getConfig()->matchintime_debug_level,
                 "\n    -------------------------------------\n"
                 "    Matching LM %s\n", lm.id());

    bool success = _MatchLandmarkInTime(lm, lm_container->tracking_homographies,
                                        images, feature_options, z,  matches,
                                        search_radius);

    feature_matches.push_back(matches);

    if (!success) {
      PrintMessage(TrackingConfig::getConfig()->matchintime_debug_level,
                   "    MIT: FAILED\n");
      continue;
    }
    num_matches++;
  }

  PrintMessage(TrackingConfig::getConfig()->matchintime_debug_level,
               "    Tracked %d features\n", num_matches);
  PrintMessage(TrackingConfig::getConfig()->matchintime_debug_level, "</MatchInTime>\n\n");

  return num_matches;
}
