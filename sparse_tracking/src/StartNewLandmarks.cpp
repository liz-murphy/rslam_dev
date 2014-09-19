// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.


#include <iostream>
#include <iomanip>
#include <string>

#include <slam_map/Landmark.h>
#include <slam_map/Measurement.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/SlamMap.h>

#include <sparse_tracking/StartNewLandmarks.h>
//#include <sparse_tracking/TrackingCVars.h>
#include <sparse_tracking/TrackingConfig.h>

#include <common_front_end/MatchHelpers.h>
#include <common_front_end/PatchMatch.h>
#include <common_front_end/Triangulation.h>

#include <utils/Utils.h>
#include <utils/MathTypes.h>
#include <utils/PatchUtils.h>
#include <utils/PrintMessage.h>
#include <utils/ESM.h>

namespace rslam {
namespace sparse {

///
/// \brief Transfer
/// \param src_cam
/// \param dst_cam
/// \param srcpix
/// \param depth
/// \return
///
Eigen::Vector2t Transfer(
    const calibu::CameraModelAndTransformT<Scalar> &src_cam,
    const calibu::CameraModelAndTransformT<Scalar> &dst_cam,
    const Eigen::Vector2t &srcpix,
    const Scalar depth = 1e8 )
{
  const Sophus::SE3t Twn = dst_cam.T_wc.inverse() * src_cam.T_wc;
  return dst_cam.camera.Project(Twn*(depth*src_cam.camera.Unproject(srcpix)));
}

///
/// \brief Draw one frustum in the view of the other.
/// This ONLY works for two cameras.
/// \param rig
/// \param cam1_id
/// \return
///
Eigen::Vector4t GetFrustrumBoundingBox(const calibu::CameraRigT<Scalar> &rig,
                                       const unsigned int cam1_id)
{
  unsigned int cam2_id = !cam1_id;

  // find the fov of the right camera in the left view
  const calibu::CameraModelAndTransformT<Scalar>& narrow_cam =
      rig.cameras[cam2_id];
  const calibu::CameraModelAndTransformT<Scalar>& wide_cam =
      rig.cameras[cam1_id];
  const float w = narrow_cam.camera.Width();
  const float h = narrow_cam.camera.Height();
  const float steps = 10;

  Eigen::Vector2tArray pixels;
  for (Scalar x = 0; x < w-1; x+= w/steps) {
    Eigen::Vector2t p =
        Transfer( narrow_cam, wide_cam, Eigen::Vector2t(x,0) );
    pixels.push_back( p );
  }
  for (Scalar y = 0; y < h; y+= h/steps) {
    Eigen::Vector2t p =
        Transfer( narrow_cam, wide_cam, Eigen::Vector2t(w,y) );
    pixels.push_back( p );
  }
  for (Scalar x = w; x > 0; x-= w/steps) {
    Eigen::Vector2t p =
        Transfer( narrow_cam, wide_cam, Eigen::Vector2t(x,h) );
    pixels.push_back( p );
  }
  for (Scalar y = h; y > 0; y-= h/steps) {
    Eigen::Vector2t p =
        Transfer( narrow_cam, wide_cam, Eigen::Vector2t(0,y) );
    pixels.push_back( p );
  }

  // bounds are ordered as t l b r
  Eigen::Vector4t bounds;
  bounds[0] = h/2;
  bounds[1] = w/2;
  bounds[2] = h/2;
  bounds[3] = w/2;

  for (const Eigen::Vector2t& pixel: pixels) {
    bounds[0] = std::min(bounds[0],pixel[1]);   // top bound
    bounds[1] = std::min(bounds[1],pixel[0]);   // left bound
    bounds[2] = std::max(bounds[2],pixel[1]);   // bottom bound
    bounds[3] = std::max(bounds[3],pixel[0]);   // right bound
  }
  return bounds;
}


///
/// \brief FillQuadTreesWithMeasurementsOfTrackedLandmarks
/// Project *tracked* landmarks and bin them in the image so we know where
/// empty parts of the image are.
/// \param current_frame
/// \param images
/// \param num_tracked_landmarks
/// \param num_tracked_multiView_landmarks
/// \return
///
bool FillQuadTreesWithMeasurementsOfTrackedLandmarks(
    SlamFramePtr         current_frame,
    FeatureImageVector   &images,
    unsigned int         &num_tracked_landmarks,
    unsigned int         &num_tracked_multiView_landmarks )
{
  num_tracked_landmarks = 0;
  num_tracked_multiView_landmarks = 0;

  MultiViewMeasurement z;
  size_t num_meas = current_frame->NumMeasurements();
  for (size_t zi = 0; zi < num_meas; ++zi) {
    bool was_tracked = false;
    unsigned int num_cameras_tracked = 0;

    for (size_t cam = 0; cam < images.size(); ++cam) {
      if (current_frame->HasGoodMeasurementInCam(zi, cam) &&
          current_frame->GetMeasurement(zi, &z)) {
        double u = z.Pixel(cam)(0);
        double v = z.Pixel(cam)(1);
        images[cam]->GetQuadTreeRef().AddPoint( round(v), round(u) );
        was_tracked = true;
        num_cameras_tracked++;
      }
    }

    if (was_tracked) {
      if( num_cameras_tracked > 1 ){
        num_tracked_multiView_landmarks++;
      }
      num_tracked_landmarks++;
    }
  }

  if (TrackingConfig::getConfig()->.startnewlandmarks_debug_level == 0) {
    printf(" QuadTrees after adding tracked landmarks \n");
    for(size_t cam=0; cam < images.size(); ++cam) {
      printf(" QuadTree camera [%lu]: \n",cam);
      images[cam]->GetQuadTreeRef().PrintTree();
    }
  }

  return true;
}

///
/// \brief UpdateQuadTrees
/// \param[in] matched_features
/// \param[in,out] images
///
inline void UpdateQuadTrees(const std::vector<Feature*> &matched_features,
                            FeatureImageVector &images)
{
  for (size_t ii = 0; ii < matched_features.size(); ++ii) {
    QuadTree& qt = images[ii]->GetQuadTreeRef();
    Feature* feature = matched_features[ii];
    if (feature) {
      qt.AddPoint( roundf(feature->y), roundf(feature->x) );
    }
  }
}


///
/// \brief Setup a new landmark in the SLAM engine.
/// \param[in] x_r 3d landmark position relative to the vehicle frame
/// \param[in] patch_orientation
/// \param[in] radius
/// \param[in] state
/// \param[in] ref_camera_id where landmark was observed
/// \param[in] images containing the features
/// \param[in,out] map pointer
/// \param[in,out] frame pointer to map frame
/// \param[in,out] matches in all images
/// \return
///
inline bool InitLandmark(
    const calibu::CameraRigT<Scalar> &rig,
    const Eigen::Vector4t          &x_r,
    const Sophus::SO3Group<Scalar> &patch_orientation,
    const Scalar                   radius_at_cam_plane,
    const LandmarkState            state,
    const unsigned int             ref_camera_id,
    const FeatureImageVector&      images,
    const std::shared_ptr<SlamMap>& map,
    SlamFramePtr                   frame,
    FeatureMatches<>&              matches)
{
  const Eigen::Vector4t x_c =
    Sophus::MultHomogeneous(rig.cameras[ref_camera_id].T_wc.inverse(), x_r);

  // GetMultiViewMatches returns the radius for a patch at
  // the z=1 plane, therefore we need to scale this patch
  const Scalar radius = radius_at_cam_plane * x_c.norm();

  LandmarkId lmId;
  lmId.landmark_index = frame->NumLandmarks();
  lmId.ref_frame_id    = frame->id();

  if (!g_common_cvars.feature_detector.compare("TRACK_2D") ||
      !g_common_cvars.feature_detector.compare("FLYBY") ||
      !g_common_cvars.feature_detector.compare("SIMULATION")) {
    lmId.track2d_id = matches.vFeatures[ref_camera_id]->id;
  } else {
    lmId.track2d_id = 0;
  }

  MeasurementId zId;
  zId.frame_id     = frame->id();
  zId.landmark_id   = lmId;

  MultiViewMeasurement z(matches.vFeatures.size());
  z.set_id(zId);

  for (size_t cam_id = 0 ; cam_id < matches.vFeatures.size(); ++cam_id) {
    Feature* pFeat = matches.vFeatures[cam_id];
    // check if we matched in this frame
    if (pFeat) {
      // flag feature as used: this is a side effect!
      pFeat->used = true;
      z.SetMatchingError(cam_id, matches.vErrors[cam_id]);
      // not actually true before optimization
      z.SetReprojectionError(cam_id, 0.0);
      z.SetPixelInCam(cam_id, pFeat->x, pFeat->y);
      z.SetFlag(cam_id, NewFeature);
      z.SetScale(cam_id, pFeat->scale);
      z.SetOrientation(cam_id, pFeat->orientation);
      z.SetPatchHomography(cam_id, matches.vH[cam_id]);
      z.PatchVector(cam_id).resize(powi(CANONICAL_PATCH_SIZE,2));
      bool bSuccess =
          images[cam_id]->LoadPatch<CANONICAL_PATCH_SIZE>(
            z.GetPatchHomography(cam_id),&(z.PatchVector(cam_id)[0]));
      if (!bSuccess && cam_id == ref_camera_id) {
        PrintMessage( g_tracking_cvars.startnewlandmarks_debug_level,
                      "  WARNING: Couldn't load Reference Patch! \n");
        return false;
      }

    } else {
      z.SetFlag(cam_id, matches.vFlags[cam_id]);
    }
  }

  // init landmark
  Landmark lm;

  Feature* pRefFeature = matches.vFeatures[ref_camera_id];
  lm.Init(zId, true, lmId, ref_camera_id, x_r, patch_orientation,
          radius, &pRefFeature->descriptor[0], pRefFeature->descsize,
          z.PatchVector(ref_camera_id));
  lm.set_state(state);
  lm.set_camera_plane_extent(radius_at_cam_plane);

  // add to the frame
  frame->AddMeasurement(z);
  frame->AddLandmark(lm);

  StreamMessage( g_tracking_cvars.startnewlandmarks_debug_level) <<
                "    StartNewLandmark::_InitLandmark() -- "
      "added new landmark " << lm.id() << "," << lm.id() << " at" <<
                lm.xrp().transpose() << std::endl;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
void StartNewLandmarks(
    const calibu::CameraRigT<Scalar> &rig,
    const std::shared_ptr<SlamMap>   &map,
    SlamFramePtr                     current_frame,
    FeatureImageVector               &images,
    const FeatureHandler::Options    &feature_options,
    unsigned int                     &num_tracked_landmarks,
    unsigned int                     &num_new_landmarks,
    sparse::FeatureMask              &feature_mask)
{

  //===========================================================
  // Init variables
  //===========================================================
  Eigen::Vector4t  x_r;
  FeatureMatches<> matches;
  const size_t num_cameras = rig.cameras.size();
  const unsigned int desired_num_landmarks =
      g_common_cvars.num_features_to_track;
  unsigned int ref_camera_id = g_tracking_cvars.ref_cam_id;
  unsigned int num_feat_tested = 0;
  bool use_frustrum   = false;
  num_new_landmarks = 0;

  //===========================================================
  // Compute frustrum in reference camera.
  // This is the valid area for selecting features.
  //===========================================================
  Eigen::Vector4t frustum;
  if (num_cameras > 1) {
    frustum = GetFrustrumBoundingBox(rig,ref_camera_id);
  } else {
    frustum << 0, 0, rig.cameras[0].camera.Height(),
        rig.cameras[0].camera.Width();
  }
  static int roi[] = {(int)frustum[0],(int)frustum[1],
                      (int)frustum[2],(int)frustum[3]};

  PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
               "<StartNewLandmarks>\n");

  //===========================================================
  // Fill quadtrees using tracked features
  //===========================================================
  unsigned int num_tracked_multiview_landmarks;
  FillQuadTreesWithMeasurementsOfTrackedLandmarks(
        current_frame, images, num_tracked_landmarks,
        num_tracked_multiview_landmarks );

  PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
               "    Attempting to add %u new features\n",
               desired_num_landmarks - num_tracked_landmarks);

  //===========================================================
  // Attempt to add new landmarks if needed
  //===========================================================
  while ((num_new_landmarks + num_tracked_landmarks) < desired_num_landmarks) {

    // rotate through the cameras so we're not biased to
    // always starting in a particular camera
    ref_camera_id++;
    ref_camera_id = ref_camera_id % num_cameras;
    num_feat_tested++;

    PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
                 "    -----------------------------------------------------\n");

    // get reference to a feature image
    FeatureImage& reference_image = *images[ref_camera_id].get();

    // get pointer to best feature (according to score and image region)
    Feature* feature =
        (use_frustrum)?reference_image.GetBestFeature( &roi[0] ):
      reference_image.GetBestFeature();

    // if we ran out of features exit
    if (!feature) {
      PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
                   "    ERROR: No more features to test.\n");
      break;
    }

    // Check that this feature is not mask
    if (feature_mask.GetMask(ref_camera_id, feature->x, feature->y)) {
      feature->used = true;
      continue;
    }

    feature_mask.SetMask(ref_camera_id, feature->x, feature->y);
    PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
                 "    Next best feature in camera-%d : [%.3f %.3f]  "
                 "scale: %0.3f\n",
                 ref_camera_id, feature->x, feature->y, feature->scale);

    // if more than one camera is in use, let's see if we
    // can find correspondences
    Sophus::SO3t patch_orientation;
    Scalar radius_at_cam_plane;
    if (GetMultiViewMatchesIfPossible(rig,
                                      ref_camera_id,
                                      images,
                                      feature,
                                      matches,
                                      patch_orientation,
                                      radius_at_cam_plane,
                                      feature_options) == false ) {
      continue;
    }

    // count number of good matches
    int num_matches=0;
    for (auto& flag : matches.vFlags) {
      if (flag == GoodMatch) {
        num_matches++;
      }
    }

    // for the moment we need a minimum number of stereo matches
    if (num_cameras > 1 && num_matches < 2) {
      if(num_tracked_multiview_landmarks < 16 ||
         !g_tracking_cvars.init_mono_landmarks) {
        continue;
      } else {
        use_frustrum = false;
      }
    }

    // Find 3d with multi-view, if possible, if not, point Xr will
    // still be valid for inverse depth initialization
    TriangulateIfPossible(rig, ref_camera_id, matches.vFeatures, x_r);

//    std::cout << "Attempting to initialize: " << x_r.transpose <<
//                 " fromm " << num_matches << " matches: ";
//    for(unsigned int  ii=0; ii < num_matches; ii++) {
//      std::cout << "cam: ii"
//      std::cout << matches.FeatureMatches
//    }

    bool bSuccess =
        InitLandmark(rig, x_r, patch_orientation, radius_at_cam_plane,
                     num_matches < 2 ? eLmkAtInfinity : eLmkActive,
                     ref_camera_id, images, map, current_frame, matches);

    // if init landmark failed try another landmark
    if (!bSuccess) {
      continue;
    }

    UpdateQuadTrees(matches.vFeatures,images);

    num_new_landmarks++;
    if (num_matches > 1) {
      num_tracked_multiview_landmarks++;
    }
  }

  PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
               "    Num new features tested: %u \n", num_feat_tested);

  if (g_tracking_cvars.startnewlandmarks_debug_level == 0) {
    printf(" QuadTrees after adding NEW landmarks \n");
    for (size_t cam_id = 0; cam_id < rig.cameras.size(); ++cam_id) {
      printf(" QuadTree camera [%lu]: \n", cam_id );
      images[cam_id]->GetQuadTreeRef().PrintTree();
    }
  }

  PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
               "    Tracked %d\n", num_tracked_landmarks);
  PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
               "    Added %d\n", num_new_landmarks);
  PrintMessage(g_tracking_cvars.startnewlandmarks_debug_level,
               "</StartNewLandmarks>\n\n");
}

}
}
