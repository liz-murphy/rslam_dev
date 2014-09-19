#include <sparse_tracking/EstimateRelativePose.h>
#include <slam_map/SlamMap.h>

namespace rslam {
namespace sparse {

const char* Pose2Str( const Eigen::Matrix4t& T )
{
  static std::string sStr;
  Eigen::Vector6t x = rslam::T2Cart(T);
  char buf[64];
  snprintf( buf, 64, "[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]'\n",
            x(0), x(1), x(2), x(3), x(4), x(5));
  sStr = buf;
  return sStr.c_str();
}

Scalar CalcSigma(const LocalMap &work_set,
                 const Sophus::SE3t &t_ab,
                 const std::vector<MultiViewMeasurement> &measurements) {
  std::map<SessionId, CameraRigPtr> rigs = work_set.cameras;

  std::vector<Scalar> errors;
  for (const MultiViewMeasurement& z : measurements) {
    CameraRigPtr rig_ptr = rigs.at(z.id().frame_id.session_id);
    for (size_t cam_id = 0; cam_id < z.NumCameras(); ++cam_id) {
      if (z.HasGoodMeasurementInCam(cam_id)) {

        // compute reprojection error
        const Sophus::SE3t  t_sw  =
            (t_ab * rig_ptr->cameras[cam_id].T_wc).inverse();
        auto it = work_set.landmarks.find(z.id().landmark_id);
        if (it == work_set.landmarks.end()) {
          LOG(FATAL) << "Tried to find landmark " << z.id() << " in work set"
                        " but it was not there.";
        }
        const Eigen::Vector4t x_w =it->second->x_w;
        const Eigen::Vector4t x_s = Sophus::MultHomogeneous(t_sw, x_w);
        const Eigen::Vector2t p   =
            rig_ptr->cameras[cam_id].camera.Project(x_s.head<3>());
        errors.push_back( (p - z.Pixel(cam_id)).squaredNorm() );
      }
    }
  }
  // now get median as our estimate of sigma
  if (!errors.empty()) {
    std::sort(errors.begin(), errors.end());
    Scalar sigma = std::sqrt(errors[errors.size()/2]);
    if( sigma < 0.1 ){
      sigma += 0.1;
    }
    return sigma;
  }
  std::cerr << "Warning. [EstimateRelativePose::CalcSigma] "
            << "Bad Sigma, there are no good measurements" << std::endl;
  return DBL_MAX;
}

int DoReThreading(const LocalMap &work_set,
                  const Sophus::SE3t &t_ab,
                  std::vector<MultiViewMeasurement> &measurements,
                  const FeatureImageVector &images) {
  int num_rethreaded_measurements = 0;
  double width = (double)images[0]->Width();
  double height = (double)images[0]->Height();

  std::map<SessionId, CameraRigPtr> rigs = work_set.cameras;

  Landmark lm;
  for (MultiViewMeasurement& z: measurements) {
    // try to find measurement
    const std::shared_ptr<LandmarkContainer>& lmc =
        work_set.landmarks.at(z.id().landmark_id);
    const Eigen::Vector4t& x_w   = lmc->x_w;
    if (!work_set.map->GetLandmark(z.id().landmark_id, &lm) ||
        lm.state() == eLmkAtInfinity) {
      continue;
    }

    CameraRigPtr rig_ptr = rigs.at(z.id().frame_id.session_id);
    for (size_t cam_id = 0; cam_id < rig_ptr->cameras.size(); ++cam_id) {
      if (z.HasGoodMeasurementInCam(cam_id)) {
        continue;
      }

      // project landmark
      const auto&           camera = rig_ptr->cameras[cam_id].camera;
      const Sophus::SE3t&   t_ws   = rig_ptr->cameras[cam_id].T_wc;
      const Sophus::SE3t    t_sw   = (t_ab * t_ws).inverse();
      const Eigen::Vector4t x_s    = Sophus::MultHomogeneous(t_sw, x_w);
      const Eigen::Vector2t p      = camera.Project(x_s.head<3>());

      // check that is inside the image
      if (p(0) < 4 || p(1) < 4 || p(0) >= width - 4 || p(1) >= height - 4) {
        continue;
      }

      // use ESM to match
      float refined_x = p(0);
      float refined_y = p(1);

      PatchHomography<CANONICAL_PATCH_SIZE> homography =
          lmc->tracking_homographies[cam_id];
      homography.SetTranslation(refined_x, refined_y);
      FeatureImage* image = images[cam_id].get();
      double error =
          image->RefineSubPixelXY(lm.patch(), homography, refined_x, refined_y);
      homography.SetTranslation(refined_x, refined_y);
      const Eigen::Vector2t pr(refined_x, refined_y);
      const Eigen::Vector2t e = pr - p;

      if (error < g_common_cvars.esm_threshold &&
          e.norm() < g_common_cvars.esm_subpixel_threshold) {
        z.SetFlag(cam_id, GoodMatch);
        z.SetPixelInCam(cam_id, refined_x, refined_y);
        z.SetPatchHomography(cam_id, homography);
        num_rethreaded_measurements++;
      }
    }
  }
  return num_rethreaded_measurements;
}

int FlagOutliers(
    const float                          outlier_threshold,
    const MatchFlag                      outlier_flag,
    const LocalMap                       &work_set,
    const Sophus::SE3t                   &t_ab,
    const SlamMap                        &map,
    std::vector<MultiViewMeasurement>    &measurements,
    double                               &msre,
    int* infinity_lms)
{
  PrintMessage(g_tracking_cvars.flagoutliers_debug_level, "<FlagOutliers>\n");

  int num_inliers = 0;
  int num_landmarks_at_infinity = 0;
  msre = 0.0;

  std::map<SessionId, CameraRigPtr> rigs = work_set.cameras;

  // compute reprojection error
  Landmark lm;
  for (MultiViewMeasurement& z : measurements) {
    const LandmarkId& lm_id = z.id().landmark_id;
    const Eigen::Vector4t x_w = work_set.landmarks.at(lm_id)->x_w;

    // if the landmark is at infinity don't change its measurements' flag
    if (!map.GetLandmark(lm_id, &lm)) {
      continue;
    } else if (lm.state() == eLmkAtInfinity) {
      num_landmarks_at_infinity++;
      continue;
    }

    auto it = rigs.find(z.id().frame_id.session_id);
    if (it == rigs.end()) {
      LOG(WARNING) << "Camera rig missing for " <<  z.id()
                   << std::endl;
      for (size_t cam_id = 0; cam_id < z.NumCameras(); ++cam_id) {
        z.SetFlag(cam_id, outlier_flag);
      }
      continue;
    }
    CameraRigPtr rig_ptr = it->second;

    for (size_t cam_id = 0; cam_id < z.NumCameras(); ++cam_id) {
      if(!z.HasGoodMeasurementInCam(cam_id)){
        continue;
      }
      // compute reprojection error
      const auto&        camera = rig_ptr->cameras[cam_id].camera;
      const Sophus::SE3t&  t_ws = rig_ptr->cameras[cam_id].T_wc;
      const Sophus::SE3t   t_sw = (t_ab * t_ws).inverse();
      const Eigen::Vector4t x_s = Sophus::MultHomogeneous(t_sw, x_w);
      const Eigen::Vector2t   p = camera.Project( x_s.head<3>() );
      const Eigen::Vector2t   e = p - z.Pixel(cam_id);
      const Scalar error = e.norm();

      PrintMessage(g_tracking_cvars.flagoutliers_debug_level,
                   "FlagOutliers: LM %s cam[%d] z=[%.2f, %.2f], "
                   "hx=[%.2f, %.2f] error %f\n",
                   lm.id(), cam_id, z.Pixel(cam_id)[0], z.Pixel(cam_id)[1],
                   p[0], p[1], error);

      z.SetReprojectionError(cam_id, error);

      //if "dist" to model is > than the inlier threshold ->FLAG as outlier
      if (error > outlier_threshold ){
        PrintMessage(g_tracking_cvars.flagoutliers_debug_level,
                     "FlagOutliers %s reprojection error %f too big\n",
                     lm.id(), error);
        z.SetFlag(cam_id, outlier_flag);
      } else {
        z.SetFlag(cam_id, GoodMatch);
        msre += error;
        num_inliers++;
      }
    }
  }

  if (num_inliers > 0) {
    msre /= num_inliers;
  } else {
    msre = DBL_MAX;
  }
  PrintMessage(g_tracking_cvars.flagoutliers_debug_level, "</FlagOutliers>\n");

  if (infinity_lms) {
    *infinity_lms = num_landmarks_at_infinity;
  }
  return num_inliers;
}

void LiftTrackingData(const CameraRigPtr& rig,
                      const Sophus::SE3t &t_ab,
                      const SlamMap &map,
                      LocalMap &local_map)
{
  // get tracking homographies for the current frame.
  unsigned int num_cameras = rig->cameras.size();
  local_map.t_sw.resize( num_cameras );

  for (size_t cam_id = 0; cam_id < num_cameras; ++cam_id) {
    local_map.t_sw[cam_id] = (t_ab * rig->cameras[cam_id].T_wc).inverse();
  }

  Landmark lm;
  for (auto& pair: local_map.landmarks) {
    const LandmarkId& lmid = pair.first;
    if (!map.GetLandmark(lmid, &lm)) {
      continue;
    }

    std::shared_ptr<LandmarkContainer>& lmc = pair.second;
    // landmarks reference frame woot
    const auto it = local_map.poses.find(lmid.ref_frame_id);
    // Twl is lifted pose of base-frame for this lm
    const Sophus::SE3t& t_wv = it->second.t_wp;

    lmc->tracking_homographies.resize(num_cameras);

    Eigen::Vector3t tl3d_lp, tr3d_lp, br3d_lp, bl3d_lp;
    lm.Get3DPatchCorners(tl3d_lp, tr3d_lp, bl3d_lp, br3d_lp);

    // tranform from base-frame (Twl) to world:
    const Eigen::Vector3t tl3d_wp = t_wv * tl3d_lp;
    const Eigen::Vector3t tr3d_wp = t_wv * tr3d_lp;
    const Eigen::Vector3t br3d_wp = t_wv * br3d_lp;
    const Eigen::Vector3t bl3d_wp = t_wv * bl3d_lp;
    const Eigen::Vector3t c3d_wp = t_wv * lm.xrp().head<3>();

    for (size_t cam_id = 0; cam_id < num_cameras; ++cam_id)  {
      // get lm 3D patch corners wrt to tracking frame (Tab)
      const calibu::CameraModelGeneric<Scalar>& cam =
          rig->cameras[cam_id].camera;

      lmc->tracking_homographies[cam_id] =
          PatchHomography<CANONICAL_PATCH_SIZE>::
          Transfer3DPatchHomography(tl3d_wp, tr3d_wp, bl3d_wp,
                                    br3d_wp, c3d_wp, cam,
                                    local_map.t_sw[cam_id]);
    }
  }
}


bool EstimateRelativePose(const ReferenceFrameId &frame_id,
                          const FeatureHandler::Options &options,
                          LocalMap &working_set,
                          BackEnd &backend,
                          const FeatureImageVector &images,
                          std::vector<MultiViewMeasurement> &new_measurements,
                          std::vector<std::vector<Feature *> > &feature_matches,
                          const bool no_tic, Sophus::SE3t &t_ab,
                          double learning_rate,
                          common::SystemStatus& system_status,
                          const std::shared_ptr<Timer> &timer,
                          std::vector<ba::ImuMeasurementT<Scalar> >& meas,
                          bool has_imu) {
  const SlamMap& map = *working_set.map;
  const CameraRigPtr& current_rig = working_set.cameras[frame_id.session_id];
  CHECK(current_rig) << "No rig found for " << frame_id;

  system_status.num_inliers = 0;

  //=================================================================
  // Predict landmark positions and patch homography
  // in the current images
  //=================================================================
  LiftTrackingData(current_rig, t_ab, map, working_set);

  //=================================================================
  // Find 3D-2D correspondences between the landmarks in the
  // working set and the detected features in the current images.
  // Cicle until enough matches have been found (60% of working set)
  //=================================================================
  if (!no_tic) timer->Tic("MatchInTime");
  bool have_enough_matches = false;
  float search_radius = g_common_cvars.initial_search_radius;
  int matching_attempts = 0;
  int max_attempts = g_common_cvars.num_match_in_time_attempts;
  while (!have_enough_matches && ++matching_attempts <= max_attempts) {
    // compute matches
    system_status.num_mit_matches =
        MatchInTime(frame_id, images, working_set, options, new_measurements,
                    feature_matches, search_radius);

    PrintMessage(g_tracking_cvars.matchintime_debug_level,
                 "MATCH-IN-TIME num tentative matches: %d\n",
                 system_status.num_mit_matches);

    have_enough_matches =
        system_status.num_mit_matches >= g_tracking_cvars.min_tracked_features;
    if (!have_enough_matches) {
      // increase search area
      PrintMessage(g_tracking_cvars.matchintime_debug_level,
                   "MIT FAILURE -> Increasing Search area \n");
      search_radius *= g_common_cvars.search_radius_grow_rate;
    }
  }
  if (!no_tic) timer->Toc("MatchInTime");

  if (matching_attempts > max_attempts) {
    return false;
  }

  //=================================================================
  // Filter outliers in the 3D-2D tentative matches set using Ransac
  //=================================================================
  if (!no_tic) timer->Tic("RANSAC");

  calibu::CameraRigT<Scalar> rig;
  std::map< SessionId, std::map<unsigned int, unsigned int> > ransac_cam_ids;
  unsigned int cam_count = 0;
  for (auto& rig_pair : working_set.cameras) {
    CameraRigPtr rig_ptr = rig_pair.second;
    for( unsigned int cam_id = 0; cam_id < rig_ptr->cameras.size(); ++cam_id) {
      rig.Add(rig_ptr->cameras[cam_id]);
      ransac_cam_ids[rig_pair.first][cam_id] = cam_count++;
    }
  }

  // generate a set of valid 3d-2d correspondences from the tentative
  // matches
  std::vector<Eigen::Vector4t> points_3d;
  std::vector<Eigen::Vector2t> points_2d;
  std::vector<unsigned int>    cam_ids;

  int num_finite_lmks = 0;
  for (MultiViewMeasurement& z : new_measurements) {
    SessionId z_session_id = z.id().frame_id.session_id;
    Landmark lm;
    if (!working_set.map->GetLandmark(z.id().landmark_id, &lm) ||
        (lm.state() == eLmkAtInfinity)) {
      continue;
    }
    ++num_finite_lmks;
    auto cam_it = working_set.cameras.find(z_session_id);
    if (cam_it == working_set.cameras.end()) continue;
    CameraRigPtr rig_ptr = cam_it->second;

    Eigen::Vector4t x_w = working_set.landmarks.at(z.id().landmark_id)->x_w;
    for (size_t cam_id = 0; cam_id < z.NumCameras(); ++cam_id) {
      if (z.HasGoodMeasurementInCam(cam_id)) {
        points_3d.push_back(x_w);
        points_2d.push_back(z.Pixel(cam_id));
        cam_ids.push_back(ransac_cam_ids[z_session_id][cam_id]);
      }
    }
  }

  // If there are tracked landmarks available, but we haven't tracked
  // any, then consider this failed.
  if (num_finite_lmks != 0 && points_2d.size() < 16) {
    if (!no_tic) timer->Toc("RANSAC");
    LOG(WARNING) << "Not enough points to estimate relative pose: "
                 << points_2d.size();
    return false;
  }

  Sophus::SE3t ransac_t_ab = t_ab;
  if (g_tracking_cvars.do_ransac) {
    if (!Ransac(points_3d, points_2d, cam_ids, rig,
                &system_status.rng, ransac_t_ab)) {
      if (!no_tic) timer->Toc("RANSAC");
      return false;
    }

    t_ab = ransac_t_ab;
  }

  // How many landmarks are at infinity
  int infinity_lms = 0;
  system_status.num_inliers =
      FlagOutliers( g_common_cvars.ransac_outlier_threshold,
                    RansacOutlier,
                    working_set,
                    ransac_t_ab,
                    map,
                    new_measurements,
                    system_status.rmsre,
                    &infinity_lms);

  PrintMessage(g_tracking_cvars.estimate_debug_level,
               "RANSAC inliers: %d, inlierPercent: %f\n",
               system_status.num_inliers,
               float(system_status.num_inliers) /
               g_common_cvars.num_features_to_track);

  // failure case
  if (system_status.num_inliers + infinity_lms < 16) {
    LOG(ERROR) << "WARNING: RANSAC failure, using motion "
               << "model alone if possible";
    if(!no_tic) timer->Toc("RANSAC");
    return false;
  }
  if (!no_tic) timer->Toc("RANSAC");

  //=================================================================
  // Estimate the frame-to-frame transform using all the inliers
  //=================================================================
  // if there is an IMU, don't do frame to frame gauss newton. An inertial
  // guess should be provided in t_ab.
  if (true) {
    if (!no_tic) timer->Tic("GaussNewton");
    backend.GaussNewton(working_set, new_measurements, map, t_ab, has_imu,
                        meas);

    // get sigma
    Scalar sigma = CalcSigma(working_set, t_ab, new_measurements);


    system_status.inlier_noise_error =
        learning_rate * (system_status.inlier_noise_error) +
        (1.0 - learning_rate)*sigma;
    Scalar outlier_threshold = 2 * (system_status.inlier_noise_error);

    system_status.num_inliers =
        FlagOutliers(outlier_threshold,
                     GaussNewtonOutlier,
                     working_set,
                     t_ab,
                     map,
                     new_measurements,
                     system_status.rmsre,
                     nullptr);


    if (!no_tic) timer->Toc("GaussNewton");

    //=================================================================
    // Given the estimated transform, try to find missing 3D-2D matches
    //=================================================================
    if (g_tracking_cvars.do_rethreading) {
      if (!no_tic) timer->Tic("Rethreading");
      int rethreaded_msr = DoReThreading(working_set, t_ab,
                                         new_measurements, images);
      PrintMessage(g_tracking_cvars.estimate_debug_level,
                   "Rethreaded msr: %d\n", rethreaded_msr );
      backend.GaussNewton(working_set, new_measurements, map, t_ab, has_imu,
                          meas);
      FlagOutliers(outlier_threshold,
                   GaussNewtonOutlier,
                   working_set,
                   t_ab,
                   map,
                   new_measurements,
                   system_status.rmsre, nullptr);
      if (!no_tic) timer->Toc("Rethreading");
    }
  }
  return true;
}

}  // namespace sparse
}  // namespace rslam
