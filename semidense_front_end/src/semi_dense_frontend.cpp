#include <semidense_front_end/semi_dense_frontend.h>
#include <common_front_end/CommonFrontEndConfig.h>
//#include <SparseFrontEnd/FrontEndCVars.h>
//#include <SemiDenseFrontEnd/vitrack-cvars.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/SlamMap.h>

using namespace rslam;

namespace rslam {
namespace sd {
namespace timer {
static const std::string kIterate = "Iterate";
static const std::string kTrackSD = "Track SD";
static const std::string kAddImage = "Add Image";
static const std::string kOptimize = "Optimize";
static const std::string kPrune = "Prune Tracks";
static const std::string kUpdateMap = "Update Map";
static const std::string kBA = "BA";
static const std::string kSNL = "SNL";
}  // namespace rslam
}  // namespace sd
}  // namespace timer


SemiDenseFrontEnd::SemiDenseFrontEnd() : is_quitting_(false) {}

SemiDenseFrontEnd::~SemiDenseFrontEnd() {
  is_quitting_ = true;
  aac_cond_.notify_all();
  if(aac_thread_.joinable()){
    aac_thread_.join();
  }
}

inline void mats_for_image_array(const std::shared_ptr<pb::ImageArray> &frames,
    std::vector<cv::Mat>* mats){
  mats->clear();
  mats->reserve(frames->Size());
  for(int i=0; i < frames->Size(); ++i)
  {
    mats->push_back(frames->at(i)->Mat());
  }
}

void SemiDenseFrontEnd::configCallback(semidense_front_end::SemiDenseConfig &config, uint32_t level)
{
  pyramid_levels_ = config.pyramid_levels;
  patch_size_ = config.patch_size;
  num_features_ = config.num_features;
  feature_cells_ = config.feature_cells;
  tracker_center_weight_ = config.tracker_center_weight;
  ncc_threshold_ = config.ncc_threshold;
  lmk_outlier_ratio_threshold_ = config.lmk_outlier_ratio_threshold;
  use_imu_measurements_ = config.use_imu_measurements;
  use_imu_for_guess_ = config.use_imu_for_guess;
  min_poses_for_imu_ = config.min_poses_for_imu;

}

bool SemiDenseFrontEnd::Init(const calibu::CameraRigT<Scalar> &rig,
                             const std::shared_ptr<pb::ImageArray> &frames,
                             const double timestamp, const bool has_imu,
                             const std::shared_ptr<SlamMap> &map,
                             const std::shared_ptr<PlaceMatcher> &place_matcher,
                             const std::shared_ptr<Timer> &timer,
                             bool using_sim_data) {
  is_simulation_ = using_sim_data;
  has_imu_ = has_imu;
  map_ = map;
  place_matcher_ = place_matcher;
  timer_ = timer;
  timer_->set_window_size(CommonFrontEndConfig::getConfig()->getTimerWindowSize());
  old_rig_ = rig;
  calibu::CreateFromOldRig(&rig, &rig_);
  timer_->set_window_size(CommonFrontEndConfig::getConfig()->getTimerWindowSize());

  hold_frame_token_ = map_->GetHoldToken();

  int patch_size = 9;
  sdtrack::KeypointOptions keypoint_options;
  keypoint_options.gftt_feature_block_size = patch_size;
  keypoint_options.max_num_features = num_features_ * 2;
  keypoint_options.gftt_min_distance_between_features = 3;
  keypoint_options.gftt_absolute_strength_threshold = 0.005;
  sdtrack::TrackerOptions tracker_options;
  tracker_options.pyramid_levels = pyramid_levels_;
  tracker_options.detector_type = sdtrack::TrackerOptions::Detector_GFTT;
  tracker_options.num_active_tracks = num_features_;
  tracker_options.use_robust_norm_ = false;
  tracker_options.robust_norm_threshold_ = 30;
  tracker_options.patch_dim = patch_size_;
  tracker_options.default_rho = 1.0/5.0;
  tracker_options.feature_cells = feature_cells_;
  tracker_options.iteration_exponent = 2;
  tracker_options.center_weight = tracker_center_weight_;
  tracker_options.dense_ncc_threshold = ncc_threshold_;
  tracker_options.harris_score_threshold = 2e6;
  tracker_options.gn_scaling = 1.0;
  tracker_.Initialize(keypoint_options, tracker_options, &rig_);

  // Start the first frame.
  current_frame_ = map_->AddFrame(timestamp);
  if (has_imu_) {
    current_frame_->set_b(Eigen::Vector6t::Zero());
    current_frame_->set_v_r(Eigen::Vector3t::Zero());
    current_frame_->set_g_r(
    optimization_.GetImuBuffer().elements.front().a.normalized() *
    ba::Gravity);
  }
  current_frame_->set_t_vs(rig_.t_wc_[0]);
  current_frame_->set_cam_params(rig_.cameras_[0]->GetParams());

  // Add a new image and start landmarks on the tracker. There is no need to
  // add these landmarks to the map yet, as we do not know if they tracked.
  Sophus::SE3d guess;// = prev_delta_t_ba_ * prev_t_ba_;
  if(!guess.translation().any()) {
    guess.translation() = Eigen::Vector3d(0,0,0.1);
  }
  std::vector<cv::Mat> images;
  mats_for_image_array(frames, &images);

  tracker_.AddImage(images, guess);
  tracker_.AddKeyframe();
  tracker_.StartNewLandmarks();

  for (std::shared_ptr<sdtrack::DenseTrack> track: tracker_.GetNewTracks()) {
    track->external_data = current_frame_->id().id;
    track->keypoints.back()[0].external_data = current_frame_->id().id;
  }
  keyframe_tracks_ = tracker_.GetNewTracks().size();

  optimization_.Init(map_);
  optimization_callbacks_.visual_callback = [this](
      const ba::VisualBundleAdjuster<Scalar>& ba,
      const rslam::optimization::LiftResults& results) {
    if (results.pose_ba_ids.empty()) return;

    // Now update all the tracks.

    // Start with pose 0 (the head pose) and put them all relative to that.
    Sophus::SE3t current_t_pw = ba.GetPose(0).t_wp.inverse();

    // For each track find the t_ba to the current pose.
    for (const std::shared_ptr<sdtrack::DenseTrack>& track :
             tracker_.GetCurrentTracks()) {
      ReferenceFrameId ref_pose_id(track->external_data, map_->id());
      LandmarkId lmk_id(ref_pose_id, track->external_data2);
      auto lmk_it = results.landmark_ba_ids.find(lmk_id);
      if (lmk_it == results.landmark_ba_ids.end()) continue;

      const ba::LandmarkT<double>& lmk = ba.GetLandmarkObj(lmk_it->second);
      const Sophus::SE3t ref_t_wp = ba.GetPose(lmk.ref_pose_id).t_wp;

      track->t_ba = current_t_pw * ref_t_wp;

      // Set the rho from the landmark.
      Eigen::Vector4t x_r =
          Sophus::MultHomogeneous((ref_t_wp * rig_.t_wc_[track->ref_cam_id]).inverse(),
          //Sophus::MultHomogeneous((ref_t_wp * rig_.t_wc_[0]).inverse(),
                                  lmk.x_w);
      // Normalize the xyz component of the ray to compare to the original ray.
      x_r /= x_r.head<3>().norm();
      track->ref_keypoint.rho = x_r[3];

      if (ba.LandmarkOutlierRatio(lmk.id) > lmk_outlier_ratio_threshold_ && !track->tracked) {
        track->is_outlier = true;
      }
      else
      {
        track->is_outlier = false;
      }
    }
  };

  aac_thread_ = std::thread(&SemiDenseFrontEnd::AsyncBaFunc,this);

  system_status_.is_initialized = true;
  return true;
}

void SemiDenseFrontEnd::RegisterImuMeasurement(const Eigen::Vector3t &w,
                                               const Eigen::Vector3t &a,
                                               const double time) {
  optimization_.RegisterImuMeasurement(w, a, time);
  aac_optimization_.RegisterImuMeasurement(w, a, time);
}

void SemiDenseFrontEnd::CreateLandmark(
    const std::shared_ptr<sdtrack::DenseTrack>& track,
    LandmarkId* lm_id) {
  CHECK_NOTNULL(lm_id);
  CHECK_EQ(rig_.cameras_.size(), track->keypoints.front().size());

  ReferenceFrameId ref_pose_id(track->external_data, map_->id());
  SlamFramePtr ref_pose = map_->GetFramePtr(ref_pose_id);
  lm_id->landmark_index = ref_pose->NumLandmarks();
  lm_id->ref_frame_id = ref_pose_id;

  MeasurementId meas_id(ref_pose_id, *lm_id);

  MultiViewMeasurement meas(rig_.cameras_.size());
  meas.set_id(meas_id);

  for (size_t cam_idx = 0; cam_idx < rig_.cameras_.size(); ++cam_idx) {
    const sdtrack::Keypoint& kp = track->keypoints[0][cam_idx];
    if (kp.tracked) {
      meas.SetPixelInCam(cam_idx, kp.kp[0], kp.kp[1]);
      meas.SetFlag(cam_idx, GoodMatch);
      meas.SetPatchHomography(cam_idx,
                              PatchHomography<CANONICAL_PATCH_SIZE>(
                                  kp.kp[0], kp.kp[1], 1.0));
    } else {
      meas.SetFlag(cam_idx, MissingFeature);
    }
  }

  Eigen::Vector4d ray;
  ray.head<3>() = track->ref_keypoint.ray / track->ref_keypoint.rho;
  ray[3] = 1.0;

  // Move the landmark from the camera frame to the vehicle frame (where
  // the map stores it)
  ray = Sophus::MultHomogeneous(rig_.t_wc_[track->ref_cam_id], ray);
  //ray = Sophus::MultHomogeneous(rig_.t_wc_[0], ray);
  Eigen::Vector3t rt(0, 1, 0);
  Eigen::Vector3t fw = -ray.head<3>();          // center of cam to lm
  Eigen::Vector3t dn = fw.cross(rt);
  rt = dn.cross(fw);
  fw.normalize();
  rt.normalize();
  dn.normalize();

  Eigen::Matrix3t orientation_mat_c;
  orientation_mat_c.col(0) = fw;
  orientation_mat_c.col(1) = rt;
  orientation_mat_c.col(2) = dn;

  Landmark lm;
  lm.Init(meas_id, true, *lm_id, track->ref_cam_id, ray,
          Sophus::SO3t(orientation_mat_c), 1.0, nullptr, 0,
          Landmark::PatchVectorT());
  lm.set_state(eLmkActive);
  ref_pose->AddMeasurement(meas);
  ref_pose->AddLandmark(lm);
  track->external_data2 = lm_id->landmark_index;
}

void SemiDenseFrontEnd::AddKeyframe() {
  // Now look at any landmarks that were successfully tracked, and add new
  // landmarks for them.
  LandmarkId lm_id;
  MultiViewMeasurement meas(rig_.cameras_.size());

  ++system_status_.keyframe_number;
  for (std::shared_ptr<sdtrack::DenseTrack> track :
           tracker_.GetCurrentTracks()) {
    for (size_t cam_idx = 0; cam_idx < rig_.cameras_.size(); ++cam_idx) {
      track->keypoints.back()[cam_idx].external_data = current_frame_->id().id;
      //track->ref_cam_id = 0;//  cam_idx;
    }

    if (track->keypoints.size() < 2) continue;

    bool new_lm = (track->keypoints.size() >= 2 &&
                   track->external_data2 == UINT_MAX);
    if (new_lm) {
      CreateLandmark(track, &lm_id);
    } else {
      lm_id.ref_frame_id =
          ReferenceFrameId(track->external_data, map_->id());
      lm_id.landmark_index = track->external_data2;
    }

    // For a new landmark or keyframe, we must add all measurements or
    // the latest one respectively.
    size_t num_kps = track->keypoints.size();
    for (size_t ii = (new_lm ? 1 : num_kps - 1); ii < num_kps; ++ii) {
      ReferenceFrameId meas_pose_id(track->keypoints[ii][0].external_data,
                                    map_->id());
      meas.set_id(MeasurementId(meas_pose_id, lm_id));

      for (size_t cam_idx = 0; cam_idx < rig_.cameras_.size(); ++cam_idx) {
        const sdtrack::Keypoint& kp = track->keypoints[ii][cam_idx];
        if (kp.tracked) {
          meas.SetPixelInCam(cam_idx, kp.kp[0], kp.kp[1]);
          meas.SetFlag(cam_idx, GoodMatch);
          meas.SetPatchHomography(cam_idx,
                                  PatchHomography<CANONICAL_PATCH_SIZE>(
                                      kp.kp[0], kp.kp[1], 1.0));
        } else {
          meas.SetFlag(cam_idx, MissingFeature);
        }
      }
      std::vector<MultiViewMeasurement> measurements;
      measurements.push_back(meas);
      map_->AddNewMeasurementsToFrame(meas_pose_id, measurements);
    }
  }
}


bool SemiDenseFrontEnd::Iterate(const std::shared_ptr<pb::ImageArray> &frames,
                                double timestamp) {
  ++system_status_.frame_number;
  system_status_.time = timestamp;

  // If this is a keyframe, set it as one on the tracker.
  prev_delta_t_ba_ = tracker_.t_ba() * prev_t_ba_.inverse();

  if (is_prev_keyframe_) {
    prev_t_ba_ = Sophus::SE3t();
  } else {
    prev_t_ba_ = tracker_.t_ba();
  }

  if (is_prev_keyframe_) {
    // Add the node and edge
    prev_frame_ = current_frame_;
    current_frame_  = map_->AddFrame(system_status_.time);
    current_edge_ = map_->AddEdge(prev_frame_, current_frame_, Sophus::SE3t());
  }

  Sophus::SE3t guess = prev_delta_t_ba_ * prev_t_ba_;
  if(!guess.translation().any()) {
    guess.translation()[2] = 0.001;
  }

  bool used_imu_for_guess = false;
  if (use_imu_measurements_ &&
      use_imu_for_guess_ &&
      system_status_.keyframe_number >= min_poses_for_imu_) {

    ba::PoseT<Scalar> start_pose;
    start_pose.t_wp = Sophus::SE3t();
    start_pose.b = prev_frame_->b();
    start_pose.v_w = prev_frame_->v_r();
    start_pose.time = prev_frame_->time();
    // Integrate the measurements since the last frame.
    std::vector<ba::ImuMeasurementT<Scalar> > meas =
        optimization_.GetImuBuffer().GetRange(prev_frame_->time(),
                                        current_frame_->time());

    std::vector<ba::ImuPoseT<Scalar> > imu_poses;
    ba::VisualInertialBundleAdjuster<Scalar>::ImuResidual::IntegrateResidual(
        start_pose, meas, start_pose.b.head<3>(), start_pose.b.tail<3>(),
        optimization_.GetImuCalibration().g_vec, imu_poses);

    if (imu_poses.size() > 1) {
      ba::ImuPoseT<Scalar>& last_pose = imu_poses.back();
      guess = last_pose.t_wp.inverse() * imu_poses.front().t_wp;
      current_edge_->set_transform(guess);
      current_frame_->set_v_r(guess.so3() * last_pose.v_w);
      used_imu_for_guess = true;
    }
  }

  Tic(sd::timer::kIterate);
  Tic(sd::timer::kTrackSD);

  std::vector<cv::Mat> images;
  mats_for_image_array(frames, &images);

  Tic(sd::timer::kAddImage);
  tracker_.AddImage(images, guess);
  Toc(sd::timer::kAddImage);

  // Optimize all levels.
  Tic(sd::timer::kOptimize);
  tracker_.OptimizeTracks(-1);
  Toc(sd::timer::kOptimize);

  // Get rid of any outliers.
  Tic(sd::timer::kPrune);
  tracker_.PruneTracks();
  Toc(sd::timer::kPrune);

  if (CommonFrontEndConfig::getConfig()->doKeyframing()) {
    const double track_ratio = (double)tracker_.num_successful_tracks() /
        (double)keyframe_tracks_;
    const double total_trans = tracker_.t_ba().translation().norm();
    const double total_rot = tracker_.t_ba().so3().log().norm();

    bool keyframe_condition = track_ratio < 0.8 || total_trans > 0.2 ||
        total_rot > 0.1;

    if (keyframe_tracks_ != 0) {
      if (keyframe_condition) {
        is_keyframe_ = true;
      } else {
        is_keyframe_ = false;
      }
    }

    // If this is a keyframe, set it as one on the tracker.
    prev_delta_t_ba_ = tracker_.t_ba() * prev_t_ba_.inverse();

    if (is_keyframe_) {
      tracker_.AddKeyframe();
    }
    is_prev_keyframe_ = is_keyframe_;
  } else {
    tracker_.AddKeyframe();
  }
  Toc(sd::timer::kTrackSD);

  Tic(sd::timer::kUpdateMap);
  if (is_keyframe_) {
    AddKeyframe();
  }
  Toc(sd::timer::kUpdateMap);

  map_->SetHoldFrames(hold_frame_token_,
                      current_frame_->id(),
                      CommonFrontEndConfig::getConfig()->getTrackerHoldDepth(),
                      true);

  if (is_keyframe_) {
    Tic(sd::timer::kBA);
    if (CommonFrontEndConfig::getConfig()->doBundleAdjustment()) {
      DoSynchronousBundleAdjustment();
    }
    Toc(sd::timer::kBA);

    // Now start new landmarks.
    Tic(sd::timer::kSNL);
    tracker_.StartNewLandmarks();
    if (!CommonFrontEndConfig::getConfig()->doBundleAdjustment()) {
      tracker_.TransformTrackTabs(tracker_.t_ba());
    }
    Toc(sd::timer::kSNL);

    keyframe_tracks_ = tracker_.GetCurrentTracks().size();

    for (std::shared_ptr<sdtrack::DenseTrack> track: tracker_.GetNewTracks()) {
      track->external_data = current_frame_->id().id;
    }
  } else {
    Tic(sd::timer::kBA);
    Toc(sd::timer::kBA);
    Tic(sd::timer::kSNL);
    Toc(sd::timer::kSNL);
  }

  // We always want to update the edge based on the tracking results.
  if (SlamEdgePtr edge = map_->GetEdgePtr(prev_frame_->id(),
                                          current_frame_->id())) {
    edge->set_transform(tracker_.t_ba().inverse());
  }

  UpdateStats();
  aac_cond_.notify_all();
  Toc(sd::timer::kIterate);
  return true;
}

void SemiDenseFrontEnd::UpdateStats() {
  system_status_.num_tracked_landmarks = tracker_.num_successful_tracks();
  system_status_.num_new_landmarks = tracker_.GetNewTracks().size();
  tracking_stats_.analytics["Trkd-lmks"] =
      static_cast<double>(system_status_.num_tracked_landmarks);
  tracking_stats_.analytics["New-lmks"] =
      static_cast<double>(system_status_.num_new_landmarks);
}

bool SemiDenseFrontEnd::IterateBa() {
  DoSynchronousBundleAdjustment();
  return true;
}

void SemiDenseFrontEnd::AsyncBaFunc() {
  LOG(INFO) << "Starting async ba thread.";
  aac_optimization_.Init(map_);

  std::chrono::seconds wait_time(1);
  std::mutex async_mutex;
  std::unique_lock<std::mutex> lock(async_mutex);
  while (!is_quitting_) {
    if (!CommonFrontEndConfig::getConfig()->doAsyncBundleAdjustment()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    } else if (aac_cond_.wait_for(lock, wait_time) == std::cv_status::timeout) {
      continue;
    }

    rslam::optimization::AdaptiveOptions options(CommonFrontEndConfig::getConfig()->doAdaptiveWindow(),
                                     true,
                                     CommonFrontEndConfig::getConfig()->getBAWindowSize(), 100);
    // We don't want to include the current frame in our optimizations
    options.ignore_frame = current_frame_->id();
    aac_optimization_.RefineMap(CommonFrontEndConfig::getConfig()->getAsyncBAWindowSize(),
                           prev_frame_->id(),
                           CommonFrontEndConfig::getConfig()->getBANumIterAdaptive(),
                           has_imu_,
                           old_rig_,
                           options,
                           false,
                           EdgeAttrib_AsyncIsBeingOptimized,

                           /** @todo Use backend_callbacks_ here.
                            *
                            * First the tracker interactions must be
                            * made thread-safe so that updating
                            * current tracks can be done
                            * asynchronously.
                            */
                           {});
  }
  LOG(INFO) << "Quitting ba thread.";
}

void SemiDenseFrontEnd::system_status(common::SystemStatus *ss) const {
  CHECK_NOTNULL(ss);
  *ss = system_status_;
}

void SemiDenseFrontEnd::tracking_stats(common::TrackingStats *ts) const {
  CHECK_NOTNULL(ts);
  *ts = tracking_stats_;
}

void SemiDenseFrontEnd::Save(const std::string &filename) const {
}

void SemiDenseFrontEnd::Load(const std::string &filename) {
}

void SemiDenseFrontEnd::GetCurrentKeypointsForDisplay(
    std::vector<std::vector<cv::KeyPoint> > &keypoints) {

    keypoints.resize(2);
    for (const std::shared_ptr<sdtrack::DenseTrack>& track : tracker_.GetCurrentTracks()) {
      for (size_t cam_idx = 0; cam_idx < rig_.cameras_.size(); cam_idx++) {
        cv::KeyPoint kp((float)track->keypoints.back()[cam_idx].kp(0),(float)track->keypoints.back()[cam_idx].kp(1), 1.0);
        keypoints[cam_idx].push_back(kp);
      }
    }
}

void SemiDenseFrontEnd::set_server_proxy(
    const std::shared_ptr<SlamServerProxy> &proxy) {
}

SlamFramePtr SemiDenseFrontEnd::current_frame() const {
  return current_frame_;
}

bool SemiDenseFrontEnd::IsInitialized() const {
  return system_status_.is_initialized;
}

void SemiDenseFrontEnd::DoSynchronousBundleAdjustment() {
  ba::debug_level_threshold = google::log_severity_global;
  ba::debug_level = google::INFO + 1;

  static const rslam::optimization::AdaptiveOptions options(false, false, 15, 100);
  optimization_.RefineMap(CommonFrontEndConfig::getConfig()->getBAWindowSize(),
                     current_frame_->id(),
                     CommonFrontEndConfig::getConfig()->getBANumIter(),
                     false,
                     old_rig_,
                     options,
                     false,
                     EdgeAttrib_IsBeingOptimized,
                     optimization_callbacks_);
}

void SemiDenseFrontEnd::RegisterPoseMeasurement(
    const rslam::map::PoseMeasurement& pose) {
  if (current_frame_) {
    current_frame_->AddPoseMeasurement(pose);
  }
}
