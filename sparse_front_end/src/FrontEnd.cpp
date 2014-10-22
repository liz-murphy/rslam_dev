// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//  \file FrontEnd.cpp
//  This class contains the main slam engine program glue-logic.
#include <unistd.h>
#include <math.h>  // for log

#include <fstream>
#include <sstream>
#include <vector>
#include <utility>
#include <string>

#include <sparse_tracking/TrackingConfig.h>

#include <common_front_end/CommonFrontEndConfig.h>
#include <common_front_end/EssentialMatrix.h>
#include <common_front_end/PatchMatch.h>

#include <slam_map/MapVisitor/RelativePoseMapVisitor.h>
#include <slam_map/ProtobufIO.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/SlamMap.h>
#include <slam_map/TransformEdge.h>
#include <sparse_front_end/FrontEnd.h>
#include <sparse_front_end/FrontEndServerInterface.h>
#include <sparse_front_end/LiftLocalMap.h>
#include <sparse_front_end/PushLocalMap.h>
#include <sparse_front_end/ClosestKeyframeMapVisitor.h>
#include <sparse_tracking/EstimateRelativePose.h>
#include <sparse_tracking/StartNewLandmarks.h>
#include <utils/Utils.h>
#include <pb_msgs/frontend.pb.h>
#include <miniglog/logging.h>
#include <condition_variable>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace rslam {
namespace sparse {

using Eigen::Vector2t;
using Eigen::Vector4t;
using Eigen::Vector6t;

struct RelocalizationCandidate {
  ReferenceFrameId frame;
  Scalar score;
  Sophus::SE3t t_ab;
  std::vector<MultiViewMeasurement> new_measurements;
};

FrontEnd::FrontEnd() : is_simulation_(false),
                       has_imu_(false),
                       is_lost_(false),
                       is_quitting_localizer_(false),
                       is_quitting_ba_(false),
                       is_relocalizer_busy_(false),
                       is_async_busy_(false),
                       hold_frame_token_(0) {
}

FrontEnd::~FrontEnd() {
  // also kill the relocalizer thread
  if (ba_thread_ && ba_thread_->joinable()) {
    ROS_INFO("Quitting BA thread...");
    is_quitting_ba_ = true;
    async_ba_cond_.notify_all();
    ba_thread_->join();
    ROS_INFO("BA thread shut down.");
  }

  if (relocalizer_thread_ && relocalizer_thread_->joinable()) {
    ROS_INFO("Quitting relocalizer thread...");
    is_quitting_localizer_ = true;
    relocalizer_cond_.notify_all();
    relocalizer_thread_->join();
    ROS_INFO("Relocalizer thread shut down.");
  }

  ROS_INFO("All threads closed, shutting down.");
  Clear();
}

void FrontEnd::Tic(const std::string &name) {
  if (timer_) {
    timer_->Tic(name);
  }
}

void FrontEnd::Toc(const std::string &name) {
  if (timer_) {
    timer_->Toc(name);
  }
}

void FrontEnd::SetupFeatureHandlers() {
  // We need to update the dynamic reconfigure server
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::BoolParameter bool_param;
  dynamic_reconfigure::IntParameter int_param;
  dynamic_reconfigure::Config conf;

  if (CommonFrontEndConfig::getConfig()->getFeatureDetector() == common_front_end::CommonFrontEndParams_DOG) 
  {
    bool_param.name = "do_subpixel_refinement";
    bool_param.value = false;
    conf.bools.push_back(bool_param);
    CommonFrontEndConfig::getConfig()->setSubPixelRefinement(false);

    bool_param.name = "use_feature_buckets";
    bool_param.value = false;
    conf.bools.push_back(bool_param);
    CommonFrontEndConfig::getConfig()->setUseFeatureBuckets(false);
  
    int_param.name = "num_quadtree_levels";
    int_param.value = 1;
    conf.ints.push_back(int_param);
    CommonFrontEndConfig::getConfig()->setNumQuadTreeLevels(1);

  } 
  srv_req.config = conf;
  ros::service::call("/rslam_engine_node/set_parameters", srv_req, srv_resp);

 /* options_.dog_edge_threshold  = CommonFrontEndConfig::getConfig()->dog_edge_threshold;
  options_.dog_threshold       = CommonFrontEndConfig::getConfig()->dog_threshold;
  options_.dog_num_octaves     = CommonFrontEndConfig::getConfig()->dog_octaves;
  options_.dog_sigma           = CommonFrontEndConfig::getConfig()->dog_sigma;

  options_.freak_orientation_normalized = CommonFrontEndConfig::getConfig->freakOrientationNormalized();
  options_.freak_scale_normalized = CommonFrontEndConfig::getConfig->freakScaleNormalized();
  options_.freak_num_octaves   = CommonFrontEndConfig::getConfig()->getFreakOctaves();
  options_.freak_pattern_scale = CommonFrontEndConfig::getConfig()->getFreakPatternScale();


  options_.fast_threshold      = CommonFrontEndConfig::getConfig()->fast_threshold;
  options_.fast_levels         = CommonFrontEndConfig::getConfig()->fast_levels;
  options_.fast_level_factor   = CommonFrontEndConfig::getConfig()->fast_level_factor;
  options_.fast_skip_level0    = CommonFrontEndConfig::getConfig()->fast_skip_level0;

  options_.brisk_threshold             = CommonFrontEndConfig::getConfig()->brisk_threshold;
  options_.brisk_num_octaves           = CommonFrontEndConfig::getConfig()->brisk_octaves;
  options_.brisk_is_rotation_invariant = CommonFrontEndConfig::getConfig()->is_rotation_invariant;
  options_.brisk_do_3d_refinement      = false;
*/
  // initialize handlers (this takes a while, just done once)
  for (size_t ii = 0; ii < rig_.cameras.size(); ++ii) {
    feature_handler_[ii].Init();
  }
}

void FrontEnd::reset()
{
  ROS_ERROR("Front end reset NOT IMPLEMENTED!!!");
}

bool FrontEnd::Init(
    const calibu::CameraRigT<Scalar>    &rig,
    const std::shared_ptr<pb::ImageArray> &frames,
    const double                        timestamp,
    const bool                          has_imu,
    const std::shared_ptr<SlamMap>      &map,
    const std::shared_ptr<PlaceMatcher> &place_matcher,
    const std::shared_ptr<Timer>        &timer,
    bool                                using_sim_data) {
  is_simulation_ = using_sim_data;
  has_imu_ = has_imu;
  // Reset member variables.
  Clear();

  // Random Seed (for repeatability).
  system_state_.rng.seed(CommonFrontEndConfig::getConfig()->getRansacSeed());

  //=========================================================
  // Set map, place matcher, timer, vehicle configuration,
  // and front/back end optimizers.
  //=========================================================
  map_ = map;
  place_matcher_ = place_matcher;
  timer_ = timer;
  timer_->set_window_size(timer_window_size_);
  rig_ = rig;
  tracking_stats_.SetRigSize(rig_.cameras.size());

  hold_frame_token_ = map_->GetHoldToken();

  SetupFeatureHandlers();

  // This instance is used for loop closure/map relaxation.
  back_end_opt_.Init(map_);

  // This instance is used for odometry.
  front_end_opt.Init(map_);

  if (frames->Size() > 0) {
    unsigned int wide_fov_cam_id = calibu::LargestFrustum(rig_);
    dense_aligner_.Init(rig_.cameras[wide_fov_cam_id].camera, 32,
                        CommonFrontEndConfig::getConfig()->getDenseAlignThreshold());
    std::shared_ptr<pb::Image> ref_img = frames->at(wide_fov_cam_id);
    dense_aligner_.SetRefImage(*ref_img);
  }
  // load ground truth if we have it
  LoadGroundTruth();

  //=========================================================
  // Process first frame
  //=========================================================
  tracking_stats_.match_flags_histogram.resize(rig_.cameras.size());

  // This adds a node with no edges in the map.
  current_frame_ = map_->AddFrame(timestamp);
  initial_frame_ = current_frame_->id();

  // Set the current tvs (camera to vehicle transform) and
  // camera parameters on this node.
  if (has_imu) {
    CHECK(!front_end_opt.GetImuBuffer().elements.empty());
    current_frame_->set_g_r(
        // ((Eigen::Vector3t() <<
        // 0, -1, 0)).finished().normalized() * ba::Gravity);
        front_end_opt.GetImuBuffer().elements.front().a.normalized() *
        ba::Gravity);
    ROS_INFO("Setting starting frame gravity to %s", boost::lexical_cast<std::string>(current_frame_->g_r().transpose()).c_str() );

     ROS_INFO("Setting starting frame bias to %s", boost::lexical_cast<std::string>(current_frame_->b().transpose()).c_str() );
  }
  current_frame_->set_t_vs(rig_.cameras[0].T_wc);
  current_frame_->set_cam_params(rig_.cameras[0].camera.GenericParams());
  reference_frame_ = current_frame_;

  // Allocate the feature images.
  current_feature_images_.resize(rig_.cameras.size());
  keyframe_images_.resize(rig_.cameras.size());

  for (size_t ii = 0; ii < rig_.cameras.size(); ii++) {
    current_feature_images_[ii] = std::make_shared<FeatureImage>(
        CommonFrontEndConfig::getConfig()->getPyramidLevels(),
        CommonFrontEndConfig::getConfig()->getPyramidLevelFactor(),
        CommonFrontEndConfig::getConfig()->getNumQuadtreeLevels(),
        CommonFrontEndConfig::getConfig()->getNumFeaturesToTrack(),
        CommonFrontEndConfig::getConfig()->getMaxFeaturesInCell(),
        CommonFrontEndConfig::getConfig()->useFeatureBuckets());
    keyframe_images_[ii] = std::make_shared<FeatureImage>(
        CommonFrontEndConfig::getConfig()->getPyramidLevels(),
        CommonFrontEndConfig::getConfig()->getPyramidLevelFactor(),
        CommonFrontEndConfig::getConfig()->getNumQuadtreeLevels(),
        CommonFrontEndConfig::getConfig()->getNumFeaturesToTrack(),
        CommonFrontEndConfig::getConfig()->getMaxFeaturesInCell(),
        CommonFrontEndConfig::getConfig()->useFeatureBuckets());
  }

  // Add first frame to the place matcher object.
  ExtractFeatures(frames, current_feature_images_);

  if (!frames->Empty()) {
    std::shared_ptr<pb::Image> place = frames->at(0);
    AddPlace(current_frame_->id(), *place);
  }

  // Initialize and allocate space for the feature mask.
  for (size_t ii = 0; ii < rig_.cameras.size(); ++ii) {
    feature_mask_.AddImage(frames->at(ii)->Width(),
                           frames->at(ii)->Height());
    // feature_mask_.push_back(new unsigned char[frames->at(ii)->Width() *
    //    frames->at(ii)->Height()]);
  }

  feature_mask_.Clear();
  // Function in StartNewLandmarks.h
  StartNewLandmarks(rig_, map_, current_frame_, current_feature_images_,
                    system_state_.num_tracked_landmarks,
                    system_state_.num_new_landmarks,
                    feature_mask_);

  // Save data for display.
  tracking_stats_.analytics["Trkd-lmks"] =
      static_cast<double>(system_state_.num_tracked_landmarks);
  tracking_stats_.analytics["New-lmks"]  =
      static_cast<double>(system_state_.num_new_landmarks);

  //=========================================================
  // Set front end initial state.
  //=========================================================
  system_state_.frame_number = 0;
  system_state_.keyframe_number = 0;
  system_state_.time = timestamp;
  system_state_.inlier_noise_error = inlier_threshold_;

  learning_rate_ = 0.9;  // slow
  system_state_.is_initialized =  false;

  system_state_.drop_new_frame = true;

  is_lost_ = false;
  t_ab_ = Sophus::SE3t();

  //=========================================================
  // Start relocalization/loop closure thread.
  //=========================================================
  relocalization_frame_id_ = current_frame_->id();
  is_relocalizer_busy_ = false;
  relocalizer_thread_ = std::make_shared<std::thread>(
      std::bind(&FrontEnd::RelocalizerFunc, this));

  ba_thread_ = std::make_shared<std::thread>(
      std::bind(&FrontEnd::AsyncBaFunc,this));

  return true;
}

bool FrontEnd::IsInitialized() const {
  return system_state_.is_initialized;
}

bool FrontEnd::SwitchToKeyframe(const ReferenceFrameId& closest_keyframe_id,
                                const Sophus::SE3t& t_ab) {
  //=========================================================
  // If the closest keyframe is different from our current
  // reference keyframe, switch the reference pointer and
  // update the edge connecting the current position to the map.
  //=========================================================

  if (closest_keyframe_id != reference_frame_->id()) {
    ROS_DEBUG_NAMED("sparse_front_end::Iterate","Iterate: Switching keyframe from id=%d to %d", reference_frame_->id().id ,closest_keyframe_id.id);

    SlamFramePtr pNewKeyframe = map_->GetFramePtr(closest_keyframe_id);
    SlamEdgePtr pEdge = map_->GetEdgePtr(reference_frame_->id(),
                                         current_frame_->id());
    // swaps out the start and sets the new transform
    map_->SwapEdgeStartFrame(pEdge->id(), reference_frame_->id(),
                             closest_keyframe_id, t_ab);

    reference_frame_ = pNewKeyframe;
    return true;
  } else {
    return false;
  }
}

bool FrontEnd::IterateBa() {
  // this modifies m_Rig if calibration is active
  // also modifies the map
  // BackEnd::AdaptiveOptions options(false, 0, 0);
  BackEnd::AdaptiveOptions options(false, false, 15, 100);
  uint32_t depth = ba_window_size_;
  uint32_t num_iterations = ba_num_iter_;

  Eigen::Vector3t initial_gravity, initial_velocity;
  Eigen::Vector6t initial_bias;
  if (!system_state_.is_imu_converged) {
    depth = system_state_.keyframe_number;
    num_iterations = 4;

    SlamFramePtr initial = map_->GetFramePtr(initial_frame_);
    CHECK(initial);  // Something's wrong if we don't have our initial pose
    initial_gravity  = initial->g_r();
    initial_bias     = initial->b();
    initial_velocity = initial->v_r();
  }

  front_end_opt.RefineMap(depth,
                          current_frame_->id(),
                          num_iterations,
                          has_imu_,
                          rig_,
                          options,
                          true,
                          EdgeAttrib_IsBeingOptimized,
                          BackEnd::RefineMapCallbacks());

  // Get ready for the heuristics!
  if (!system_state_.is_imu_converged && system_state_.keyframe_number > 20) {
    SlamFramePtr initial = map_->GetFramePtr(initial_frame_);
    CHECK(initial);  // Something's wrong if we don't have our initial pose

    Scalar gravity_diff  = (initial->g_r() - initial_gravity).norm();
    Scalar velocity_diff = (initial->v_r() - initial_velocity).norm();
    Scalar bias_diff     = (initial->b() - initial_bias).norm();
    if (gravity_diff < 0.1 && bias_diff < 0.1 && velocity_diff < 0.1) {
      system_state_.is_imu_converged = true;
    }
  }
  return true;
}

void FrontEnd::ProcessKeyframe(
    const cv::Mat& query_frame,
    std::vector<MultiViewMeasurement>* new_measurements,
    std::vector<std::vector<Feature*> >* feature_matches) {

  ROS_DEBUG_NAMED("sparse_front_end::Iterate","Iterate: Frame id=%d is a new keyframe", current_frame_->id().id);

  //=========================================================
  // If we are not lost, include measurements
  //=========================================================
  if (system_state_.tracking_status == common::eTrackingGood) {
    map_->AddNewMeasurementsToFrame(current_frame_->id(), *new_measurements);
  }

  //=========================================================
  // If using 2d tracked features,flag features that failed tracking as
  // not used so that start new landmarks can use them.
  //=========================================================
  // ALWAYS using 2d tracked features now ...
  ProcessFailedTracks(*new_measurements, feature_matches);
  
  //=========================================================
  // Start new landmarks if needed
  //=========================================================
  Tic("StartNewLandmarks");
  StartNewLandmarks(rig_, map_, current_frame_, current_feature_images_,
                    system_state_.num_tracked_landmarks,
                    system_state_.num_new_landmarks,
                    feature_mask_);

  // save data for display
  tracking_stats_.analytics["Trkd-lmks"] =
      static_cast<double>(system_state_.num_tracked_landmarks);
  tracking_stats_.analytics["New-lmks"]  =
      static_cast<double>(system_state_.num_new_landmarks);
  Toc("StartNewLandmarks");


  //=========================================================
  // Do slidding window bundle adjustment
  //=========================================================
  if (system_state_.tracking_status == common::eTrackingGood &&
      do_bundle_adjustment_) {
    Tic("BA");
    IterateBa();
    Toc("BA");
  }

  //=========================================================
  // Mark that we just created a new keyframe
  //=========================================================
  system_state_.drop_new_frame = true;
  t_ab_ = Sophus::SE3t();

  // only update keyframe feature images and  query thumbnail
  // if the relocalizer is not busy
  if (!is_relocalizer_busy_ && do_relocalization_) {
    for (size_t ii = 0; ii < current_feature_images_.size(); ii++) {
      keyframe_images_[ii]->Copy(*current_feature_images_[ii]);
      keyframe_images_[ii]->ResetFeatures();
    }
    relocalization_frame_id_ = current_frame_->id();
    query_vector_ = query_frame.clone();

    // notify the relocalizer thread to start
    is_relocalizer_busy_ = true;
    relocalizer_cond_.notify_one();
  }
}

bool FrontEnd::Iterate(const std::shared_ptr<pb::ImageArray>& frames,
                       double timestamp) {
  // Start timer for the main cycle.
  Tic("FrontEnd");

  //=========================================================
  // Update state of the front end.
  //=========================================================
  system_state_.frame_number++;
  system_state_.time = timestamp;

  //=========================================================
  // Map initialization.
  //
  // If we succeed in initializing, process this frame as normal.
  //=========================================================
  if (!system_state_.is_initialized) {
    system_state_.is_initialized = Initialization(frames, timestamp);
    system_state_.tracking_status = common::eTrackingGood;
    Toc("FrontEnd");
    return true;
  }

  //=========================================================
  // Check if our previous position has become a keyframe.
  // When this happens we create a new frame in the map that
  // represents our current position. There is no edge between
  // this frame and the map. An adge is added after attempting
  // to localize at least once.
  //=========================================================
  Sophus::SE3t t_ab;
  if (system_state_.drop_new_frame) {
    reference_frame_ = current_frame_;

    previous_id_ = current_frame_->id();
    current_frame_  = map_->AddFrame(system_state_.time);
    ++system_state_.keyframe_number;

    ROS_DEBUG_NAMED("sparse_front_end::Iterate","Iterate: Adding new frame with id = %d",current_frame_->id().id);

    server_interface_.AsyncUploadMapToServer(
        map_, place_matcher_, reference_frame_->id());
  } else {
    //=========================================================
    // We didn't add a new keyframe. Update current frame.
    // Check if there is a closer keyframe and use that one
    // as a reference for localizing.
    //=========================================================

    current_frame_->set_time(system_state_.time);
    map_->RemoveMeasurementsFromFrame(current_frame_->id());

    ROS_DEBUG_NAMED("sparse_front_end::Iterate","Iterate: Checking for closest keyframe");

    //=========================================================
    // this call finds our closest keyframe, as well as updating
    // the "guess" relative transform (between the keyframe and
    // our current position) using m_Tab.
    // This is called again after EstimateRelativePose in case
    // the closest keyframe changes
    //=========================================================

    /** @todo chachi Make this work with the IMU, too */
    if (!has_imu_) {
      ReferenceFrameId closest_keyframe_id =
          GetClosestKeyframeId(current_frame_->id(),
                               keyframe_search_depth_,
                               t_ab_);
      SwitchToKeyframe(closest_keyframe_id, t_ab_);
    }
    t_ab = t_ab_;
  }

  //=========================================================
  // At this point we will attempt to localize w.r.t the
  // selected reference frame. First we extract new image
  // features from the captured frames.
  //=========================================================

  Tic("ProcessImages");
  ExtractFeatures(frames, current_feature_images_);
  Toc("ProcessImages");

  //=========================================================
  // Then a working set of previously observed landmarks
  // (3d points) is selected and their positions set w.r.t
  // the coordinate frame of the reference frame.
  //=========================================================

  Tic("ComputeWorkingSet");
  if (CommonFrontEndConfig::getConfig()->doMarginalization()) {
    // If we are doing marginalization, we wish to match everything that was
    // seen in the current optimization window, but nothing more, which is why
    // the local_only flag is set on the lift operation, and the lift window
    // size is set to the ba window size.
    LiftLocalMap(map_.get(), ba_window_size_,
                 reference_frame_->id(), work_set_,
                 has_imu_, false, true);
  } else {
    LiftLocalMap(map_.get(),
                 do_keyframing_ ?
                 1u : TrackingConfig::getConfig()->matchintime_window_size,
                 reference_frame_->id(), work_set_,
                 has_imu_, false, false);
  }
  Toc("ComputeWorkingSet");

  //=========================================================
  // Matches between 3d landmarks in the working set and
  // 2d image feature points in the current image are computed
  // and used to estimate our relative position w.r.t the
  // reference frame.
  //=========================================================

  // Do dense alignment.
  if (do_dense_init_) {
    Tic("DenseAlignment");
    Sophus::SE3t hint_t_ab;
    unsigned int cam_id = calibu::LargestFrustum(rig_);
    const cv::Mat& current_image = current_feature_images_[cam_id]->ImageRef();
    dense_aligner_.Align(current_image, hint_t_ab);
    ROS_INFO("Motion hint: %s", Pose2Str(hint_t_ab.matrix()));

    t_ab = t_ab * hint_t_ab;
    Toc("DenseAlignment");
  }

  // If we have an imu, predict where the next frame will be based on the
  // imu measurements.
  // [NOTE]: This could be a problem if the reference frame is an old
  // keyframe and we can't integrate between it and the current frame.
  std::vector<ba::ImuMeasurementT<Scalar> > meas;
  Sophus::SE3t t_ab_imu;
  bool use_imu_for_estimate = false;

  // Only use the IMU for estimation if we have an IMU and we have elected
  // to use it for the frame to frame estimation.
  if (has_imu_ && use_imu_for_gn_) {
    meas = front_end_opt.GetImuBuffer().GetRange(
        reference_frame_->time(), timestamp);
    use_imu_for_estimate = !meas.empty();

    Eigen::Vector3t g_r_new, v_r_new;
    front_end_opt.IntegrateLastPose(reference_frame_->time(),
                                    timestamp,
                                    reference_frame_->v_r(),
                                    reference_frame_->g_r(),
                                    reference_frame_->b(),
                                    t_ab_imu,
                                    g_r_new,
                                    v_r_new,
                                    meas);

    // @todo Prune doesn't exist anymore?
    // if (meas.size() > 100) {
    //   front_end_opt.GetImuBuffer().Prune(reference_frame_->time(),
    //                                      meas.size() - 100);
    //   meas = front_end_opt.GetImuBuffer().GetRange(
    //       reference_frame_->time(), timestamp);
    // }
  }

  std::vector<MultiViewMeasurement> new_measurements;
  std::vector< std::vector<Feature*> > feature_matches;
  Tic("PoseEstimation");

  // Clear the feature mask before we do any matching
  feature_mask_.Clear();

  ROS_ERROR_COND(!map_->GetCamera(current_frame_->id().session_id), "No camera rig found for given frame");

  bool failed = !EstimateRelativePose(current_frame_->id(),
                                      work_set_,
                                      front_end_opt,
                                      current_feature_images_,
                                      new_measurements,
                                      feature_matches,
                                      false,
                                      t_ab,
                                      learning_rate_,
                                      system_state_,
                                      timer_,
                                      meas,
                                      use_imu_for_estimate);
  // [ TODO ? ] if ref_frame is not the previous one (in time)
  // we might have switched to a bad keyframe. We could attempt
  // to relocalize wrt the previous one.

  // Update the feature mask for every matched feature
  for (std::vector<Feature*>& lm_features : feature_matches) {
    for (size_t ii = 0 ; ii < lm_features.size() ; ++ii){
      if (lm_features[ii] != nullptr) {
        feature_mask_.SetMask(ii, lm_features[ii]->x, lm_features[ii]->y);
      }
    }
  }

  // Update the tracking stats
  tracking_stats_.analytics["MRE"] = system_state_.rmsre;
  tracking_stats_.analytics["InlierNoise"] = system_state_.inlier_noise_error;

  if (failed && use_imu_for_estimate) {
    t_ab = t_ab_imu;
    system_state_.tracking_status = common::eTrackingBad;
  } else if (failed) {
    if (current_frame_->is_isolated()) {
      t_ab = last_motion_;
    } else {
      t_ab = last_motion_ * last_motion_;
    }

    // system_state_.is_initialized = false;
    system_state_.tracking_status = common::eTrackingBad;
  } else {
    system_state_.tracking_status = common::eTrackingGood;
  }
  Toc("PoseEstimation");

  //=========================================================
  // Add edge with estimated pose or update edge if it exists.
  //=========================================================
  if (current_frame_->is_isolated()) {
    ROS_DEBUG_NAMED("sparse_front_end::Iterate","Iterate: Adding a new edge between %d and %d ", reference_frame_->id().id, current_frame_->id().id);

    if (previous_id_ == reference_frame_->id()) {
      last_motion_ = t_ab;
    } else {
      RelativePoseMapVisitor visitor(reference_frame_->id());
      visitor.set_root_id(previous_id_);
      visitor.set_depth(10);  // Limit the search
      map_->InMemoryBFS(&visitor);
      if (visitor.relative_pose_found()) {
        visitor.relative_pose(&last_motion_);
      } else {
        last_motion_ = Sophus::SE3t();
      }
    }

    map_->AddEdge(reference_frame_, current_frame_, t_ab);
   
    map_->SetHoldFrames(hold_frame_token_,
                        reference_frame_->id(),
                        CommonFrontEndConfig::getConfig()->getTrackerHoldDepth(),
                        true);
  } else {
    ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Iterate: Updating edge between %d and %d", reference_frame_->id().id, current_frame_->id().id);

    Sophus::SE3t old_t_ab;
    if (SlamEdgePtr edge = map_->GetEdgePtr(reference_frame_->id(),
                                            current_frame_->id())) {
      edge->transform(reference_frame_->id(), current_frame_->id(),
                         old_t_ab);
      last_motion_ = old_t_ab.inverse() * t_ab;
      edge->set_transform(t_ab);
    }
    map_->SetHoldFrames(hold_frame_token_,
                        reference_frame_->id(),
                        CommonFrontEndConfig::getConfig()->getTrackerHoldDepth(),
                        true);
  }

  // If we have elected not to use the IMU in the frame to frame estimation,
  // initialize the V and G values based on the previous frame.
  if (!use_imu_for_gn_) {
    const auto& rot_ba = t_ab.so3().inverse();
    current_frame_->set_g_r(rot_ba * reference_frame_->g_r());
    current_frame_->set_v_r(rot_ba * reference_frame_->v_r());
  }

  //=========================================================
  // Mark edge as broken if we are lost.
  //=========================================================
  if (SlamEdgePtr edge = map_->GetEdgePtr(reference_frame_->id(),
                                          current_frame_->id())) {
    bool is_broken = (system_state_.tracking_status == common::eTrackingBad &&
                      !has_imu_);
    edge->set_is_broken(is_broken);
    if (is_broken) {
      map_->SetEdgeAttribute(edge->id(), EdgeAttrib_Broken);
    }
  }

  //=========================================================
  // Check if the current frame is a keyframe (this checks
  // three thresholds: percentage of landmarks tracked,
  // relative distance travelled and relative angle difference
  // w.r.t reference frame). Also add keyframe if we are lost.
  // If we are not using keyframes the function _IsKeyframe
  // will always return true.
  //=========================================================

  std::shared_ptr<pb::Image> query_frame = frames->at(0);

  if (IsKeyframe(t_ab, new_measurements) ||
      system_state_.tracking_status == common::eTrackingBad) {
    ProcessKeyframe(*query_frame, &new_measurements, &feature_matches);
    // Collect pose analytics if requested.
    Sophus::SE3t pose;
    if (collect_pose_analytics_) {
      pose_analytics_.emplace_back(current_frame_->id(),
                                   async_ba_window_size_,
                                   pose,
                                   timestamp);
    }
  } else {
    map_->AddNewMeasurementsToFrame(current_frame_->id(), new_measurements);
    system_state_.drop_new_frame = false;
    t_ab_ = t_ab;
    // Tic-Toc functions that where not used
    Tic("StartNewLandmarks"); Toc("StartNewLandmarks");
    Tic("BA"); Toc("BA");
  }

  // Signal to the async_ba that it's time to run
  if (!is_async_busy_) {
    async_frame_id_ = reference_frame_->id();
    async_ba_cond_.notify_one();
  }

  tracking_stats_.UpdateMeasurementHistograms(new_measurements);

  float fAvgTrackLen = 0;
  Landmark lm;
  for (auto& pair : work_set_.landmarks) {
    if (!map_->GetLandmark(pair.second->id, &lm)) continue;
    fAvgTrackLen += lm.FeatureTrackLength() - lm.num_failed_track_attempts();
  }
  tracking_stats_.analytics["AvgTrkLen"] =
      fAvgTrackLen / work_set_.landmarks.size();

  Toc("FrontEnd");
  return true;
}

/*
 * Using initialization procedure similar to:
 * "User Friendly SLAM Initialization".
 * A. Mullon1, M. Ramachandran, G. Reitmayr, D. Wagner, R. Grasset, S. Diaz
 * ISMAR 2013
 */
bool FrontEnd::Initialization(const std::shared_ptr<pb::ImageArray>& frames,
                              double timestamp) {
  if (rig_.cameras.size() > 1) return true;

  Sophus::SE3t t_ab;

  // check if the last frame is a keyframe
  if (system_state_.drop_new_frame) {
    reference_frame_ = current_frame_;
    current_frame_  = map_->AddFrame(system_state_.time);
    ++system_state_.keyframe_number;
    map_->AddEdge(reference_frame_, current_frame_, t_ab);
    if( (current_frame_->id().id - reference_frame_->id().id) > 1)
      ROS_ERROR("Adding edge between %d and %d", reference_frame_->id().id, current_frame_->id().id);
   

  } else {
    // In case the current frame is isolated, add an edge so that the pose is
    // correctly lifted.
    if (current_frame_->is_isolated()) {
      map_->AddEdge(reference_frame_, current_frame_, t_ab);
      
      if( (current_frame_->id().id - reference_frame_->id().id) > 1)
        ROS_ERROR("Adding edge between %d and %d", reference_frame_->id().id, current_frame_->id().id);
   

    }
    current_frame_->set_time(system_state_.time);
    map_->RemoveMeasurementsFromFrame(current_frame_->id());
    t_ab = t_ab_;
  }

  // Extract features.
  Tic("ProcessImages");
  ExtractFeatures(frames, current_feature_images_);
  Toc("ProcessImages");

  // Get features to track from the reference frame
  Tic("ComputeWorkingSet");
  LiftLocalMap(map_.get(), 1u, reference_frame_->id(), work_set_,
               has_imu_, false, false);
  Toc("ComputeWorkingSet");

  ROS_ERROR_COND(work_set_.poses.find(current_frame_->id()) == work_set_.poses.end(),"Current frame did not lift during initialization. Check if isolated.");

  // Data association.
  Tic("PoseEstimation");
  std::vector<MultiViewMeasurement> new_measurements;
  std::vector< std::vector<Feature*> > feature_matches;

  Tic("MatchInTime");
  CameraRigPtr rig = map_->GetCamera(current_frame_->id().session_id);
  LiftTrackingData(rig, t_ab, *map_, work_set_);
  MatchInTime(current_frame_->id(),
              current_feature_images_,
              work_set_,
              new_measurements,
              feature_matches,
              CommonFrontEndConfig::getConfig()->getInitialSearchRadius());
  Toc("MatchInTime");
  ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Made %d new measurements", (int)new_measurements.size());

  Tic("RANSAC"); Toc("RANSAC");
  Tic("GaussNewton");

  // get 2d matches
  std::vector<Eigen::Vector2t> src_pts;
  std::vector<Eigen::Vector2t> dst_pts;
  MultiViewMeasurement z_src;
  Scalar mean_disparity = 0.0;
  Landmark lm;
  for (const MultiViewMeasurement& z : new_measurements) {
    if (z.HasGoodMeasurementInCam(0)) {
      // Get previous point
      if (!map_->GetLandmark(z.id().landmark_id, &lm)) continue;
      const MeasurementId& zid = lm.GetFeatureTrackRef().front();
      map_->GetMeasurement(zid, z_src);
      src_pts.push_back(z_src.Pixel(0));
      dst_pts.push_back(z.Pixel(0));
      mean_disparity += (src_pts.back() - dst_pts.back()).norm();
    }
  }
  mean_disparity /= src_pts.size();

  ba::Options<Scalar> options;
  ba::BundleAdjuster<Scalar, 1, 6, 0> ba_init;
  unsigned int ref_pose_id;
  unsigned int cur_pose_id;
  Scalar proj_error, unary_error, binary_error, inertial_error;

  unsigned int n = rig_.cameras[0].camera.NumParams();
  Scalar* params = rig_.cameras[0].camera.data();
  Scalar w = 0.0;
  if (n == 5) {
    w = std::fabs(params[4]);
  }

  ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Mean disp: %f distortion %f", mean_disparity, w);

  if (mean_disparity > init_min_disparity_ &&
      // Need 8 points to do fundamental matrix extraction
      src_pts.size() >= 8 &&
      w <= init_max_distortion_) {

    // Estimate pose from 2d matches.
    ROS_DEBUG_NAMED("sparse_front_end::Iterate", "POSE 2D-C ----------- computing pose...");

    ComputeCameraPoseFromEssentialMatrix(src_pts,
                                         dst_pts,
                                         rig_.cameras[0].camera.K(),
                                         &system_state_.rng,
                                         t_ab);

    ROS_DEBUG_NAMED("sparse_front_end::Iterate","Init pose from 2d corresp tab: %s", boost::lexical_cast<std::string>(t_ab.matrix()).c_str());
  } else {
    // Use BA to estimate only t_ab from 3D-2D matches
    ROS_DEBUG_NAMED("sparse_front_end::Iterate","POSE ONLY BA");
    
    ba_init.Init(options, 1, new_measurements.size(), new_measurements.size(),
                 rig_.cameras[0].T_wc);
    for (unsigned int ii = 0; ii < rig_.cameras.size(); ++ii) {
      ba_init.AddCamera(rig_.cameras[ii].camera, rig_.cameras[ii].T_wc);
    }
    ref_pose_id = ba_init.AddPose(Sophus::SE3t(), false);
    cur_pose_id = ba_init.AddPose(t_ab, true);
    // add residual blocks
    for (const MultiViewMeasurement & z : new_measurements) {
      if (z.HasGoodMeasurementInCam(0)) {
        const Eigen::Vector4t& x_w =
            work_set_.landmarks.at(z.id().landmark_id)->x_w;
        unsigned int landmark_id =
            ba_init.AddLandmark(x_w, ref_pose_id, 0, false);
        ba_init.AddProjectionResidual(z.Pixel(0), cur_pose_id,
                                      landmark_id, 0);
      }
    }

    ba_init.Solve(TrackingConfig::getConfig()->gn_max_num_iter);
    t_ab = ba_init.GetPose(cur_pose_id).t_wp;
    ba_init.GetErrors(proj_error, unary_error, binary_error, inertial_error);

    ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Init pose-only tab: %s error:%f\n", Pose2Str(t_ab.matrix()), proj_error);
  }

  ROS_DEBUG_NAMED("sparse_front_end::Iterate","POSE & LMKS BA -------------------");
  
  // Use BA to estimate t_ab and inverse depth
  Eigen::Vector3t c1(0, 0, 0);
  Eigen::Vector3t c2 = t_ab.translation();
  ba_init.Init(options, 1, new_measurements.size(), new_measurements.size(),
               rig_.cameras[0].T_wc);
  for (unsigned int ii = 0; ii < rig_.cameras.size(); ++ii) {
    ba_init.AddCamera(rig_.cameras[ii].camera, rig_.cameras[ii].T_wc);
  }
  ref_pose_id = ba_init.AddPose(Sophus::SE3t(), false);
  cur_pose_id = ba_init.AddPose(t_ab, true);
  // add residual blocks
  unsigned int num_active_landmarks = 0;
  unsigned int num_pasive_landmarks = 0;
  for (const MultiViewMeasurement & z : new_measurements) {
    std::shared_ptr<LandmarkContainer> landmark_container =
        work_set_.landmarks.at(z.id().landmark_id);

    if (z.HasGoodMeasurementInCam(0)) {
      const Eigen::Vector4t& x_w = landmark_container->x_w;

      Scalar angle = ComputeAngle(c1, c2, x_w.head<3>());
      bool optimize_landmark = false;
      if (angle > 5.0) {
        optimize_landmark = true;
        num_active_landmarks++;
      } else {
        num_pasive_landmarks++;
      }

      landmark_container->ba_id =
          ba_init.AddLandmark(x_w, ref_pose_id, 0, optimize_landmark);
      ba_init.AddProjectionResidual(z.Pixel(0), cur_pose_id,
                                    landmark_container->ba_id, 0);
    } else {
      landmark_container->ba_id = UINT_MAX;
    }
  }

  unsigned int total_landmarks = num_active_landmarks + num_pasive_landmarks;
  ROS_DEBUG_NAMED("sparse_front_end::Iterate","Optimizing  %d/%d", num_active_landmarks, total_landmarks); 
  ba_init.Solve(TrackingConfig::getConfig()->gn_max_num_iter);
  
  // Get estimated pose
  t_ab = ba_init.GetPose(cur_pose_id).t_wp;

  work_set_.poses[current_frame_->id()].t_wp = t_ab;

  // Get updated landmarks
  for (auto& pair : work_set_.landmarks) {
    std::shared_ptr<LandmarkContainer> lmc = pair.second;
    if (lmc->ba_id != UINT_MAX) {
      if (ba_init.IsLandmarkReliable(lmc->ba_id)) {
        lmc->x_w = ba_init.GetLandmark(lmc->ba_id);
        lmc->x_w /= lmc->x_w[3];
      }
    }
  }
  work_set_.PushGlobalCoordsToRelative();

  ba_init.GetErrors(proj_error, unary_error, binary_error, inertial_error);

  ROS_DEBUG_NAMED("sparse_front_end::Iterate","Init pose-and-lmks tab: %s error:%f\n", Pose2Str(t_ab.matrix()), proj_error);

  Toc("GaussNewton");
  Toc("PoseEstimation");


  t_ab_ = t_ab;
  map_->AddNewMeasurementsToFrame(current_frame_->id(), new_measurements);

  if (current_frame_->is_isolated()) {
    map_->AddEdge(reference_frame_, current_frame_, t_ab);
    
    if( (current_frame_->id().id - reference_frame_->id().id) > 1)
      ROS_ERROR("Isolated frame: Adding edge between %d and %d", reference_frame_->id().id, current_frame_->id().id);

    map_->SetHoldFrames(hold_frame_token_,
                        reference_frame_->id(),
                        CommonFrontEndConfig::getConfig()->getTrackerHoldDepth(),
                        true);
  } else {
    if (SlamEdgePtr edge = map_->GetEdgePtr(reference_frame_->id(),
                                            current_frame_->id())) {
      edge->set_transform(t_ab);
    }
    map_->SetHoldFrames(hold_frame_token_,
                        reference_frame_->id(),
                        CommonFrontEndConfig::getConfig()->getTrackerHoldDepth(),
                        true);
  }

  // Push changes to the map
  PushLocalMap(map_.get(), work_set_);
  // Check if we have a keyframe

  ROS_DEBUG_NAMED("sparse_front_end::Iterate","Active landmarks: %d \tTotal landmarks: %d", num_active_landmarks, total_landmarks);
  
  float landmark_pct =
      (static_cast<float>(num_active_landmarks) / total_landmarks);
  if (landmark_pct > init_min_pctg_init_landmarks_) {
    // If it is a keyframe, start new feature tracks if needed
    Tic("StartNewLandmarks");
    feature_mask_.Clear();
    StartNewLandmarks(rig_, map_, current_frame_, current_feature_images_,
                      system_state_.num_tracked_landmarks,
                      system_state_.num_new_landmarks, feature_mask_);
    Toc("StartNewLandmarks");
    Tic("BA");
    IterateBa();
    Toc("BA");
    system_state_.drop_new_frame = true;
    t_ab_ = Sophus::SE3t();
    if (system_state_.frame_number < init_min_keyframes_) {
      ROS_DEBUG_NAMED("sparse_front_end::Initialization", "Not enough keyframes to initialize.");
      return false;
    } else {
      ROS_DEBUG_NAMED("sparse_front_end::Initialization", "Initialization successful.");
      return true;
    }
  } else {
    Tic("StartNewLandmarks");  Toc("StartNewLandmarks");
    Tic("BA"); Toc("BA");
    system_state_.drop_new_frame = false;
    ROS_DEBUG_NAMED("sparse_front_end::Initialization","Only %f landmarks tracked.", landmark_pct);
    return false;
  }
}

void FrontEnd::Clear() {
  system_state_.Clear();
  tracking_stats_.analytics.clear();
}

bool FrontEnd::IsKeyframe(
    const Sophus::SE3t& t_ab,
    const std::vector<MultiViewMeasurement>& measurements) {
  if (!do_keyframing_) {
    return true;
  }
  // check percentage of lanmarks tracked
  int num_landmarks = 0;
  int num_tracked   = 0;

  for (const MultiViewMeasurement& z : measurements) {
    if (z.HasGoodMeasurement()) {
      num_tracked++;
    }
    num_landmarks++;
  }

  Scalar dPctgTracked  = static_cast<Scalar>(num_tracked) / num_landmarks;
  Scalar dDistTraveled = t_ab.translation().norm();
  Scalar dAngleDifference = t_ab.so3().log().maxCoeff() * 180/M_PI;

  bool add_keyframe = false;
  if (num_tracked < 2 * TrackingConfig::getConfig()->min_tracked_features) {
    ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Adding new keyframe because only %d features were tracked.", num_tracked); 
    add_keyframe = true;
  }

  if (dPctgTracked  < keyframe_threshold_) {
    ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Adding new keyframe since only %.2f (>%.2f) percent of landmarks were tracked [%d/%d].",
                 dPctgTracked, keyframe_threshold_, num_tracked, num_landmarks);
    add_keyframe = true;
  }

  if (dDistTraveled > keyframe_max_distance_) {
    ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Adding new keyframe since %.2f meters (>%.2f) were travelled since the last keyframe.",
                 dDistTraveled, keyframe_max_distance_);
    add_keyframe = true;
  }

  if (dAngleDifference > keyframe_max_angle_) {
    ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Adding new keyframe since %.2f degrees (>%.2f) were travelled since the last keyframe.",
                 dAngleDifference, keyframe_max_angle_);
    add_keyframe = true;
  }

  return add_keyframe;
}

void FrontEnd::AsyncBaFunc() {
  ROS_INFO("Starting async ba thread.");
  async_ba_.Init(map_);

  std::chrono::seconds wait_time(1);
  std::mutex async_mutex;
  std::unique_lock<std::mutex> lock(async_mutex);
  while (!is_quitting_ba_) {
    is_async_busy_ = false;
    if (!do_async_bundle_adjustment_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    } else if (async_ba_cond_.wait_for(lock, wait_time) ==
               std::cv_status::timeout) {
      continue;
    }

    is_async_busy_ = true;
    BackEnd::AdaptiveOptions options(do_adaptive_window_,
                                     true,
                                     ba_window_size_, 100);
    // We don't want to include the current frame in our optimizations
    options.ignore_frame = current_frame_->id();
    async_ba_.RefineMap(async_ba_window_size_,
                        async_frame_id_,
                        ba_num_iter_adaptive_,
                        has_imu_,
                        rig_,
                        options,
                        false,
                        EdgeAttrib_AsyncIsBeingOptimized,
                        BackEnd::RefineMapCallbacks());
  }
  is_async_busy_ = false;
  ROS_INFO("Quitting ba thread.");
}

void FrontEnd::RelocalizerFunc() {
  const std::chrono::milliseconds to_wait(30);
  std::unique_lock<std::mutex> lock(relocalizer_mutex_);

  ROS_DEBUG_NAMED("sparse_front_end::Relocalizer", "Starting relocalizer thread");
  LocalMap work_set;
  ReferenceFrameId query_frame_id(0, map_->id());
  last_successful_relocalization_frame_id = query_frame_id;

  common::TrackingStats reloc_tracking_stats = tracking_stats_;
  uint64_t reloc_hold_token = map_->GetHoldToken();
  while (!is_quitting_localizer_) {
    if (!do_relocalization_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // wait for a new keyframe
    while (is_quitting_localizer_ == false &&
           (query_frame_id == relocalization_frame_id_ ||
            is_relocalizer_busy_ == false)) {
      map_->RemoveFrameHold(reloc_hold_token);
      relocalizer_cond_.wait_for(lock, to_wait);
    }

    if (is_quitting_localizer_) break;

    query_frame_id = relocalization_frame_id_;
    SlamFramePtr current_frame = map_->GetFramePtr(query_frame_id);
    if (!current_frame) continue;

    std::vector<PlaceMatchCandidateT> place_matches;
    place_matcher_->GetPotentialPlaceMatchesOrAddPlace(query_frame_id,
                                                       query_vector_,
                                                       place_matches);

    if (place_matches.empty()) {
      std::cout << "No place matches ...\n";
      server_interface_.AsyncQueryServerWithPlace(
          map_, place_matcher_, query_frame_id, query_vector_);
      is_relocalizer_busy_ = false;
      continue;
    }

    ROS_DEBUG_NAMED("sparse_front_end::Relocalizer","Found %d match candidates", (int)place_matches.size());

    const PlaceMatchCandidate& match = place_matches[0];

    std::vector<MultiViewMeasurement> meas;
    // For all the keypoints in the candidate
    MultiViewMeasurement z(1);
    MeasurementId zid(query_frame_id, LandmarkId());

    ROS_DEBUG_NAMED("sparse_front_end::Relocalizer", "Creating %d matching measurements", (int)match.match.size());

    for (size_t i = 0; i < match.query.size(); ++i) {
      // Set its landmark_id, frame_id accordingly
      zid.landmark_id = match.query_landmarks[i];
      z.set_id(zid);

      // Set the pixel location from the keypoint
      z.SetPixelInCam(0, match.query[i].pt.x, match.query[i].pt.y);

      // Set Flag to GoodMatch
      z.SetFlag(0, GoodMatch);

      ROS_DEBUG_NAMED("sparse_front_end::Relocalizer","Adding measurement at %f,%f", match.query[i].pt.x, match.query[i].pt.y);
      meas.push_back(z);
    }

    SlamFramePtr matched_frame = map_->GetFramePtr(match.getFrameId());
    if (!matched_frame) {
      ROS_DEBUG_NAMED("sparse_front_end::Relocalizer", "Matched_frame not found. No loop closure completed.");
      continue;
    }

    SlamFramePtr query_frame = map_->GetFramePtr(query_frame_id);
    map_->AddNewMeasurementsToFrame(query_frame_id, meas);
    SlamEdgePtr edge = map_->AddEdge(query_frame, matched_frame, match.Tqm,
                                     true);
      
    ROS_DEBUG_NAMED("sparse_front_end::Relocalizer", "Loop closed with edge %d and T_q_m: %s", edge->id().id, boost::lexical_cast<std::string>(match.Tqm.matrix()).c_str());

    // Release frames while we do the relaxation, since we're going
    // to be iterating over the whole map anyways
    map_->RemoveFrameHold(reloc_hold_token);

    ROS_DEBUG_NAMED("sparse_front_end::Relocalizer", "Relaxing map at %d", relocalization_frame_id_.id);
 
    back_end_opt_.RelaxMap(map_->NumFrames(),
                           query_frame_id, 100,
                           rig_);
    ROS_DEBUG_NAMED("sparse_front_end::Relocalizer", "Map relaxation finished");

    // we are no longer lost
    if (is_lost_) {
      ROS_DEBUG_NAMED("sparse_front_end::Relocalizer", "I'M FOUND");
      is_lost_ = false;
    }
    is_relocalizer_busy_ = false;
  }
}

void FrontEnd::ProcessFailedTracks(
    const std::vector<MultiViewMeasurement>& new_measurements,
    std::vector<std::vector<Feature*> >* feature_matches) {
  for (size_t ii = 0; ii < new_measurements.size(); ++ii) {
    const MultiViewMeasurement& z  = new_measurements[ii];
    const LandmarkId& lm_id = z.id().landmark_id;
    if (lm_id.track2d_id >= 0 && !z.HasGoodMeasurementInCam(0)) {
      if (Feature* feature = feature_matches->at(ii)[0]) {
        feature->used = false;
      }
      ROS_DEBUG_NAMED("sparse_front_end::Iterate", "Bad measurement [%s] of landmark: %d resetting to -1", MatchStr(z.Flag(0)), lm_id.track2d_id);

      if (SlamFramePtr frame = map_->GetFramePtr(
              lm_id.ref_frame_id)) {
        Landmark lm;
        frame->GetLandmark(lm_id.landmark_index, &lm);
        lm.set_track2d_id(-1);
        frame->SetLandmark(lm_id.landmark_index, lm);
      }
    }
  }
}

void FrontEnd::PrintTrack(const Landmark &lm) {
  ROS_INFO("LM %d,%d has %d measurements: ",lm.id().landmark_index, lm.id().track2d_id, (int)lm.GetFeatureTrackRef().size());
  for (const MeasurementId& zid : lm.GetFeatureTrackRef()) {
    MultiViewMeasurement z;
    if (map_->GetMeasurement(zid, z)) {
      ROS_INFO("    %s",z.InfoStr().c_str());
    }
  }
}

inline void ExtractFeaturesWithHandler(
    const std::shared_ptr<FeatureImage>& feature_image,
    FeatureHandler* handler,
    const std::shared_ptr<pb::Image>& image) {
  feature_image->SetImage(image);
  feature_image->ExtractFeatures(*handler);
}

bool FrontEnd::ExtractFeatures(const std::shared_ptr<pb::ImageArray>& frames,
                               FeatureImageVector& feature_images) {
  if (frames->Size() == 0) {
    return false;
  } else if (frames->Size() == 1) {
    ExtractFeaturesWithHandler(feature_images[0],
        &feature_handler_[0],
        frames->at(0));
  } else {
    // extract features (threaded)
    std::vector<std::thread> workers;
    for (size_t ii = 0; ii < rig_.cameras.size(); ++ii) {
      workers.emplace_back(ExtractFeaturesWithHandler,
                           feature_images[ii],
                           &feature_handler_[ii],
                           frames->at(ii));
    }

    for (size_t ii = 0; ii < rig_.cameras.size(); ++ii) {
      workers[ii].join();
    }
  }
  return true;
}

void FrontEnd::ResetFeatures(FeatureImageVector images) {
  for (size_t ii = 0; ii< images.size(); ++ii) {
    images[ii]->ResetFeatures();
  }
}

void FrontEnd::LoadGroundTruth() {
  if (ground_truth_file_.empty() == false) {
    std::ifstream fin;
    Vector6t pose;

    fin.open(ground_truth_file_.c_str());
    if (fin.is_open()) {
      while (!fin.eof()) {
        fin >> pose(0) >> pose(1) >> pose(2) >> pose(3) >> pose(4) >> pose(5);
        ground_truth_.push_back(pose);
      }
    } else {
      ROS_ERROR("ERROR -> LoadGroundTruth(): opening ground truth file failed");
    }
    fin.close();
  }
}

void FrontEnd::PrintPoseCart(Sophus::SE3t &Tab) {
  Vector6t p = rslam::T2Cart(Tab.matrix());
  ROS_INFO("std::setw(8) x: std::setw(13) %f",p(0));
  /*ROS_INFO("y:" << std::setw(13) << p(1);
  ROS_INFO("z:" << std::setw(13) << p(2);
  ROS_INFO("roll:" << std::setw(13) << p(3)*57.2957796;
  ROS_INFO("pitch:"<< std::setw(13) << p(4)*57.2957796;
  ROS_INFO("yaw:"  << std::setw(13) << p(5)*57.2957796;*/
}

Scalar FrontEnd::ComputeRMSRE(const unsigned int window_size) {
  LocalMap localMap;
  LiftLocalPoses(map_.get(), window_size, current_frame_->id(),
                 localMap, has_imu_);
  localMap.PushRelativeCoordsToGlobal();

  double rmsre = 0.0;
  int    num_measurements = 0;

  for (auto & pair : localMap.landmarks) {
    std::shared_ptr<LandmarkContainer> container = pair.second;
    for (std::shared_ptr<MeasurementContainer> z : container->measurements) {
      // compute reprojection error
      Sophus::SE3t Tsw  = rig_.cameras[z->cam_id].T_wc.inverse() *
          localMap.poses[z->id.frame_id].t_wp.inverse();

      const Vector4t x_s = Sophus::MultHomogeneous(Tsw, container->x_w);
      const Vector2t p =
          rig_.cameras[z->cam_id].camera.Project(x_s.head<3>());
      const Vector2t e = p - z->z;
      rmsre += e.squaredNorm();
      num_measurements++;
    }
  }

  rmsre = std::sqrt(rmsre / num_measurements);

  return rmsre;
}

ReferenceFrameId FrontEnd::GetClosestKeyframeId(const ReferenceFrameId& root_id,
                                                const unsigned int depth,
                                                Sophus::SE3t &t_ab) {
  ClosestKeyframeMapVisitor visitor(keyframe_max_angle_,keyframe_max_distance_);
  visitor.set_depth(depth);
  visitor.set_root_id(root_id);
  map_->BFS(&visitor);
  visitor.relative_pose(&t_ab);
  return visitor.closest_keyframe();
}

void FrontEnd::AddPlace(const ReferenceFrameId& id, const cv::Mat& img) {
  //=========================================================
  // Save keyframe thumbnail in our place matcher object
  //=========================================================
  server_interface_.AsyncQueryServerWithPlace(
      map_, place_matcher_, id, img);
#if _BUILD_OVV
  // Iterate over current images to extract the descriptors which are
  // stored in Compressed Row Storage
  /*std::vector<unsigned char *> descriptors;
    unsigned int descsize=0;
    current_feature_images_[0]->GetDescriptors(descriptors, descsize);
    cv::Mat m(descriptors.size(),descsize/4, CV_32F);
    for(unsigned int i=0; i < descriptors.size(); i++)
    {
    cv::Mat tmp(1,descsize/4,CV_32F,descriptors[i]);
    tmp.row(0).copyTo(m.row(i));
    }
    place_matcher_->AddPlace(next_place_id, m); */

#else
  place_matcher_->AddPlace(id, img);
#endif
}

void FrontEnd::Save(const std::string& /*filename*/) const {
}

void FrontEnd::Load(const std::string& /*filename*/) {
}

void FrontEnd::RegisterPoseMeasurement(
    const rslam::map::PoseMeasurement& pose) {
  if (current_frame_) {
    current_frame_->AddPoseMeasurement(pose);
  }
}

void FrontEnd::configCallback(sparse_front_end::SparseFrontEndConfig &config, uint32_t level)
{
  ROS_ERROR("IN FRONT END CONFIG CALLBACK");
  collect_pose_analytics_ = config.collect_pose_analytics;
  use_imu_for_gn_ = config.use_imu_for_gn;
  do_adaptive_window_ = config.do_adaptive_window;
  do_dense_init_ = config.do_dense_init;
  do_bundle_adjustment_= config.do_bundle_adjustment;
  do_async_bundle_adjustment_ = config.do_async_bundle_adjustment;
  use_inverse_depth_parameterization_ = config.use_inverse_depth_parameterization;
  do_keyframing_ = config.do_keyframing;
  do_relocalization_ = config.do_relocalization;

  relocalizer_match_threshold_ = config.relocalizer_match_threshold;
  relocalizer_min_keyframe_separation_ = config.relocalizer_min_keyframe_separation;
  min_keyframes_for_initializing_ = config.min_keyframes_for_initializing;
  keyframe_search_depth_ = config.keyframe_search_depth;
  async_ba_window_size_ = config.async_ba_window_size;
  ba_window_size_ = config.ba_window_size;
  ba_num_iter_ = config.ba_num_iter;
  ba_num_iter_adaptive_ = config.ba_num_iter_adaptive;

  keyframe_threshold_ = config.keyframe_threshold;
  keyframe_max_distance_ = config.keyframe_max_distance;
  keyframe_max_angle_ = config.keyframe_max_angle;

  init_min_keyframes_ = config.init_min_keyframes;
  init_min_disparity_ = config.init_min_disparity;
  init_max_distortion_ = config.init_max_distortion;
  init_min_pctg_init_landmarks_ = config.init_min_pctg_init_landmarks;
  inlier_threshold_ = config.inlier_threshold;

  ground_truth_file_ = config.ground_truth_file;
  timer_window_size_ = config.timer_window_size;
}

}  // namespace sparse
}  // namespace rslam
