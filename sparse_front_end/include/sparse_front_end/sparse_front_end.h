// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <ba/InterpolationBuffer.h>
#include <optimization/optimization.h>
#include <common_front_end/DenseAlignment.h>
#include <common_front_end/FeatureImage.h>
#include <pb_msgs/ImageArray.h>
#include <pb_msgs/Image.h>
#include <place_matching/PlaceMatcher.h>
#include <slam_map/SlamMapFwd.h>
#include <slam_server/SlamServerFwd.h>
#include <sparse_front_end/FrontEndServerInterface.h>
#include <sparse_tracking/MatchInTime.h>
#include <sparse_tracking/FeatureMask.h>
#include <utils/MathTypes.h>
#include <utils/Timer.h>
#include <common_front_end/front_end.h>

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <sparse_front_end/SparseFrontEndConfig.h>

namespace rslam {
namespace sparse {
struct PoseAnalytics {
  PoseAnalytics(const ReferenceFrameId& id, int window_size,
                const Sophus::SE3t& pose, double time)
      : pose_id(id), async_window_size(window_size), global_pose(pose),
        timestamp(time) {}
  ReferenceFrameId pose_id;
  int         async_window_size;
  Sophus::SE3t     global_pose;
  double           timestamp;
};

class SparseFrontEnd : public FrontEnd{
  typedef PlaceMatchCandidate PlaceMatchCandidateT;
  typedef std::vector<cv::KeyPoint> KeypointVector;
 public:
  SparseFrontEnd();
  ~SparseFrontEnd();

  ///
  /// \brief Initializes the frontend
  /// \param[in] rig holds camera models (intrinsics and extrinsics).
  /// \param[in] frames captured images
  /// \param[in] timestamp the timestamp of the images in frames
  /// \param[in] has_imu indicates whether or not there is an imu
  /// \param[in,out] map pointer to the map.
  /// \param[in,out] place_matcher pointer to localizer
  /// \param[in,out] timer pointer to timer object.
  /// \return
  ///
  bool Init(
      const calibu::CameraRigT<Scalar>  &rig,
      const std::shared_ptr<pb::ImageArray>& frames,
      const double                      timestamp,
      const bool                        has_imu,
      const std::shared_ptr<SlamMap>&    map,
      const std::shared_ptr<PlaceMatcher>& place_matcher,
      const std::shared_ptr<Timer>&     timer,
      bool                              using_sim_data = false);

  ///
  /// \brief Processes a multi-camera frame and imu (if available)
  /// \param[in] frames captured
  /// \param[in] timestamp of the images
  /// \return
  ///
  bool Iterate(const std::shared_ptr<pb::ImageArray>& frames,
               double timestamp);

  ///
  /// \brief Iterates the bundle adjuster
  /// \return
  ///
  bool IterateBa();

  ///
  /// \brief Returns the current keypoints for view in the display
  /// \param[out] vector that is filled with keypoint values
  ///
  void GetCurrentKeypointsForDisplay(std::vector<KeypointVector> &keypoints) {
    for (size_t ii = 0; ii < current_feature_images_.size(); ++ii) {
      keypoints.push_back(current_feature_images_[ii]->GetKeypoints());
    }
  }

  ///
  /// \brief returns current tracking stats for display
  /// \param[out] ts is filled with current tracking data
  ///
  void tracking_stats(common::TrackingStats* ts) const {
    CHECK_NOTNULL(ts);
    *ts = tracking_stats_;
  }

  void system_status(common::SystemStatus *ss) const override {
    CHECK_NOTNULL(ss);
    *ss = system_status_;
  }

  ///
  /// \brief Returns a const ref to the imu calibration struct
  /// \return const ref to the imu calibration struct
  ///
  const ba::ImuCalibrationT<Scalar> &GetImuCalibration() {
    return async_ba_.GetImuCalibration();
  }

  ///
  /// \brief Registers an imu measurement with the BA engine
  /// \param[in] 3 vector of angular rates
  /// \param[in] 3 vector of accelerations
  /// \param[in] timestamp for the measurements
  ///
  void RegisterImuMeasurement(const Eigen::Vector3t &w,
                              const Eigen::Vector3t & a,
                              const double time) {
    SparseFrontEnd::async_ba_.RegisterImuMeasurement(w, a, time);
    SparseFrontEnd::front_end_opt_.RegisterImuMeasurement(w, a, time);
  }

  void RegisterPoseMeasurement(
      const map::PoseMeasurement& pose) override;

  void Save(const std::string& filename) const;
  void Load(const std::string& filename);

  SlamFramePtr current_frame() const {
    return current_frame_;
  }

  void set_server_proxy(const std::shared_ptr<SlamServerProxy>& proxy) {
    server_interface_.set_server_proxy(proxy);
  }

  ///
  /// \returns Whether the system has initialized landmarks yet.
  bool IsInitialized() const;

  ///
  /// \brief Config callback for dynamic reconfigure server
  void configCallback(sparse_front_end::SparseFrontEndConfig &config, uint32_t level);

  ///
  void reset();

protected:
 
  ///
  /// \brief Pushes the start time of a segment given the name
  /// \param[in] name the name of the element to time
  ///
  inline void Tic(const std::string &name = "");
  ///
  /// \brief Pops the start time and calculates duration of a segment
  /// \param[in] the name of the segment
  ///
  inline void Toc(const std::string &name = "");

  void ProcessKeyframe(
      const cv::Mat& query_frame,
      std::vector<MultiViewMeasurement>* new_measurements,
      std::vector<std::vector<Feature*> >* feature_matches);

private:
  ///
  /// \brief If keyframing is enabled, switches to the given keyframe
  /// \param[in] closest_keyframe_id id of the keyframe to switch to
  /// \param[in] t_ab relative edge to this keyframe
  /// \return
  ///
  bool SwitchToKeyframe(const ReferenceFrameId& closest_keyframe_id,
                        const Sophus::SE3t &t_ab);

  ///
  /// \brief Clears the system state
  ///
  void Clear();

  ///
  /// \brief Initialization
  /// \param[in] frames
  /// \return
  ///
  bool Initialization(const std::shared_ptr<pb::ImageArray>& frames,
                      double timestamap);

  ///
  /// \brief builds image pyramids, extract features, etc.
  /// \param[in] frames captured
  /// \param[out] feature image with extracted features
  /// \return
  ///
  bool ExtractFeatures(const std::shared_ptr<pb::ImageArray>& frames,
                       FeatureImageVector &images);

  ///
  /// \brief Properly handles tracks which are prematurely lost
  /// \param[in,out] new_measurements
  /// \param[in,out] feature_matches
  ///
  void ProcessFailedTracks(
      const std::vector<MultiViewMeasurement> &new_measurements,
      std::vector<std::vector<Feature*> >* feature_matches);

  ///
  /// \brief _GetClosestKeyframeId
  /// \param[in] root_id
  /// \param[in] depth
  /// \param[in,out] t_ab
  /// \return the id of the closest keyframe
  ///
  ReferenceFrameId GetClosestKeyframeId(const ReferenceFrameId& root_id,
                                        const unsigned int depth,
                                        Sophus::SE3t &t_ab);

  ///
  /// \brief Returns true if the current pose should be a keyframe
  /// \param[in] t_ab estimated relative transform
  /// \param[in] measurements
  /// \return true if the current pose should be a keyframe
  ///
  bool IsKeyframe(const Sophus::SE3t &t_ab,
                  const std::vector<MultiViewMeasurement> &measurements);

  ///
  /// \brief If match in time fails call this before expanding search
  /// \param [in,out] images
  ///
  void ResetFeatures(FeatureImageVector images);

  ///
  /// \brief Relocalization thread
  ///
  void RelocalizerFunc();

  ///
  /// \brief Bundle adjustment thread
  /// \return
  ///
  void AsyncBaFunc();

  ///
  /// \brief
  /// \param[in] frame id
  /// \param[out] place_matches potential matches
  /// \return
  ///
  void GetPotentialPlaceMatches(
      const ReferenceFrameId& frame_id,
      std::vector<PlaceMatchCandidateT> &place_matches);

  ///
  /// \brief print pose in cartesian values
  /// \param[in] Tab pose to print
  /// \return
  ///
  void PrintPoseCart(Sophus::SE3t& Tab);

  ///
  /// \brief Compute root mean sq reprojection error in a window
  /// \param[in] window_size
  /// \return
  ///
  Scalar ComputeRMSRE(const unsigned int window_size);

  ///
  /// \brief Print sequence of measurements of a landmark
  /// \param[in] landmark object
  /// \return
  ///
  void PrintTrack(const Landmark& lm);

  void SetupFeatureHandlers();

  void AddPlace(const ReferenceFrameId& id, const cv::Mat& img);

private:
  Scalar                        learning_rate_;
  LocalMap                      work_set_;
  Sophus::SE3t                  t_ab_;
  SlamFramePtr                  reference_frame_;
  FeatureHandler                feature_handler_[2];
  FeatureImageVector            current_feature_images_;
  ba::BundleAdjuster<Scalar, 1, 6, 0> gauss_newton_ba_;

  Eigen::Matrix6t                 covariance_;
  DenseAlignment                  dense_aligner_;

  // relocalizer variables
  bool                            has_imu_;
  mutable std::atomic<bool>       is_lost_;
  mutable std::atomic<bool>       is_quitting_localizer_;
  mutable std::atomic<bool>       is_quitting_ba_;
  mutable std::atomic<bool>       is_relocalizer_busy_;
  mutable std::atomic<bool>       is_async_busy_;
  ReferenceFrameId                relocalization_frame_id_;
  ReferenceFrameId                last_successful_relocalization_frame_id;
  std::atomic<ReferenceFrameId>   async_frame_id_;
  uint64_t                        hold_frame_token_;
  FeatureImageVector              keyframe_images_;
  cv::Mat                         query_vector_;
  mutable std::condition_variable relocalizer_cond_;
  mutable std::mutex              relocalizer_mutex_;
  mutable std::condition_variable async_ba_cond_;

  common::TrackingStats           tracking_stats_;

  mutable std::mutex              place_mutex_;
  sparse::FeatureMask             feature_mask_;
  std::list<PoseAnalytics>        pose_analytics_;

  ReferenceFrameId previous_id_;
  Sophus::SE3t last_motion_;
  FrontEndServerInterface server_interface_;
  ReferenceFrameId initial_frame_;

  // configurable variables through dynamic reconfigure
  bool collect_pose_analytics_;
  bool use_imu_for_gn_;
  bool do_adaptive_window_;
  bool do_dense_init_;
  bool do_bundle_adjustment_;
  bool do_async_bundle_adjustment_;
  bool use_inverse_depth_parameterization_;
  bool do_keyframing_;
  bool do_relocalization_;
  int relocalizer_match_threshold_;
  int relocalizer_min_keyframe_separation_;
  int min_keyframe_separation_;
  int min_keyframes_for_initializing_;
  int keyframe_search_depth_;
  int async_ba_window_size_;
  int ba_window_size_;
  int ba_num_iter_;
  int ba_num_iter_adaptive_;
  int use_only_camera_id_;
  double keyframe_threshold_;
  double keyframe_max_distance_;
  double keyframe_max_angle_;

  // Mono
  int init_min_keyframes_;
  double init_min_disparity_;
  double init_max_distortion_;
  double init_min_pctg_init_landmarks_;

  //tracker
  double inlier_threshold_;
  int server_upload_map_size_;
  int server_download_map_size_;

  //gui
  int timer_window_size_;
  double server_query_spread_;
};
}
}
