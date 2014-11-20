#pragma once
#include <tuple>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <miniglog/logging.h>

#include <optimization/optimization.h>
#include <common_front_end/front_end.h>
#include <common_front_end/SystemStatus.h>
#include <common_front_end/TrackingStats.h>
#include <ba/BundleAdjuster.h>
#include <sdtrack/semi_dense_tracker.h>

#include <semidense_front_end/SemiDenseConfig.h>

namespace rslam {
class SemiDenseFrontEnd : public FrontEnd{
 public:
  SemiDenseFrontEnd();
  ~SemiDenseFrontEnd();
  bool Init(const calibu::CameraRigT<Scalar> &rig,
            const std::shared_ptr<pb::ImageArray> &frames,
            const double timestamp, const bool has_imu,
            const std::shared_ptr<SlamMap> &map,
            const std::shared_ptr<PlaceMatcher> &place_matcher,
            const std::shared_ptr<Timer> &timer,
            bool using_sim_data) override;
  void RegisterImuMeasurement(const Eigen::Vector3t &w,
                              const Eigen::Vector3t &a,
                              const double time) override;
  void RegisterPoseMeasurement(
      const rslam::map::PoseMeasurement& pose) override;

  bool Iterate(const std::shared_ptr<pb::ImageArray> &frames,
               double timestamp) override;
  bool IterateBa() override;
  void tracking_stats(common::TrackingStats *ts) const override;
  void system_status(common::SystemStatus *ss) const override;
  void Save(const std::string &filename) const override;
  void Load(const std::string &filename) override;
  void GetCurrentKeypointsForDisplay(
      std::vector<std::vector<cv::KeyPoint> > &keypoints) override;
  void set_server_proxy(const std::shared_ptr<SlamServerProxy> &proxy) override;
  SlamFramePtr current_frame() const override;
  bool IsInitialized() const override;

  void configCallback(semidense_front_end::SemiDenseConfig &config, uint32_t level);

 protected:
  void AddKeyframe();
  void CreateLandmark(const std::shared_ptr<sdtrack::DenseTrack> &track,
                      LandmarkId* lm_id);
  void DoSynchronousBundleAdjustment();
  void UpdateStats();
  void AsyncBaFunc();
  void Tic(const std::string& name) { if (timer_) timer_->Tic(name); }
  void Toc(const std::string& name) { if (timer_) timer_->Toc(name); }

 private:
  uint32_t keyframe_tracks_ = UINT_MAX;
  Sophus::SE3t prev_t_ba_, prev_delta_t_ba_;

  bool is_keyframe_ = true;
  bool is_prev_keyframe_ = true;
  bool is_simulation_;
  bool has_imu_;
  std::atomic<bool> is_quitting_;

  uint64_t hold_frame_token_;

  sdtrack::SemiDenseTracker tracker_;

  SlamFramePtr prev_frame_;
  optimization::Optimization::RefineMapCallbacks front_end_opt_callbacks_;
  rslam::common::TrackingStats tracking_stats_;
  std::condition_variable aac_cond_;

  SlamEdgePtr current_edge_;

  // parameters
  int pyramid_levels_;
  int patch_size_;
  int num_features_;
  int feature_cells_;
  double tracker_center_weight_;
  double ncc_threshold_;
  double lmk_outlier_ratio_threshold_;

  bool use_imu_for_guess_;
  bool use_imu_measurements_;
  int min_poses_for_imu_;
};
}  // namespace rslam
