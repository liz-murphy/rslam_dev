#pragma once
#include <pb_msgs/ImageArray.h>
#include <place_matching/PlaceMatcher.h>
#include <slam_map/ReferenceFrame.h>
#include <utils/MathTypes.h>
#include <utils/Timer.h>
#include <common_front_end/SystemStatus.h>
#include <common_front_end/TrackingStats.h>
#include <optimization/optimization.h>
#include <thread>
#include <condition_variable>

namespace rslam {
class FrontEnd{
 public:
  virtual bool Init(
      const calibu::CameraRigT<Scalar>  &rig,
      const std::shared_ptr<pb::ImageArray>& frames,
      const double                      timestamp,
      const bool                        has_imu,
      const std::shared_ptr<SlamMap>&    map,
      const std::shared_ptr<PlaceMatcher>& place_matcher,
      const std::shared_ptr<Timer>&     timer,
      bool                              using_sim_data = false);

  virtual void RegisterImuMeasurement(const Eigen::Vector3t &w,
                                      const Eigen::Vector3t & a,
                                      const double time);

 virtual void RegisterPoseMeasurement(
      const rslam::map::PoseMeasurement& pose);

  virtual bool Iterate(const std::shared_ptr<pb::ImageArray>& frames,
                       double timestamp);
  bool IterateBa();

  virtual void tracking_stats(common::TrackingStats *ts) const;
  virtual void system_status(common::SystemStatus *ss) const;
  virtual void Save(const std::string& filename) const;
  virtual void Load(const std::string& filename);

  virtual void GetCurrentKeypointsForDisplay(
      std::vector<std::vector<cv::KeyPoint> > &keypoints);
  virtual SlamFramePtr current_frame() const;
  virtual bool IsInitialized() const;

  void AsyncBaFunc();

 protected:
  /// \brief Pushes the start time of a segment given the name
  /// \param[in] name the name of the element to time
  void Tic(const std::string& name) { if (timer_) timer_->Tic(name); }
  
  /// \brief Pops the start time and calculates duration of a segment
  /// \param[in] the name of the segment  
  void Toc(const std::string& name) { if (timer_) timer_->Toc(name); }


  common::SystemStatus system_status_;
  calibu::Rig<Scalar> rig_;
  calibu::CameraRigT<Scalar> old_rig_;
  std::shared_ptr<PlaceMatcher> place_matcher_;
  std::shared_ptr<SlamMap>      map_;
  std::shared_ptr<Timer>        timer_;
  rslam::optimization::Optimization front_end_opt_, async_ba_;

  SlamFramePtr prev_frame_, current_frame_;
  SlamFramePtr reference_frame_;

  std::thread ba_thread_;
  std::thread relocalizer_thread_;

  bool is_quitting_;
  bool is_async_busy_;
  bool do_landmark_init_;
  mutable std::condition_variable async_ba_cond_;
  ReferenceFrameId initial_frame_;
  optimization::Optimization::RefineMapCallbacks front_end_opt_callbacks_;


};
}  // namespace rslam
