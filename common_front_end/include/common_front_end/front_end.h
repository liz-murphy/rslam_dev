#pragma once
#include <pb_msgs/ImageArray.h>
#include <place_matching/PlaceMatcher.h>
#include <slam_map/SlamMapFwd.h>
#include <slam_server/SlamServerFwd.h>
#include <utils/MathTypes.h>
#include <utils/Timer.h>
#include <common_front_end/common_front_end_fwd.h>
#include <common_front_end/SystemStatus.h>
#include <common_front_end/TrackingStats.h>
#include <optimization/optimization.h>
#include <thread>

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
      bool                              using_sim_data = false) = 0;

  virtual void RegisterImuMeasurement(const Eigen::Vector3t &w,
                                      const Eigen::Vector3t & a,
                                      const double time) = 0;

 virtual void RegisterPoseMeasurement(
      const rslam::map::PoseMeasurement& pose) = 0;

  virtual bool Iterate(const std::shared_ptr<pb::ImageArray>& frames,
                       double timestamp) = 0;
  virtual bool IterateBa() = 0;

  virtual void tracking_stats(common::TrackingStats *ts) const = 0;
  virtual void system_status(common::SystemStatus *ss) const = 0;
  virtual void Save(const std::string& filename) const = 0;
  virtual void Load(const std::string& filename) = 0;

  virtual void GetCurrentKeypointsForDisplay(
      std::vector<std::vector<cv::KeyPoint> > &keypoints) = 0;
  virtual void set_server_proxy(
      const std::shared_ptr<SlamServerProxy>& proxy) = 0;
  virtual SlamFramePtr current_frame() const = 0;
  virtual bool IsInitialized() const = 0;

 protected:
  common::SystemStatus system_status_;
  calibu::Rig<Scalar> rig_;
  calibu::CameraRigT<Scalar> old_rig_;
  std::shared_ptr<PlaceMatcher> place_matcher_;
  std::shared_ptr<SlamMap>      map_;
  std::shared_ptr<Timer>        timer_;
  rslam::optimization::Optimization front_end_opt_, async_ba_;

  SlamFramePtr current_frame_;
  SlamFramePtr reference_frame_;

  std::thread ba_thread_;
  std::thread relocalizer_thread_;




};
}  // namespace rslam
