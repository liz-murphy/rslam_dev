// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

// Glue code:
// 1) captues images
// 2) iterates the SLAM system

#include <calibu/Calibu.h>
#include <common_front_end/FrontendInterface.h>
#include <common_front_end/SparseSimData.h>
#include <common_front_end/TrackingStats.h>
#include <pb_msgs/Image.h>
#include <pb_msgs/ImageArray.h>
#include <rslam_engine/RslamEngineOptions.h>
#include <rslam_engine/RslamEngineConfig.h>
#include <slam_server/SlamServerProxy.h>
#include <slam_map/SlamMapFwd.h>
#include <utils/ImageProcessing.h>
#include <dynamic_reconfigure/server.h>
#include <sparse_front_end/SparseFrontEndConfig.h>
#include <common_front_end/CommonFrontEndConfig.h>
#include <common_front_end/CommonFrontEndParamsConfig.h>
#include <back_end/BackEndConfig.h>
#include <back_end/BackEndParamsConfig.h>
#include <common_front_end/FREAKConfig.h>
#include <common_front_end/FASTConfig.h>
#include <common_front_end/SURFConfig.h>
#include <sparse_front_end/FrontEnd.h>

class PlaceMatcher;
class Timer;
namespace rslam {
class RslamEngine {
 public:
  RslamEngine();
  ~RslamEngine();

  // Main  API.

  /** Reset internal structures of engine */
  bool Reset(const RslamEngineOptions& options,
             const calibu::CameraRigT<Scalar>& rig_in);

  /** Initialize with first data */
  bool Init(const std::shared_ptr<pb::ImageArray>& images);
  void Iterate(const std::shared_ptr<pb::ImageArray>& images);
  bool IterateBa();

  // End the app
  void Finish();

  // Save all important data to disk.
  void Save();

  double Timestamp();
  void ImuCallbackHandler(const pb::ImuMsg& ref);
  void PosysCallbackHandler(const pb::PoseMsg& ref);
  void PrintAppInfo();

  void tracking_stats(common::TrackingStats* ts) const {
    CHECK_NOTNULL(ts);
    frontend_->tracking_stats(ts);
  }

  void system_status(common::SystemStatus *ss) const {
    CHECK_NOTNULL(ss);
    frontend_->system_status(ss);
  }

  ReferenceFrameId first_frame() const {
    return first_frame_;
  }

  // Public access objects
  //std::shared_ptr<rslam::FrontEndInterface> frontend_;
  std::shared_ptr<sparse::FrontEnd> frontend_;
 
  // ROS pollution for dynamic reconfigure
  std::shared_ptr<dynamic_reconfigure::Server<sparse_front_end::SparseFrontEndConfig> > sparse_frontend_dr_srv_;
  std::shared_ptr<dynamic_reconfigure::Server<common_front_end::CommonFrontEndParamsConfig> > common_frontend_dr_srv_;
  std::shared_ptr<dynamic_reconfigure::Server<back_end::BackEndParamsConfig> > back_end_dr_srv_;
  std::shared_ptr<dynamic_reconfigure::Server<common_front_end::FREAKConfig> > FREAK_dr_srv_;
  std::shared_ptr<dynamic_reconfigure::Server<common_front_end::FASTConfig> > FAST_dr_srv_;
  std::shared_ptr<dynamic_reconfigure::Server<common_front_end::SURFConfig> > SURF_dr_srv_;
 
  dynamic_reconfigure::Server<sparse_front_end::SparseFrontEndConfig>::CallbackType sparse_front_end_cb;
  dynamic_reconfigure::Server<common_front_end::CommonFrontEndParamsConfig>::CallbackType common_front_end_cb;
  dynamic_reconfigure::Server<back_end::BackEndParamsConfig>::CallbackType back_end_cb;
  dynamic_reconfigure::Server<common_front_end::FREAKConfig>::CallbackType FREAK_cb;
  dynamic_reconfigure::Server<common_front_end::FASTConfig>::CallbackType FAST_cb;
  dynamic_reconfigure::Server<common_front_end::SURFConfig>::CallbackType SURF_cb;
 
  std::shared_ptr<SlamMap> map_;
  calibu::CameraRigT<Scalar> rig_;
  std::shared_ptr<Timer> timer_;
  common::TrackingStats tracking_stats_;
  std::shared_ptr<pb::ImageArray> images_;

 protected:
  void SaveFrames() const;
  void ResetVars();
  bool HasLookupTable(const std::string& sRigFile);
  void SetupSimulator(const RslamEngineOptions& options);
  bool InitResetCameras(const RslamEngineOptions& options,
                        const calibu::CameraRigT<Scalar>& rig);
  bool LoadSimFrame();
  void LoadCurrentImages();

 private:
  bool                           is_using_sim_imu_;
  bool                           is_using_sim_data_;
  bool                           is_mono_tracking_;
  bool                           is_tracking_2d_;
  bool                           is_rectified_;
  bool                           is_continuing_;
  bool                           have_imu_;
  bool                           persist_map_;
  int                            active_camera_id_;
  int64_t                        frame_number_, last_used_frame_number_;
  std::shared_ptr<PlaceMatcher>  place_matcher_;
  ImageProcessing                image_proc_;
  std::vector<cv::Mat>           rectified_frames_;
  std::vector<Eigen::Vector4i>   rois_;
  std::shared_ptr<SparseSimData> simulator_;
  std::string                    working_directory_;
  std::string                    error_results_file_;
  ReferenceFrameId               first_frame_;

  std::map<unsigned int, ReferenceFrameId> sim_frames_;
  std::shared_ptr<SlamServerProxy> server_proxy_;
  RslamTracker                   tracker_type_;
  //std::unique_ptr<geocon::geodetic2local> lla2local_;
  CommonFrontEndConfig* common_front_end_config_;
  BackEndConfig* back_end_config_;
};
}
