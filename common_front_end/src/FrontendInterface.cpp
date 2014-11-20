#include <common_front_end/FrontendInterface.h>

bool FrontEndInterface::Init(      
    const calibu::CameraRigT<Scalar>  &rig,
    const std::shared_ptr<pb::ImageArray>& frames,
    const double                      timestamp,
    const bool                        has_imu,
    const std::shared_ptr<SlamMap>&    map,
    const std::shared_ptr<PlaceMatcher>& place_matcher,
    const std::shared_ptr<Timer>&     timer,
    bool                              using_sim_data = false)
{
  // Is there an IMU?
  has_imu_ = has_imu;
  
  // Set up map, place matcher, timer, robot configuration and frontend and backend optimizers
  map_ = map;
  place_matcher_ = place_matcher;
  timer_ = timer;
  timer_->set_window_size(CommonFrontEndConfig::getConfig()->getTimerWindowSize());
  rig_ = rig;
  tracking_stats_.SetRigSize(rig_.cameras.size());
  timer_->set_window_size(CommonFrontEndConfig::getConfig()->getTimerWindowSize());

  // Set up handlers to support tracking (virtual function)
  SetupFeatureHandlers();

  // Set up access to the map
  hold_frame_token_ = map_->GetHoldToken();

  // Set up BA - this instance is used for odometry
  front_end_opt_.Init(map_);

  // Set the camera rig
  calibu::CreateFromOldRig(&rig, &rig_);

  //========================================================================================
  // Process first frame
  //========================================================================================
  current_frame_ = map_->AddFrame(timestamp);
  initial_frame_ = current_frame_->id();

  if(has_imu_) {
    ROS_ASSERT(!front_end_opt_.GetImuBuffer().elements.empty());
    current_frame_->set_b(Eigen::Vector6t::Zero());
    current_frame_->set_v_r(Eigen::Vector3t::Zero());
    current_frame_->set_g_r(front_end_opt_.GetImuBuffer().elements.front().a.normalized()*ba::Gravity);
  }

  current_frame_->set_t_vs(rig_.t_wc_[0]);
  current_frame_->set_cam_params(rig_.cameras_[0]->GetParams());





  



    




}
