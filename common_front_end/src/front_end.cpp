#include <common_front_end/front_end.h>
#include <common_front_end/CommonFrontEndConfig.h>

using namespace rslam;

void FrontEnd::AsyncBaFunc()
{
  ROS_INFO("Starting async ba thread.");
  async_ba_.Init(map_);

  std::chrono::seconds wait_time(1);
  std::mutex async_mutex;
  std::unique_lock<std::mutex> lock(async_mutex);

  while (!is_quitting_) {
    is_async_busy_ = false;
    if (!CommonFrontEndConfig::getConfig()->doAsyncBundleAdjustment()) 
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    } 
    else if (async_ba_cond_.wait_for(lock, wait_time) == std::cv_status::timeout) 
    {
      continue;
    }

    is_async_busy_ = true;
    rslam::optimization::AdaptiveOptions options(CommonFrontEndConfig::getConfig()->doAdaptiveWindow(), true,
        CommonFrontEndConfig::getConfig()->getBAWindowSize(), 100);

    options.ignore_frame = current_frame_->id();

    async_ba_.RefineMap(CommonFrontEndConfig::getConfig()->getAsyncBAWindowSize(),
        prev_frame_->id(),
        CommonFrontEndConfig::getConfig()->getBANumIterAdaptive(),
        old_rig_,
        options,
        false,
        EdgeAttrib_AsyncIsBeingOptimized,
        /* TO DO: Use backend_callbacks_ here 
         * First make tracker interactions thread safe so updating current tracks can be done asynchronously
         */
        {});
  }
  is_async_busy_ = false;
  ROS_INFO("Quitting ba thread.");
}

/// Synchronous BA
bool FrontEnd::IterateBa()
{
  optimization::AdaptiveOptions options(false, false, 15, 100);
  front_end_opt_.RefineMap(CommonFrontEndConfig::getConfig()->getBAWindowSize(),
                     current_frame_->id(),
                     CommonFrontEndConfig::getConfig()->getBANumIter(),
                     old_rig_,
                     options,
                     do_landmark_init_,
                     EdgeAttrib_IsBeingOptimized,
                     front_end_opt_callbacks_);

  return true;
}


    /*bool FrontEndInterface::Init(      
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
    */

