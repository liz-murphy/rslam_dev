#include <rslam_engine/RslamApp.h>
#include <common_front_end/CommonFrontEndConfig.h>

using namespace rslam;
using namespace sensor_msgs;
using namespace message_filters;

//BackEndConfig *BackEndConfig::m_configInstance = 0;
ServerConfig *ServerConfig::m_configInstance = 0;
CommonFrontEndConfig *CommonFrontEndConfig::m_configInstance = 0;
OptimizationConfig *OptimizationConfig::s_instance = 0;
TrackingConfig *TrackingConfig::m_configInstance = 0;

RslamApp::RslamApp(std::string &image_topic)
{
  gui_init = false;
  image_transport::ImageTransport it0(nh_);
  pub0 = it0.advertise("vis/camera_0", 1);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("slam_map", 10);
  landmark_pub = nh_.advertise<visualization_msgs::Marker>("landmarks", 10);
  double vis_publish_period;
  nh_.param("vis_publish_period", vis_publish_period, 10.0);

  engine.reset(new RslamEngine());
  engine->PrintAppInfo();
  engine_initialized = false;
  left_image_sub = new message_filters::Subscriber<Image>(nh_, "camera/left/image", 1000);
  left_info_sub = new message_filters::Subscriber<CameraInfo>(nh_, "camera/left/camera_info", 1000);
  right_image_sub = new message_filters::Subscriber<Image>(nh_, "camera/right/image", 1000);
  right_info_sub = new message_filters::Subscriber<CameraInfo>(nh_, "camera/right/camera_info", 1000);
  
  sync = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(10),*left_image_sub, *left_info_sub, *right_image_sub, *right_info_sub);
 // sync = new TimeSynchronizer<Image, CameraInfo, Image, CameraInfo>(*left_image_sub, *left_info_sub, *right_image_sub, *right_info_sub, 10000);
  sync->registerCallback(boost::bind(&RslamApp::stereo_callback, this, _1, _2, _3, _4));

  images_ = pb::ImageArray::Create();
  // quick hack to read in camera models
  std::string filename = "/home/liz/Data/GWU/cameras.xml";
  rig = calibu::ReadXmlRig(filename);
  if(rig.cameras.empty())
    ROS_ERROR("Camera rig is empty");
  else
    ROS_INFO("Loaded camera rig");

  engine->Reset(options,rig);
  gui_ = std::make_shared<Gui>();
  gui_->set_map(std::make_shared<PointerSlamMapProxy>(engine->map_));
  global_view_.reset(new GlobalMapView(std::make_shared<PointerSlamMapProxy>(engine->map_)));
  gui_->SetCameraRig(rig);

  vis_thread_ = new boost::thread(boost::bind(&RslamApp::visLoop, this, vis_publish_period));

  initial_pose_ = Sophus::SE3t();

  options.place_matcher_options.parameters.set(1);
  options.place_matcher_options.parameters.k = 0;
  options.place_matcher_options.parameters.dislocal = -1;
  options.place_matcher_options.parameters.use_nss = false;
  options.place_matcher_options.parameters.alpha = 0.03;
}

bool RslamApp::Capture(pb::CameraMsg& images_msg, const ImageConstPtr& left_image, const ImageConstPtr& right_image)
{
  pb::ImageMsg* l_msg = images_msg.add_image();
  pb::ImageMsg* r_msg = images_msg.add_image();

  images_msg.set_device_time((left_image->header.stamp).toSec());

  l_msg->set_type(pb::PB_UNSIGNED_BYTE);
  l_msg->set_height(left_image->height);
  l_msg->set_width(left_image->width);
  l_msg->set_format(pb::PB_LUMINANCE);
  l_msg->set_data(&left_image->data[0], left_image->step*left_image->height);

  r_msg->set_type(pb::PB_UNSIGNED_BYTE);
  r_msg->set_height(right_image->height);
  r_msg->set_width(right_image->width);
  r_msg->set_format(pb::PB_LUMINANCE);
  r_msg->set_data(&right_image->data[0], right_image->step*right_image->height);
}

void RslamApp::visLoop(double vis_publish_period)
{
  if(vis_publish_period = 0)
    return;

  ros::Rate r(10.0);
  while(ros::ok())
  {
    if(gui_->init())
    {
      cv::Mat im;
      gui_->GetDisplayImage(im);

      cv_bridge::CvImagePtr cv_ptr_im(new cv_bridge::CvImage);
      cv_ptr_im->image = im;
      cv_ptr_im->encoding = sensor_msgs::image_encodings::BGR8;
      sensor_msgs::ImagePtr img_msg = cv_ptr_im->toImageMsg();
      img_msg->header.stamp = ros::Time::now();
      pub0.publish(img_msg);
      
      visualization_msgs::Marker slam_map_msg;
      visualization_msgs::Marker landmark_msg;
      if(gui_->GetMap(slam_map_msg))
        marker_pub.publish(slam_map_msg);
      if(gui_->GetLandmarks(landmark_msg))
        landmark_pub.publish(landmark_msg);
    }
    r.sleep();
  }
}

void RslamApp::stereo_callback(const ImageConstPtr& left_image, const CameraInfoConstPtr& left_cam_info, const ImageConstPtr& right_image, const CameraInfoConstPtr&     right_cam_info)
{
  images_->Ref().Clear();
  Capture(images_->Ref(), left_image, right_image);
  if(!engine_initialized)
  {
    engine->Init(images_);
    engine_initialized = true;
    ROS_INFO("SLAM engine initialized");
  }
  engine->Iterate(images_);

  // Update transforms
  // Get relative transform and estimate visual odometry

  // Get global map pose (relates camera to map)
  // Use this to calculate map-->odom
  Sophus::SE3t* out = new Sophus::SE3t();
  if(global_view_->GetFramePose(engine->frontend_->current_frame()->id(),out))
  {
    *out = initial_pose_*(*out);
    geometry_msgs::TransformStamped map_trans;
    map_trans.header.stamp = ros::Time::now();
    map_trans.header.frame_id = "map";
    map_trans.child_frame_id = "base_link";

    map_trans.transform.translation.x = out->data()[4];
    map_trans.transform.translation.y = out->data()[5];
    map_trans.transform.translation.z = out->data()[6];
    map_trans.transform.rotation.x = out->data()[0];
    map_trans.transform.rotation.y = out->data()[1];
    map_trans.transform.rotation.z = out->data()[2];
    map_trans.transform.rotation.w = out->data()[3];
    map_broadcaster_.sendTransform(map_trans);
  }
  engine->timer_->PrintToTerminal(2);
  std::vector<std::vector<cv::KeyPoint> > keypoints;
  engine->frontend_->GetCurrentKeypointsForDisplay(keypoints);
  gui_->Update(images_, engine->frontend_->current_frame()->id(), keypoints);
}



