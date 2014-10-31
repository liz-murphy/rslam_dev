#include <rslam_engine/RslamApp.h>
#include <common_front_end/CommonFrontEndConfig.h>

using namespace rslam;
using namespace sensor_msgs;
using namespace message_filters;

//BackEndConfig *BackEndConfig::m_configInstance = 0;
ServerConfig *ServerConfig::m_configInstance = 0;
CommonFrontEndConfig *CommonFrontEndConfig::m_configInstance = 0;
BackEndConfig *BackEndConfig::s_instance = 0;
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
  sync->registerCallback(boost::bind(&RslamApp::stereo_callback, this, _1, _2, _3, _4));

  images_ = pb::ImageArray::Create();
      vis_thread_ = new boost::thread(boost::bind(&RslamApp::visLoop, this, vis_publish_period));



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
    if(engine_initialized)
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
  std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();
  Capture(images->Ref(), left_image, right_image);
  if(!engine_initialized)
  {
   calibu::CameraRig rig;
    calibu::CameraModelAndTransform cop_left;
    calibu::CameraModel cam_left("calibu_fu_fv_u0_v0");
    cam_left.SetVersion(8);
    cam_left.SetName("left");
    cam_left.SetIndex(0);
    cam_left.SetSerialNumber(0);
    cam_left.SetImageDimensions(left_cam_info->width, left_cam_info->height);
    ROS_INFO("Dimensions set");
    Eigen::VectorXd params(4);  // matches up with fu, fv, u0, v0
    params(0) = left_cam_info->K[0];
    params(1) = left_cam_info->K[4];
    params(2) = left_cam_info->K[2];
    params(3) = left_cam_info->K[5];
    cam_left.SetGenericParams(params);
    ROS_INFO("Params set");
    Eigen::MatrixXd rdf = Eigen::MatrixXd::Zero(3,3);
    rdf(0,1) = 1;
    rdf(1,2) = 1;
    rdf(2,0) = 1;
    cam_left.SetRDF(rdf.transpose());
    ROS_INFO("RDF set");
    cop_left.camera = cam_left;
    Eigen::MatrixXd rotation = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd translation(3,1);
    translation(0) = left_cam_info->P[3];
    translation(1) = left_cam_info->P[7];
    translation(2) = left_cam_info->P[11];
    std::cout << "translation: " << translation << "\n";
    std::cout << "rotation: " << rotation << "\n";
    cop_left.T_wc = Sophus::SE3(rotation,translation);
    rig.cameras.push_back(cop_left);
    ROS_INFO("Left done...");

    calibu::CameraModelAndTransform cop_right;
    calibu::CameraModel cam_right("calibu_fu_fv_u0_v0");
    cam_right.SetVersion(8);
    cam_right.SetName("right");
    cam_right.SetIndex(1);
    cam_right.SetSerialNumber(0);
    cam_right.SetImageDimensions(right_cam_info->width, right_cam_info->height);
    params(0) = right_cam_info->K[0];
    params(1) = right_cam_info->K[4];
    params(2) = right_cam_info->K[2];
    params(3) = right_cam_info->K[5];
    cam_right.SetGenericParams(params);
    cam_right.SetRDF(rdf.transpose());
    cop_right.camera = cam_right;

    rotation = Eigen::MatrixXd::Identity(3,3);
    translation(0) = right_cam_info->P[3];
    translation(1) = right_cam_info->P[7];
    translation(2) = left_cam_info->P[11];
    std::cout << "translation: " << translation << "\n";
    std::cout << "rotation: " << rotation << "\n";
    cop_right.T_wc = Sophus::SE3(rotation,translation);
    rig.cameras.push_back(cop_right);
    ROS_INFO("Camera rig initialized");

    calibu::WriteXmlRig("calibu_calibration.xml",rig);

/*
    // quick hack to read in camera models
  std::string filename = "/home/liz/Data/GWU/cameras.xml";
  rig = calibu::ReadXmlRig(filename);
  if(rig.cameras.empty())
    ROS_ERROR("Camera rig is empty");
  else
    ROS_INFO("Loaded camera rig");
*/

    options.tracker_type_ = Tracker_Sparse;
    engine->Reset(options,rig);
    gui_ = std::make_shared<Gui>();
    gui_->set_map(std::make_shared<PointerSlamMapProxy>(engine->map_));
    global_view_.reset(new GlobalMapView(std::make_shared<PointerSlamMapProxy>(engine->map_)));
    gui_->SetCameraRig(rig);
    initial_pose_ = Sophus::SE3t();
    options.place_matcher_options.parameters.set(1);
    options.place_matcher_options.parameters.k = 0;
    options.place_matcher_options.parameters.dislocal = -1;
    options.place_matcher_options.parameters.use_nss = false;
    options.place_matcher_options.parameters.alpha = 0.03;
    engine->Init(images);
    engine_initialized = true;
    ROS_INFO("SLAM engine initialized");
  }
  engine->Iterate(images);

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
  gui_->Update(images, engine->frontend_->current_frame()->id(), keypoints);
}



