#include <ros/ros.h>
#include <rslam_engine/RslamEngine.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/publisher.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

#include <string>

#include <calibu/cam/CameraRig.h>
#include <back_end/BackEndConfig.h>
#include <slam_server/ServerConfig.h>
#include <common_front_end/CommonFrontEndConfig.h>
#include <sparse_front_end/FrontEndConfig.h>
#include <sparse_tracking/TrackingConfig.h>

#include <image_transport/image_transport.h>

#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <rslam_engine/TrackView.h>
#include <rslam_engine/Gui.h>

#include <slam_map/PointerSlamMapProxy.h>

#include <boost/thread.hpp>

#include <utils/Timer.h>

#include <visualization_msgs/Marker.h>

using namespace rslam;
using namespace sensor_msgs;
using namespace message_filters;

//typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
//typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

BackEndConfig *BackEndConfig::s_instance = 0;
ServerConfig *ServerConfig::m_configInstance = 0;
CommonFrontEndConfig *CommonFrontEndConfig::m_configInstance = 0;
FrontEndConfig *FrontEndConfig::m_configInstance = 0;
TrackingConfig *TrackingConfig::m_configInstance = 0;

image_transport::Publisher pub0;
ros::Publisher marker_pub;


class RslamApp
{
  private:
    bool gui_init;
    std::shared_ptr<RslamEngine> engine;
    std::shared_ptr<GlobalMapView> global_view;
    RslamEngineOptions options;

    ros::NodeHandle nh_;

    std::string image_topic_;
    bool received_left_cam_info_, received_right_cam_info_;
    CameraInfo left_cam_info_;

    message_filters::Subscriber<Image> *left_image_sub, *right_image_sub;
    message_filters::Subscriber<CameraInfo> *left_info_sub, *right_info_sub;
    TimeSynchronizer<Image, CameraInfo, Image, CameraInfo> *sync;

    calibu::CameraRigT<Scalar> rig;
    bool engine_initialized;

    std::shared_ptr<Gui> gui_;

    boost::thread* vis_thread_;
    std::shared_ptr<pb::ImageArray> images_;
  public:
    RslamApp(std::string &image_topic)
    {
      gui_init = false;
      image_transport::ImageTransport it0(nh_);
      pub0 = it0.advertise("vis/camera_0", 1);
      marker_pub = nh_.advertise<visualization_msgs::Marker>("slam_map", 10);
      double vis_publish_period;
      nh_.param("vis_publish_period", vis_publish_period, 10.0);

      engine.reset(new RslamEngine());
      engine->PrintAppInfo();
      engine_initialized = false;
      left_image_sub = new message_filters::Subscriber<Image>(nh_, "camera/left/image", 1000);
      left_info_sub = new message_filters::Subscriber<CameraInfo>(nh_, "camera/left/camera_info", 1000);
      right_image_sub = new message_filters::Subscriber<Image>(nh_, "camera/right/image", 1000);
      right_info_sub = new message_filters::Subscriber<CameraInfo>(nh_, "camera/right/camera_info", 1000);
      
      sync = new TimeSynchronizer<Image, CameraInfo, Image, CameraInfo>(*left_image_sub, *left_info_sub, *right_image_sub, *right_info_sub, 10000);
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
      gui_->SetCameraRig(rig);
      
      vis_thread_ = new boost::thread(boost::bind(&RslamApp::visLoop, this, vis_publish_period));
    }

    bool Capture(pb::CameraMsg& images_msg, const ImageConstPtr& left_image, const ImageConstPtr& right_image)
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

    void visLoop(double vis_publish_period)
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
          if(gui_->GetMap(slam_map_msg))
            marker_pub.publish(slam_map_msg);
        }
        r.sleep();
      }
    }

    void stereo_callback(const ImageConstPtr& left_image, const CameraInfoConstPtr& left_cam_info, const ImageConstPtr& right_image, const CameraInfoConstPtr& right_cam_info)
    {
       images_->Ref().Clear();
       Capture(images_->Ref(), left_image, right_image);
       if(!engine_initialized)
       {
         engine->Init(images_);
         engine_initialized = true;
         ROS_INFO("SLAM engine initialized");
       }
       ROS_INFO("Got stereo images and info");
       engine->Iterate(images_);
       engine->timer_->PrintToTerminal(2);
       std::vector<std::vector<cv::KeyPoint> > keypoints;
       engine->frontend_->GetCurrentKeypointsForDisplay(keypoints);
       gui_->Update(images_, engine->frontend_->current_frame()->id(), keypoints);
       // Display

       /*cv_bridge::CvImagePtr cv_ptr_left;
       cv_bridge::CvImagePtr cv_ptr_right;
       
       try
       {
         cv_ptr_left = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::MONO8);
         cv_ptr_right = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::MONO8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
       }

       gui_init = true;
 */      
    }

 };

int main(int argc, char **argv)
{
  // Set up image subscribers
  ros::init(argc, argv, "rslam_app");
  ros::NodeHandle nh("~"); 

 
  std::string image_topic;
  nh.param<std::string>("stereo_topic",image_topic,"/camera");

   RslamApp* rslam_app = new RslamApp(image_topic);

  while(ros::ok())
  {
    ros::spin();
  }

  delete rslam_app;
  return 0;

}
