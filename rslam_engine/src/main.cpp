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

using namespace rslam;
using namespace sensor_msgs;
using namespace message_filters;

//typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
//typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

class RslamApp
{
  private:
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

  public:
    RslamApp(std::string &image_topic)
    {
      left_image_sub = new message_filters::Subscriber<Image>(nh_, "camera/left/image", 10);
      left_info_sub = new message_filters::Subscriber<CameraInfo>(nh_, "camera/left/camera_info", 10);
      right_image_sub = new message_filters::Subscriber<Image>(nh_, "camera/right/image", 10);
      right_info_sub = new message_filters::Subscriber<CameraInfo>(nh_, "camera/right/camera_info", 10);
      
      sync = new TimeSynchronizer<Image, CameraInfo, Image, CameraInfo>(*left_image_sub, *left_info_sub, *right_image_sub, *right_info_sub, 100);
      sync->registerCallback(boost::bind(&RslamApp::stereo_callback, this, _1, _2, _3, _4));

      std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();

    }

    void stereo_callback(const ImageConstPtr& left_image, const CameraInfoConstPtr& left_cam_info, const ImageConstPtr& right_image, const CameraInfoConstPtr& right_cam_info)
    {
      ROS_INFO("Got stereo images and info");
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
