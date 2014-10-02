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
image_transport::Publisher pub1;

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

    calibu::CameraRigT<Scalar> rig;
    bool engine_initialized;

  public:
    RslamApp(std::string &image_topic)
    {
      engine.reset(new RslamEngine());
      engine->PrintAppInfo();
      engine_initialized = false;
      left_image_sub = new message_filters::Subscriber<Image>(nh_, "camera/left/image", 1000);
      left_info_sub = new message_filters::Subscriber<CameraInfo>(nh_, "camera/left/camera_info", 1000);
      right_image_sub = new message_filters::Subscriber<Image>(nh_, "camera/right/image", 1000);
      right_info_sub = new message_filters::Subscriber<CameraInfo>(nh_, "camera/right/camera_info", 1000);
      
      sync = new TimeSynchronizer<Image, CameraInfo, Image, CameraInfo>(*left_image_sub, *left_info_sub, *right_image_sub, *right_info_sub, 10000);
      sync->registerCallback(boost::bind(&RslamApp::stereo_callback, this, _1, _2, _3, _4));

      //std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();
      // quick hack to read in camera models
      std::string filename = "/home/liz/Data/GWU/cameras.xml";
      rig = calibu::ReadXmlRig(filename);
      if(rig.cameras.empty())
        ROS_ERROR("Camera rig is empty");
      else
        ROS_INFO("Loaded camera rig");

      engine->Reset(options,rig);
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
     // engine->Init(images);

       r_msg->set_type(pb::PB_UNSIGNED_BYTE);
       r_msg->set_height(right_image->height);
       r_msg->set_width(right_image->width);
       r_msg->set_format(pb::PB_LUMINANCE);
       r_msg->set_data(&right_image->data[0], right_image->step*right_image->height);
    }

    void stereo_callback(const ImageConstPtr& left_image, const CameraInfoConstPtr& left_cam_info, const ImageConstPtr& right_image, const CameraInfoConstPtr& right_cam_info)
    {
       std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();
      Capture(images->Ref(), left_image, right_image);
       if(!engine_initialized)
       {
         engine->Init(images);
         engine_initialized = true;
         ROS_INFO("SLAM engine initialized");
       }
       ROS_INFO("Got stereo images and info");
       engine->Iterate(images);
       // Display
       std::vector<std::vector<cv::KeyPoint> > keypoints;
       engine->frontend_->GetCurrentKeypointsForDisplay(keypoints);

       cv_bridge::CvImagePtr cv_ptr_left;
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

       cur_frames_.clear();
       cur_frames_.push_back(cv_ptr_left->image.clone());
       cur_frames_.push_back(cv_ptr_right->image.clone());

       // Grab info from the front end
       std::vector<MultiViewMeasurement> cur_meas(num_meas, MultiViewMeasurement(rig.cameras.size()));

       track_info_->Update(cur_frames_, cur_measurements_, cur_keypoints_, current_frame_id_);

       //cv::drawKeypoints(cv_ptr_left->image, keypoints[0], cv_ptr_left->image);
       //cv::drawKeypoints(cv_ptr_right->image, keypoints[1], cv_ptr_right->image);
       //pub0.publish(cv_ptr_left->toImageMsg());
       //pub1.publish(cv_ptr_right->toImageMsg());

       // Both images, tracked landmarks and stereo correspondences
       cv::Size sz1 = cv_ptr_left->image.size();
       cv::Size sz2 = cv_ptr_right->image.size();
       cv::Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
       // Move right boundary to the left.
       im3.adjustROI(0, 0, 0, -sz2.width);
       cv_ptr_left->image.copyTo(im3);
       // Move the left boundary to the right, right boundary to the right.
       im3.adjustROI(0, 0, -sz1.width, sz2.width);
       cv_ptr_right->image.copyTo(im3);
       // restore original ROI.
       im3.adjustROI(0, 0, sz1.width, 0);

       cv::namedWindow( "Feature Matching", cv::WINDOW_AUTOSIZE );// Create a window for display.
       cv::imshow( "Feature Matching", im3);  
       cv::waitKey(0);
    }

 };

int main(int argc, char **argv)
{
  // Set up image subscribers
  ros::init(argc, argv, "rslam_app");
  ros::NodeHandle nh("~"); 

  std::string image_topic;
  nh.param<std::string>("stereo_topic",image_topic,"/camera");

  image_transport::ImageTransport it0(nh);
  pub0 = it0.advertise("vis/camera_0", 1);

  image_transport::ImageTransport it1(nh);
  pub1 = it1.advertise("vis/camera_1", 1);

  RslamApp* rslam_app = new RslamApp(image_topic);
  while(ros::ok())
  {
    ros::spin();
  }

  delete rslam_app;
  return 0;

}
