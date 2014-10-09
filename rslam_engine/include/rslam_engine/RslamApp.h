#ifndef RSLAM_APP_H
#define RSLAM_APP_H

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
#include <tf/transform_broadcaster.h>


class RslamApp
{
  public:
    RslamApp(std::string &image_topic);
    bool Capture(pb::CameraMsg& images_msg, const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image);
    void visLoop(double vis_publish_period);
    void stereo_callback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::CameraInfoConstPtr& left_cam_info, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& right_cam_info);
    
  private:
    image_transport::Publisher pub0;
    ros::Publisher marker_pub;
    ros::Publisher landmark_pub;

    bool gui_init;
    std::shared_ptr<rslam::RslamEngine> engine;
    std::shared_ptr<GlobalMapView> global_view_;
    rslam::RslamEngineOptions options;

    ros::NodeHandle nh_;

    std::string image_topic_;
    bool received_left_cam_info_, received_right_cam_info_;
    sensor_msgs::CameraInfo left_cam_info_;

    message_filters::Subscriber<sensor_msgs::Image> *left_image_sub, *right_image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *left_info_sub, *right_info_sub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> *sync;

    calibu::CameraRigT<Scalar> rig;
    bool engine_initialized;

    std::shared_ptr<Gui> gui_;

    boost::thread* vis_thread_;
    std::shared_ptr<pb::ImageArray> images_;

    Sophus::SE3t initial_pose_;

    tf::TransformBroadcaster map_broadcaster_;
};

#endif
