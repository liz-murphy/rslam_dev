#include <rslam_engine/RslamApp.h>

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
