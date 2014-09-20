// Place to store formally global rslam parameters

class FrontEndConfig 
{
public:
   static FrontEndConfig * getConfig();
   int server_debug_level;
   bool collect_pose_analytics;
   bool use_imu_for_gn;
   bool do_adaptive_window;
   bool do_dense_init;
   bool do_bundle_adjustment;
   bool do_async_bundle_adjustment;
   bool use_inverse_depth_parameterization;
   bool do_keyframing;
   bool do_relocalization;
   int relocalizer_match_threshold;
   int min_keyframe_separation;
   int min_keyframes_for_initializing;
   int keyframe_search_depth;
   int async_ba_window_size;
   int ba_window_size;
   int ba_num_iter;
   int ba_num_iter_adaptive;
   int use_only_camera_id;
   double keyframe_threshold;
   double keyframe_max_distance;
   double keyframe_max_angle;

   // Mono
   int init_min_keyframes;
   double init_min_disparity;
   double init_max_distortion;
   double init_min_pctg_init_landmarks;

   //tracker
   double inlier_threshold;
   int server_upload_map_size;
   int server_download_map_size;

   //gui

   private:
   FrontEndConfig(){
     server_debug_level=1;
   };
   static FrontEndConfig * m_configInstance;
};
