// Place to store formally global rslam parameters
#ifndef FRONT_END_CONFIG_H
#define FRONT_END_CONFIG_H

class FrontEndConfig 
{
public:
   static FrontEndConfig * getConfig();
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
   int relocalizer_min_keyframe_separation;
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
   std::string ground_truth_file;
   int timer_window_size;
   double server_query_spread;

   // debug
   bool debug_map;
   bool debug_feature_detection;
   int ceres_debug_level;
   int iterate_debug_level;
   int initialization_debug_level;
   int relocalizer_debug_level;
   int server_debug_level;

   private:
   FrontEndConfig(){
     collect_pose_analytics = false;
     use_imu_for_gn = false;
     do_adaptive_window = true;
     do_dense_init = false;
     do_bundle_adjustment = true;
     do_async_bundle_adjustment = true;
     use_inverse_depth_parameterization = false;
     do_keyframing = true;
     do_relocalization = true;
     relocalizer_match_threshold = 50;
     relocalizer_min_keyframe_separation = 20;
     min_keyframes_for_initializing = 6;
     keyframe_search_depth = 10;
     async_ba_window_size = 30;
     ba_window_size = 15;
     ba_num_iter = 2;
     ba_num_iter_adaptive = 10;
     use_only_camera_id = -1;
     keyframe_threshold = 0.5;
     keyframe_max_distance = 0.5;
     keyframe_max_angle = 10.0;
     init_min_keyframes = 2;
     init_min_disparity = 10.0;
     init_max_distortion = 0.2;
     init_min_pctg_init_landmarks = 0.5;
     inlier_threshold = 1.4;
     server_upload_map_size = 50;
     server_download_map_size = 2000;
     ground_truth_file = "";
     timer_window_size = 40;
     server_query_spread = 15.0;
     debug_map = false;
     debug_feature_detection = false;
     ceres_debug_level = 1;
     iterate_debug_level = 1;
     initialization_debug_level = 1;
     relocalizer_debug_level = 1;
     server_debug_level=1;

   };
   static FrontEndConfig * m_configInstance;
};

#endif
