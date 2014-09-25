// Place to store formally global rslam parameters
#ifndef FRONT_END_CONFIG_H
#define FRONT_END_CONFIG_H

class FrontEndConfig 
{
public:
   static FrontEndConfig * getConfig()
   {
     if(!m_configInstance)
       m_configInstance = new FrontEndConfig;
     return m_configInstance;
   };
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
     m_configInstance->collect_pose_analytics = false;
     m_configInstance->use_imu_for_gn = false;
     m_configInstance->do_adaptive_window = true;
     m_configInstance->do_dense_init = false;
     m_configInstance->do_bundle_adjustment = true;
     m_configInstance->do_async_bundle_adjustment = true;
     m_configInstance->use_inverse_depth_parameterization = false;
     m_configInstance->do_keyframing = true;
     m_configInstance->do_relocalization = true;
     m_configInstance->relocalizer_match_threshold = 50;
     m_configInstance->relocalizer_min_keyframe_separation = 20;
     m_configInstance->min_keyframes_for_initializing = 6;
     m_configInstance->keyframe_search_depth = 10;
     m_configInstance->async_ba_window_size = 30;
     m_configInstance->ba_window_size = 15;
     m_configInstance->ba_num_iter = 2;
     m_configInstance->ba_num_iter_adaptive = 10;
     m_configInstance->use_only_camera_id = -1;
     m_configInstance->keyframe_threshold = 0.5;
     m_configInstance->keyframe_max_distance = 0.5;
     m_configInstance->keyframe_max_angle = 10.0;
     m_configInstance->init_min_keyframes = 2;
     m_configInstance->init_min_disparity = 10.0;
     m_configInstance->init_max_distortion = 0.2;
     m_configInstance->init_min_pctg_init_landmarks = 0.5;
     m_configInstance->inlier_threshold = 1.4;
     m_configInstance->server_upload_map_size = 50;
     m_configInstance->server_download_map_size = 2000;
     m_configInstance->ground_truth_file = "";
     m_configInstance->timer_window_size = 40;
     m_configInstance->server_query_spread = 15.0;
     m_configInstance->debug_map = false;
     m_configInstance->debug_feature_detection = false;
     m_configInstance->ceres_debug_level = 1;
     m_configInstance->iterate_debug_level = 1;
     m_configInstance->initialization_debug_level = 1;
     m_configInstance->relocalizer_debug_level = 1;
     m_configInstance->server_debug_level=1;

   };
   static FrontEndConfig * m_configInstance;
};

#endif
