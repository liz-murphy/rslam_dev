#ifndef TRACKING_CONFIG_H
#define TRACKING_CONFIG_H

// Place to store formally global rslam parameters

class TrackingConfig
{
public:
   static TrackingConfig * getConfig(){
     if(m_configInstance == 0)
       m_configInstance = new TrackingConfig;
     return m_configInstance;
   };
   bool                 do_ransac;
   bool                 do_rethreading;
   bool                 init_mono_landmarks;

   unsigned int         ref_cam_id;
   int                  min_tracked_features;
   unsigned int         gn_max_num_iter;
   unsigned int         matchintime_window_size;
   unsigned int         startnewlandmarks_debug_level;
   unsigned int         matchintime_debug_level;
   unsigned int         flagoutliers_debug_level;
   unsigned int         estimate_debug_level;

private:
   TrackingConfig(){
    m_configInstance->do_ransac = true;
    m_configInstance->do_rethreading = false;
    m_configInstance->init_mono_landmarks = true;
    m_configInstance->ref_cam_id = 0;
    m_configInstance->min_tracked_features = 15;
    m_configInstance->gn_max_num_iter = 10;
    m_configInstance->matchintime_window_size = 5;
    m_configInstance->startnewlandmarks_debug_level = 1;
    m_configInstance->matchintime_debug_level = 1;
    m_configInstance->flagoutliers_debug_level = 1;
    m_configInstance->estimate_debug_level = 1;
   };
   static TrackingConfig * m_configInstance;
};

#endif
