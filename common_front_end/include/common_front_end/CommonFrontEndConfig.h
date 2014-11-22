#ifndef COMMON_FRONT_END_CONFIG_H
#define COMMON_FRONT_END_CONFIG_H

#include <common_front_end/CommonFrontEndParamsConfig.h>
#include <string>

class CommonFrontEndConfig
{
  public:

    static CommonFrontEndConfig * getConfig()
    {
      if(!m_configInstance)
        m_configInstance = new CommonFrontEndConfig;
      return m_configInstance;
    }

    void configCallback(common_front_end::CommonFrontEndParamsConfig &config, uint32_t level)
    {
      initial_search_radius_ = config.initial_search_radius;
      search_radius_grow_rate_ = config.search_radius_grow_rate;
      num_match_in_time_attempts_ = config.num_match_in_time_attempts;
      num_features_to_track_ = config.num_features_to_track;
      max_features_in_cell_ = config.max_features_in_cell;
      num_quadtree_levels_ = config.num_quadtree_levels;
      do_subpixel_refinement_ = config.do_subpixel_refinement;
      esm_threshold_ = config.esm_threshold;
      esm_subpixel_threshold_ = config.esm_subpixel_threshold;
      esm_use_search_roi_ = config.esm_use_search_roi;

      do_jealous_matching_ = config.do_jealous_matching;
      do_marginalization_ =  config.do_marginalization;

      tracker_hold_depth_ = config.tracker_hold_depth; 

      ransac_seed_ = config.ransac_seed;
      ransac_max_trials_ = config.ransac_max_trials;
      ransac_outlier_threshold_ = config.ransac_outlier_threshold;
      ransac_probability_ = config.ransac_probability;
    
      use_feature_buckets_ = config.use_feature_buckets;

      timer_window_size_ = config.timer_window_size;

      do_keyframing_ = config.do_keyframing;
      do_bundle_adjustment_ = config.do_bundle_adjustment;
      do_async_bundle_adjustment_ = config.do_async_bundle_adjustment;
      do_adaptive_window_ = config.do_adaptive_window;
      async_ba_window_size_ = config.async_ba_window_size;
      ba_window_size_ = config.ba_window_size;
      ba_num_iter_adaptive_ = config.ba_num_iter_adaptive;
      ba_num_iter_ = config.ba_num_iter;
    }
  
    int getInitialSearchRadius(){return initial_search_radius_;};
    double getSearchRadiusGrowRate(){return search_radius_grow_rate_;};
    int getNumFeaturesToTrack(){return num_features_to_track_;}; 
    int getMaxFeaturesInCell(){return max_features_in_cell_;};
    int getNumQuadtreeLevels(){return num_quadtree_levels_;};
    bool doSubpixelRefinement(){return do_subpixel_refinement_;};
    double getEsmThreshold(){return esm_threshold_;};
    double getEsmSubpixelThreshold(){return esm_subpixel_threshold_;};
    bool esmUseSearchROI(){return esm_use_search_roi_;};
    bool doJealousMatching(){return do_jealous_matching_;};
    bool doMarginalization(){return do_marginalization_;};
    int getTrackerHoldDepth(){return tracker_hold_depth_;};
    int getRansacSeed(){return ransac_seed_;};
    int getRansacMaxTrials(){return ransac_max_trials_;};
    double getRansacOutlierThreshold(){return ransac_outlier_threshold_;};
    double getRansacProbability(){return ransac_probability_;};

    void setNumQuadTreeLevels(int levels){num_quadtree_levels_ = levels;};
    void setSubPixelRefinement(bool val){do_subpixel_refinement_ = val;};
    void setUseFeatureBuckets(bool val){use_feature_buckets_=val;};
    bool useFeatureBuckets(){return use_feature_buckets_;};

    double getDenseAlignThreshold(){return dense_align_threshold_;};
    
    int getNumMatchInTimeAttempts(){return num_match_in_time_attempts_;};

    int getTimerWindowSize(){return timer_window_size_;};
    bool doKeyframing(){return do_keyframing_;};
    bool doBundleAdjustment(){return do_bundle_adjustment_;};
    bool doAsyncBundleAdjustment(){return do_async_bundle_adjustment_;};
    bool doAdaptiveWindow(){return do_adaptive_window_;};
    int getAsyncBAWindowSize(){return async_ba_window_size_;};
    int getBAWindowSize(){return ba_window_size_;};
    int getBANumIterAdaptive(){return ba_num_iter_adaptive_;};
    int getBANumIter(){return ba_num_iter_;};

    // have any of the parameters that have been modified caused a need to restart the front end
    bool resetRequired(){return reset_required_;};
    void resetDone(){reset_required_ = false;};

  private:
    bool reset_required_;
    int initial_search_radius_;
    double search_radius_grow_rate_;

    int num_features_to_track_;
    int max_features_in_cell_;
    int num_quadtree_levels_;
    bool do_subpixel_refinement_;
    double esm_threshold_;
    double esm_subpixel_threshold_;
    bool esm_use_search_roi_;
    bool enforce_epipolar_geometry_;
    bool do_jealous_matching_;
    bool do_marginalization_;
    double dense_align_threshold_;
    int relocalizer_hold_depth_;
    int tracker_hold_depth_;
    int ransac_seed_;
    int ransac_max_trials_;
    int ransac_min_inlier_percentage_;
    double ransac_outlier_threshold_;
    double ransac_probability_;
    bool use_feature_buckets_;

    bool is_rotation_invariant_;

    double fast_harris_score_threshold_;
  
    int num_match_in_time_attempts_;

    // timer
    int timer_window_size_;

    bool do_keyframing_;
    bool do_bundle_adjustment_;

    bool do_async_bundle_adjustment_;
    bool do_adaptive_window_;
    int async_ba_window_size_;
    int ba_window_size_;
    int ba_num_iter_adaptive_;
    int ba_num_iter_;


    CommonFrontEndConfig(){
      initial_search_radius_=20;
      search_radius_grow_rate_=1.2;
      num_match_in_time_attempts_=3;
      num_features_to_track_=128;
      max_features_in_cell_=20;
      num_quadtree_levels_=5;
      do_subpixel_refinement_=true;
      esm_threshold_=18.0;
      esm_subpixel_threshold_=1.2;
      esm_use_search_roi_=false;
      enforce_epipolar_geometry_=true;
      do_jealous_matching_=false;
      do_marginalization_=false;
      dense_align_threshold_=35.0;
      relocalizer_hold_depth_=100;
      tracker_hold_depth_=50;
      ransac_seed_=1;
      ransac_max_trials_=500;
      ransac_min_inlier_percentage_=0.5;
      ransac_outlier_threshold_=1.2;
      ransac_probability_=0.99;
      use_feature_buckets_=true;
      is_rotation_invariant_=false;
     
     fast_harris_score_threshold_=10000;


      timer_window_size_ = 40;
    };
    static CommonFrontEndConfig * m_configInstance;
};
#endif


