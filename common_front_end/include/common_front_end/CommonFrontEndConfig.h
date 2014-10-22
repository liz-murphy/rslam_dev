#ifndef COMMON_FRONT_END_CONFIG_H
#define COMMON_FRONT_END_CONFIG_H

#include <common_front_end/CommonFrontEndParamsConfig.h>
#include <common_front_end/FREAKConfig.h>
#include <common_front_end/SURFConfig.h>
#include <common_front_end/FASTConfig.h>
#include <string>

class CommonFrontEndConfig
{
  public:

    std::string getFeatureDetectorStr()
    {
      switch(feature_detector_)
      {
        case common_front_end::CommonFrontEndParams_DOG:
          return "SURF";
        case common_front_end::CommonFrontEndParams_FAST:
          return "FAST";
        default:
          return "OOPS";
      }
    }

    static CommonFrontEndConfig * getConfig()
    {
      if(!m_configInstance)
        m_configInstance = new CommonFrontEndConfig;
      return m_configInstance;
    }

    void configFASTCallback(common_front_end::FASTConfig &config, uint32_t level)
    {
      fast_threshold_ = config.fast_threshold;
      fast_do_nms_ = config.do_nms;
      fast_skip_level0_ = config.skip_level0;
      reset_required_ = true; // any changes to features need a reset
    }

    void configFREAKCallback(common_front_end::FREAKConfig &config, uint32_t level)
    { 
      freak_orientation_normalized_ = config.orientation_normalized;
      freak_scale_normalized_ = config.scale_normalized;
      freak_pattern_scale_ = config.pattern_scale;
      freak_n_octaves_ = config.n_octaves;  

      // If these change you need to reset tracking ...
      reset_required_ = true;
    }

    void configSURFCallback(common_front_end::SURFConfig &config, uint32_t level)
    {
      surf_hessian_threshold_ = config.hessian_threshold;
      surf_n_octaves_ = config.n_octaves;
      surf_n_octave_layers_ = config.n_octave_layers;
      surf_extended_ = config.extended;
      surf_upright_ = config.upright; 
      reset_required_ = true;
    }

    void configCallback(common_front_end::CommonFrontEndParamsConfig &config, uint32_t level)
    {
      initial_search_radius_ = config.initial_search_radius;
      search_radius_grow_rate_ = config.search_radius_grow_rate;
      num_match_in_time_attempts_ = config.num_match_in_time_attempts;
      num_features_to_track_ = config.num_features_to_track;
      feature_matching_threshold_ = config.feature_matching_threshold;
      match_error_threshold_ = config.match_error_threshold;
      match_error_factor_ = config.match_error_factor;
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

      if(feature_detector_ != config.feature_detector)
        reset_required_ = true;

      if(feature_descriptor_ != config.feature_descriptor)
        reset_required_ = true;

      feature_detector_ = config.feature_detector;
      feature_descriptor_ = config.feature_descriptor;

      pyramid_levels_ = config.pyramid_levels;
      pyramid_level_factor_ = config.pyramid_level_factor;
    }
  
    int getInitialSearchRadius(){return initial_search_radius_;};
    double getSearchRadiusGrowRate(){return search_radius_grow_rate_;};
    int getNumFeaturesToTrack(){return num_features_to_track_;}; 
    int getFeatureMatchingThreshold(){return feature_matching_threshold_;}; 
    double getMatchErrorThreshold(){return match_error_threshold_;};
    double getMatchErrorFactor(){return match_error_factor_;};
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
    int getFeatureDetector(){return feature_detector_;}; 
    int getFeatureDescriptor(){return feature_descriptor_;}; 

    int getPyramidLevels(){return pyramid_levels_;};
    double getPyramidLevelFactor(){return pyramid_level_factor_;};

    void setNumQuadTreeLevels(int levels){num_quadtree_levels_ = levels;};
    void setSubPixelRefinement(bool val){do_subpixel_refinement_ = val;};
    void setUseFeatureBuckets(bool val){use_feature_buckets_=val;};
    bool useFeatureBuckets(){return use_feature_buckets_;};

    double getDenseAlignThreshold(){return dense_align_threshold_;};
    double getSurfHessianThreshold(){return surf_hessian_threshold_;};
    int getSurfNOctaves(){return surf_n_octaves_;};
    int getSurfNOctaveLayers(){return surf_n_octave_layers_;};
    bool surfExtended(){return surf_extended_;};
    bool surfUpright(){return surf_upright_;};

    bool freakOrientationNormalized(){return freak_orientation_normalized_;};
    bool freakScaleNormalized(){return freak_scale_normalized_;};
    float getFreakPatternScale(){return freak_pattern_scale_;};
    int getFreakNOctaves(){return freak_n_octaves_;};

    int getFastThreshold(){return fast_threshold_;};
    bool fastDoNonMaxSuppression(){return fast_do_nms_;};
    bool fastSkipLevel0(){return fast_skip_level0_;};

    int getNumMatchInTimeAttempts(){return num_match_in_time_attempts_;};

    // have any of the parameters that have been modified caused a need to restart the front end
    bool resetRequired(){return reset_required_;};
    void resetDone(){reset_required_ = false;};

  private:
    bool reset_required_;
    int initial_search_radius_;
    double search_radius_grow_rate_;

    int num_features_to_track_;
    int feature_matching_threshold_;
    double match_error_threshold_;
    double match_error_factor_;
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
    int feature_detector_;
    int feature_descriptor_;

    int pyramid_levels_;
    double pyramid_level_factor_;

    bool is_rotation_invariant_;

    // FAST feature detection
    int fast_threshold_;
    bool fast_do_nms_;
    bool fast_skip_level0_;
    //int fast_levels_;
    //double fast_level_factor_;

    /*int brisk_threshold_;
    int brisk_octaves_;
    bool brisk_refine_3d_;
*/

    // DOG/SURF detection
    double dog_threshold_;
    double dog_edge_threshold_;
    int dog_octaves_;
    double dog_sigma_;

    double fast_harris_score_threshold_;
  
    int num_match_in_time_attempts_;

    // freak descriptors
    bool freak_orientation_normalized_;
    bool freak_scale_normalized_;
    float freak_pattern_scale_;
    int freak_n_octaves_;  
    
    // surf descriptors
    double surf_hessian_threshold_;
    int surf_n_octaves_;
    int surf_n_octave_layers_;
    bool surf_extended_;
    bool surf_upright_;

    CommonFrontEndConfig(){
      initial_search_radius_=20;
      search_radius_grow_rate_=1.2;
      num_match_in_time_attempts_=3;
      num_features_to_track_=128;
      feature_matching_threshold_=100;
      match_error_threshold_=30.0;
      match_error_factor_=1.2;
      max_features_in_cell_=20;
      num_quadtree_levels_=5;
      do_subpixel_refinement_=true;
      esm_threshold_=18.0;
      esm_subpixel_threshold_=1.2;
      esm_use_search_roi_=false;
      enforce_epipolar_geometry_=false;
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
      feature_detector_=common_front_end::CommonFrontEndParams_FAST;
      feature_descriptor_=common_front_end::CommonFrontEndParams_PATCH;
      is_rotation_invariant_=false;
      //fast_threshold_=20;
      //fast_levels_=4;
      //fast_level_factor_=0.8;
      /*brisk_threshold_=15;
      brisk_octaves_=3;
      brisk_refine_3d_=true;*/
      
      dog_threshold_=0.04;
      dog_edge_threshold_=10.0;
      dog_octaves_=3;
      dog_sigma_=1.6;
      fast_harris_score_threshold_=10000;

      surf_hessian_threshold_ = 400.0;
      surf_n_octaves_ = 4;
      surf_n_octave_layers_ = 2;
      surf_extended_ = true;
      surf_upright_ = false;

      freak_orientation_normalized_ = true;
      freak_scale_normalized_ = true;
      freak_pattern_scale_ = 22.0;
      freak_n_octaves_ = 4;  
    
      fast_threshold_ = 20;
      fast_do_nms_ =true;
      fast_skip_level0_=false;

      pyramid_levels_ = 4;
      pyramid_level_factor_ = 0.8;
    };
    static CommonFrontEndConfig * m_configInstance;
};
#endif


