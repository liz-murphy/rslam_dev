#ifndef COMMON_FRONT_END_CONFIG_H
#define COMMON_FRONT_END_CONFIG_H

#include <common_front_end/CommonFrontEndParamsConfig.h>
#include <string>

class CommonFrontEndConfig
{
  public:
    static CommonFrontEndConfig * getConfig();
    int initial_search_radius;
    double search_radius_grow_rate;
    int num_match_in_time_attempts;
    int num_features_to_track;
    int feature_matching_threshold;
    double match_error_threshold;
    double match_error_factor;
    int max_features_in_cell;
    int num_quadtree_levels;
    bool do_subpixel_refinement;
    double esm_threshold;
    double esm_subpixel_threshold;
    bool esm_use_search_roi;
    bool enforce_epipolar_geometry;
    bool do_jealous_matching;
    bool do_marginalization;
    double dense_align_threshold;
    int relocalizer_hold_depth;
    int tracker_hold_depth;
    int ransac_debug_level;
    int ransac_seed;
    int ransac_max_trials;
    int ransac_min_inlier_percentage;
    double ransac_outlier_threshold;
    double ransac_probability;
    bool use_feature_buckets;
    int feature_detector;
    int feature_descriptor;
    bool is_rotation_invariant;
    int fast_threshold;
    int fast_levels;
    int fast_level_factor;
    bool fast_skip_level0;
    int brisk_threshold;
    int brisk_octaves;
    bool brisk_refine_3d;
    double freak_pattern_scale;
    int freak_octaves;
    double dog_threshold;
    double dog_edge_threshold;
    int dog_octaves;
    double dog_sigma;
    double fast_harris_score_threshold;
    std::string peanut_name;
  private:
    CommonFrontEndConfig(){
      initial_search_radius=20;
      search_radius_grow_rate=1.2;
      num_match_in_time_attempts=3;
      num_features_to_track=128;
      feature_matching_threshold=100;
      match_error_threshold=30.0;
      match_error_factor=1.2;
      max_features_in_cell=20;
      num_quadtree_levels=5;
      do_subpixel_refinement=true;
      esm_threshold=18.0;
      esm_subpixel_threshold=1.2;
      esm_use_search_roi=false;
      enforce_epipolar_geometry=false;
      do_jealous_matching=false;
      do_marginalization=false;
      dense_align_threshold=35.0;
      relocalizer_hold_depth=100;
      tracker_hold_depth=50;
      ransac_debug_level=1;
      ransac_seed=1;
      ransac_max_trials=500;
      ransac_min_inlier_percentage=0.5;
      ransac_outlier_threshold=1.2;
      ransac_probability=0.99;
      use_feature_buckets=true;
      feature_detector=common_front_end::CommonFrontEndParams_FAST;
      feature_descriptor=common_front_end::CommonFrontEndParams_FREAK;
      is_rotation_invariant=false;
      fast_threshold=20;
      fast_levels=4;
      fast_level_factor=4;
      fast_skip_level0=false;
      brisk_threshold=15;
      brisk_octaves=3;
      brisk_refine_3d=true;
      freak_pattern_scale=22.0;
      freak_octaves=3;
      dog_threshold=0.04;
      dog_edge_threshold=10.0;
      dog_octaves=3;
      dog_sigma=1.6;
      fast_harris_score_threshold=10000;
      peanut_name = "jimmy";

    };
    static CommonFrontEndConfig * m_configInstance;
};
#endif
