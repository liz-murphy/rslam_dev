#pragma once
#include <vector>

namespace sdtrack
{
  struct KeypointOptions
  {
    KeypointOptions()  {}
    int max_num_features = 300;

    int fast_threshold = 10;
    bool fast_nonmax_suppression = true;

    double gftt_absolute_strength_threshold = 0.5;
    int gftt_feature_block_size = 9;
    int gftt_min_distance_between_features = 4;
    bool gftt_use_harris = true;


  };

  struct OptimizationStats
  {
    double pre_solve_error;
    double delta_pose_norm;
    double delta_lm_norm;
    double transfer_time;
    double jacobian_time;
    double schur_time;
    double solve_time;
    double lm_time;
  };

  struct OptimizationOptions
  {
    bool transfer_patches = true;
    bool optimize_landmarks = true;
    bool optimize_pose = true;
  };

  struct DescriptorOptions
  {
    DescriptorOptions() {}

    bool  freak_is_rotation_invariant = true;
    bool  freak_is_scale_invariant = true;
    int   freak_num_octaves = 3;
    float freak_pattern_scale = 22.0f;

    float surf_fHessianThreshold = 400;
    int   surf_nOctaves = 4;
    int   surf_nOctaveLayers = 22;
    bool  surf_bExtended = false;
    bool  surf_bUpright = true;

    std::vector<int> freak_selected_pairs;
  };

  struct TrackerOptions
  {
    enum DetectorType
    {
      Detector_FAST = 1,
      Detector_SURF = 2,
      Detector_GFTT = 3,
    };

    enum DescriptorType
    {
      Descriptor_FREAK = 1,
      Descriptor_SURF = 2
    };

    TrackerOptions() {}

    uint32_t search_window_width = 20;
    uint32_t search_window_height = 20;
    DetectorType detector_type = Detector_SURF;
    DescriptorType descriptor_type = Descriptor_SURF;

    uint32_t pyramid_levels = 4;
    uint32_t patch_dim = 9;
    uint32_t num_active_tracks = 10;
    uint32_t iteration_exponent = 2;
    double center_weight = 100;
    bool use_closest_track_to_seed_rho = true;
    bool use_robust_norm_ = false;
    double gn_scaling = 1.0;
    double robust_norm_threshold_ = 5;
    double default_ray_depth = 1.0;
    double default_rho = 1.0;
    double dense_rmse_threshold = 15;
    double dense_ncc_threshold = 0.75;
    double harris_score_threshold = 10000;
    bool do_corner_subpixel_refinement = false;
    uint32_t feature_cells = 8;
  };
}
