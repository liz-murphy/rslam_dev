// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <utils/CVarHelpers.h>
#include <CVars/CVar.h>
#include <string>

struct CommonFrontEndCVars {
 public:
  CommonFrontEndCVars() :
      // FRONT END - GENERAL OPTIONS
      // TRACKER options
      initial_search_radius(
          CVarUtils::CreateCVar<>(
              "tracker.SearchRadius", 20,
              "Radius of window size to search in.")),

      search_radius_grow_rate(
          CVarUtils::CreateCVar<>(
              "tracker.SearchRadiusGrowRate", 1.2f,
              "Rate at which search radius increases with each "
              "unsuccessful attempt at matching.")),

      num_match_in_time_attempts(
          CVarUtils::CreateCVar<>(
              "tracker.MatchInTimeAttempts", 3,
              "Number of times match in time should be attempted")),

#if _BUILD_OVV
      num_features_to_track(
          CVarUtils::CreateCVar<>(
              "tracker.NumFeaturesToTrack", 128u,
              "Desired number of features to track.")),
#else
      num_features_to_track(
          CVarUtils::CreateCVar<>(
              "tracker.NumFeaturesToTrack", 128u,
              "Desired number of features to track.")),
#endif

    feature_matching_threshold(
        CVarUtils::CreateCVar<>(
            "tracker.FeatureMatchingThreshold", 100,
            "Max Hamming distance to consider a match "
            "(FREAK and BRISK descriptors)")),

    match_error_threshold(
        CVarUtils::CreateCVar<>(
            "tracker.MatchErrorThreshold", 30.0f,
            "Pixel error. SAD score given by this value "
            "times PatchWidth * PatchHeight.")),

    match_error_factor(
        CVarUtils::CreateCVar<>(
            "tracker.MatchErrorFactor", 1.2f,
            "Factor by which a match is selected. "
            "Best:SecondBest errors are compared.")),

    max_features_in_cell(
        CVarUtils::CreateCVar<>(
            "features.MaxFeaturesInCell", 20u,
            "Max number of features allowed in each cell.")),

    num_quadtree_levels(
        CVarUtils::CreateCVar<>(
            "tracker.NumQuadtreeLevels", 5,
            "Desired number of QuadTree levels.")),

    do_subpixel_refinement(
        CVarUtils::CreateCVar<>(
            "tracker.DoSubPixelTracking", true, "")),

    esm_threshold(
        CVarUtils::CreateCVar<>(
            "tracker.ESMThreshold", 18.0,
            "Max ESM error to consider a valid match.")),

    esm_subpixel_threshold(
        CVarUtils::CreateCVar<>(
            "tracker.ESMSubPixelThreshold", 1.2,
            "Max ESM correction to consider a valid match.")),

    esm_use_search_roi(
        CVarUtils::CreateCVar<>(
            "tracker.ESMUseSearchROI", false,
            "Extract warped search region for ESM")),

    enforce_epipolar_geometry(
        CVarUtils::CreateCVar<>(
            "tracker.EnforceEpipolarGeometry", false,
            "Make sure landmark projects on an epipolar line.")),

    do_jealous_matching(
        CVarUtils::CreateCVar<>(
            "frontend.DoJealousMatching", false,
            "Make sure StartNewLandmarks does jealous matching.")),

    do_marginalization(
        CVarUtils::CreateCVar<>(
            "frontend.DoMarginalization", false,
            "Marginalize instead of condition.")),

    dense_align_threshold(
        CVarUtils::CreateCVar<>(
            "frontend.DenseAlignThreshold", 35.0,
            "Image difference threshold for dense alighment.")),

    relocalizer_hold_depth(
        CVarUtils::CreateCVar<>(
            "relocalizer.FrameHoldDepth", 100,
            "Depth of search for holding frames in memory for relocalizer.")),

    tracker_hold_depth(
        CVarUtils::CreateCVar<>(
            "tracker.FrameHoldDepth", 100,
            "Depth of search for holding frames in memory for tracking.")),

    // RANSAC
    ransac_debug_level(
        CVarUtils::CreateCVar<>( "debug.RANSAC", 1, "Print error level")),

    ransac_seed(
        CVarUtils::CreateCVar<>(
            "ransac.Seed", 1u, "RANSAC random seed." ) ),

    ransac_max_trials(
        CVarUtils::CreateCVar<>(
            "ransac.MaxTrials", 500u,
            "RANSAC's Max number of attempts to find a good enough model." ) ),

    ransac_min_inlier_percentage(
        CVarUtils::CreateCVar<>(
            "ransac.MinInlierPercentage", 0.5f,
            "RANSAC's Min number of inlier percentage for success." ) ),

    ransac_outlier_threshold(
        CVarUtils::CreateCVar<>(
            "ransac.OutlierThreshold", 1.2f, "RANSAC outlier threshold." ) ),

    ransac_probability(
        CVarUtils::CreateCVar<>(
            "ransac.Prob", 0.99, "RANSAC probability limit for exit." ) ),

    // FEATURES
#if _BUILD_OVV
    use_feature_buckets(
        CVarUtils::CreateCVar<>(
            "features.UseBucketting", false,
            "Use grid to distribute feature detection evenly "
            "across the images.")),
#else
    use_feature_buckets(
        CVarUtils::CreateCVar<>(
            "features.UseBucketting", true,
            "Use grid to distribute feature detection evenly "
            "across the images.")),
#endif

#if _BUILD_OVV
    feature_detector(
        CVarUtils::CreateCVar<>(
            "features.DetectorType", std::string("DOG"),"FAST,TRACK_2D,DOG")),

    feature_descriptor(
        CVarUtils::CreateCVar<>(
            "features.DescriptorType", std::string("SURF"), "FREAK,PATCH,SURF")),   // Liz purposes only
#else
    feature_detector(
        CVarUtils::CreateCVar<>(
            "features.DetectorType",

#ifdef ANDROID
            std::string("FLYBY"),
#else
            std::string("FAST"),
#endif  // ANDROID

            "FAST,TRACK_2D,FLYBY")),

    feature_descriptor(
        CVarUtils::CreateCVar<>(
            "features.DescriptorType", std::string("PATCH"), "FREAK,PATCH,SURF")),
#endif

    is_rotation_invariant(
        CVarUtils::CreateCVar<>(
            "features.RotationInvariant", false, "FREAK,BRISK")),

    fast_threshold(
        CVarUtils::CreateCVar<>(
            "features.fast.Threshold", 20,
            "FAST feature detector threshold.")),

    fast_Levels(
        CVarUtils::CreateCVar<>(
            "features.fast.NumLevels", 4, "FAST num pyramid levels.")),

    fast_level_factor(
        CVarUtils::CreateCVar<>(
            "features.fast.LevelFactor", 0.8,
            "FAST scale factor between pyramid levels")),

    fast_skip_level0(
        CVarUtils::CreateCVar<>(
            "features.fast.SkipLevel0",
#ifdef ANDROID
            true,
#else  // ANDROID
            false,
#endif  // ANDROID
            "True if level 0 of pyramid should not detect features")),

    brisk_threshold(
        CVarUtils::CreateCVar<>(
            "features.brisk.Threshold", 15,
            "BRISK->AGAST feature detector threshold.")),

    brisk_octaves(
        CVarUtils::CreateCVar<>(
            "features.brisk.Octaves", 3,
            "BRISK->AGAST feature detector octaves.")),

    brisk_refine_3d(
        CVarUtils::CreateCVar<>(
            "features.brisk.Do3dRef", true,
            "When active the keypoint subpixel location and "
            "nms will be computed in scale-space")),

    freak_pattern_scale(
        CVarUtils::CreateCVar<>(
            "features.freak.PatterScale", 22.0f, "FREAK parttern scale")),

    freak_octaves(
        CVarUtils::CreateCVar<>(
            "features.freak.Octaves", 3u, "FREAK descriptor octaves.")),

    dog_threshold(
        CVarUtils::CreateCVar<>(
            "features.dog.Threshold", 0.04f, "DOG threshold")),

    dog_edge_threshold(
        CVarUtils::CreateCVar<>(
            "features.dog.EdgeThreshold", 10.0f, "DOG edge threshold")),

    dog_octaves(
        CVarUtils::CreateCVar<>(
            "features.dog.Octaves", 3u, "DOG octaves")),

    dog_sigma(
        CVarUtils::CreateCVar<>(
            "features.dog.Sigma", 1.6f, "DOG sigma")),

    flyby_num_features(CVarUtils::CreateCVar<>(
        "features.flyby.num_features", 100,
        "Number of features for FlyBy detector to find")),

    flyby_harris_threshold(CVarUtils::CreateCVar<>(
        "features.flyby.harris_threshold", 20000000.0f,
        "Harris corner minimum strength threshold")),
    flyby_max_disparity_fraction(CVarUtils::CreateCVar<>(
        "features.flyby.max_disparity_fraction", 0.02f,
        "Maximum fraction of the image for disparity between points")),

    peanut_name(CVarUtils::CreateCVar<>("peanut name", std::string(""),
                                        "Peanut name to set IMU bias"))
  {}

  int&                  initial_search_radius;
  float&                search_radius_grow_rate;
  int&                  num_match_in_time_attempts;
  unsigned int&         num_features_to_track;
  int&                  feature_matching_threshold;
  float&                match_error_threshold;
  float&                match_error_factor;
  unsigned int&         max_features_in_cell;
  int&                  num_quadtree_levels;
  bool&                 do_subpixel_refinement;
  double&               esm_threshold;
  double&               esm_subpixel_threshold;
  bool&                 esm_use_search_roi;

  bool&                 enforce_epipolar_geometry;
  bool&                 do_jealous_matching;
  bool&                 do_marginalization;
  double&               dense_align_threshold;
  int&                  relocalizer_hold_depth;
  int&                  tracker_hold_depth;

  int&                  ransac_debug_level;
  unsigned int&         ransac_seed;
  unsigned int&         ransac_max_trials;
  float&                ransac_min_inlier_percentage;
  float&                ransac_outlier_threshold;
  double&               ransac_probability;

  bool&                 use_feature_buckets;
  std::string&          feature_detector;
  std::string&          feature_descriptor;
  bool&                 is_rotation_invariant;
  int&                  fast_threshold;
  int&                  fast_Levels;
  double&               fast_level_factor;
  bool&                 fast_skip_level0;

  int&                  brisk_threshold;
  int&                  brisk_octaves;
  bool&                 brisk_refine_3d;
  float&                freak_pattern_scale;
  unsigned int&         freak_octaves;
  float&                dog_threshold;
  float&                dog_edge_threshold;
  unsigned int&         dog_octaves;
  float&                dog_sigma;

  int&   flyby_num_features;
  float& flyby_harris_threshold;
  float& flyby_max_disparity_fraction;

  std::string& peanut_name;
};

extern CommonFrontEndCVars g_common_cvars;
