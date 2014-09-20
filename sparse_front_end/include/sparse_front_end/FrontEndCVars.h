// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef _FRONT_END_CVARS_H_
#define _FRONT_END_CVARS_H_

#include <Utils/CVarHelpers.h>
#include <CVars/CVar.h>

class FrontEndCVars
{
public:
  FrontEndCVars() :
    // FRONT END - GENERAL OPTIONS
    collect_pose_analytics(
      CVarUtils::CreateCVar<>(
          "analytics.PoseAnalytics", false, "")),

    use_imu_for_gn(
      CVarUtils::CreateCVar<>(
          "frontend.UseImuForGn", false,
          "Use the imu for position estimates before BA.")),

    do_adaptive_window(
      CVarUtils::CreateCVar<>(
          "frontend.DoAdaptiveWindow", true,
          "Do BA of pose and landmarks" ) ),

    do_dense_init(
      CVarUtils::CreateCVar<>(
        "frontend.DoDenseInit", false,
        "Compute initial rotation with full-image alighment." ) ),

    do_bundle_adjustment(
      CVarUtils::CreateCVar<>(
        "frontend.DoBundleAdjutment", true,
        "Do BA of pose and landmarks" ) ),

    do_async_bundle_adjustment(
        CVarUtils::CreateCVar<>(
            "frontend.DoAsyncBundleAdjutment", true,
            "Do BA of pose and landmarks" ) ),

    use_inverse_depth_parameterization(
      CVarUtils::CreateCVar<>(
        "frontend.UseInverseDepthParam", false,
        "Parameterize landmarks using inverse depth " ) ),

    do_keyframing(
      CVarUtils::CreateCVar<>(
          "frontend.DoKeyframing", true,
          "Use keyframing " ) ),

    do_relocalization(
      CVarUtils::CreateCVar<>(
        "relocalizer.Active", true,
        "Use relocalization thread " ) ),

    relocalizer_match_threshold(
      CVarUtils::CreateCVar<>(
        "relocalizer.MatchThreshold", 50u,
        "Use relocalization thread " ) ),

    relocalizer_min_keyframe_separation(
      CVarUtils::CreateCVar<>(
        "relocalizer.MinKeyframeSeparation", 20u,
        "Use relocalization thread " ) ),

    min_keyframes_for_initializing(
      CVarUtils::CreateCVar<>(
        "frontend.MinKeyframesForInitializing", 6u,
        "Use relocalization thread " ) ),

    keyframe_search_depth(
      CVarUtils::CreateCVar<>(
        "frontend.KeyframeSearchDepth", 10u,
        "Search region for keyframes" ) ),

    async_ba_window_size(
      CVarUtils::CreateCVar<>(
        "frontend.AsyncBAWindowSize", 30u,
        "Number of previous frames considered for "
        "the asynchronous local pose and landmark refinement." ) ),

    ba_window_size(
      CVarUtils::CreateCVar<>(
          "frontend.BAWindowSize", 15u,
        "Number of previous frames considered for "
        "the local pose and landmark refinement." ) ),

    ba_num_iter(
        CVarUtils::CreateCVar<>(
            "frontend.BundleAdjustmentNumIter", 2,
            "Number of landmark-pose refinement alternations." ) ),

    ba_num_iter_adaptive(
        CVarUtils::CreateCVar<>(
            "frontend.BundleAdjustmentNumIterAdaptive", 10,
            "Number of BA iterations in adaptive thread.")),


    use_only_camera_id(
      CVarUtils::CreateCVar<>(
        "frontend.UseOnlyCameraId", -1,
        "If different than -1, only the camera "
        "with the id provided will be used." ) ),

    keyframe_threshold(
      CVarUtils::CreateCVar<>(
        "frontend.keyframe.Threshold", 0.5,
        "Min percentage of traked landmarks, "
        "used to decide when to drop a keyframe " ) ),

    keyframe_max_distance(
      CVarUtils::CreateCVar<>(
        "frontend.keyframe.MaxDistance", 0.5,
        "Max XYZ distance between, frames used "
        "to decide when to drop a keyframe " ) ),

    keyframe_max_angle(
      CVarUtils::CreateCVar<>(
        "frontend.keyframe.MaxAngle", 10.0,
        "Max angle between frames (in degrees), "
        "used to decide when to drop a keyframe " ) ),

    // MONO INIT

    init_min_keyframes(
      CVarUtils::CreateCVar<>(
        "init.MinKeyframes", 2u,
        "Minimum number of keyframes for initialization. ") ),

    init_min_disparity(
      CVarUtils::CreateCVar<>(
        "init.MinAvgDisparity", 10.0,
        "Minimum average distance between 2d correspondences "
        "necessary to use Essential matrix for pose estimation." ) ),

    init_max_distortion(
      CVarUtils::CreateCVar<>(
        "init.MaxDistortion", 0.2,
        "Maximum camera distortion allowed in order to "
        "use Essential matrix for pose estimation." ) ),

    init_min_pctg_init_landmarks(
      CVarUtils::CreateCVar<>(
        "init.MinPctgInitializedLandmarks", 0.5,
        "Criteria for ending initialization. Minimum percentage of "
        "initialized landmarks before." ) ),

    // TRACKER
    inlier_threshold(
      CVarUtils::CreateCVar<>(
        "tracker.InlierThreshold", 1.4, "Acceptable inlier threshold." ) ),

    server_upload_map_size(
        CVarUtils::CreateCVar<>(
            "server.upload_map_size", 50u,
            "Size of map chunks to be uploaded")),

    server_download_map_size(
        CVarUtils::CreateCVar<>(
            "server.download_map_size", 2000u,
            "Size of map chunks to be downloaded")),

    // GUI
    ground_truth_file(
      CVarUtils::CreateCVar<>(
        "gui.GroundTruthFile", std::string(""),
        "File containing the ground truth poses (relative poses)." ) ),

    timer_window_size(
      CVarUtils::CreateCVar<>(
        "gui.TimerWindowSize", 40, "Timer time window size" ) ),

    server_query_spread(
        CVarUtils::CreateCVar<>(
            "server.query_spread", 15.0,
            "Minimum time between server queries" )),

    // DEBUG
    debug_map(
      CVarUtils::CreateCVar<>(  "debug.Map", false, "Print map")),

    debug_feature_detection(
      CVarUtils::CreateCVar<>( "debug.FeatureDetection", false, "")),

    ceres_debug_level(
      CVarUtils::CreateCVar<>( "debug.CERES", 1, "Print error level")),

    iterate_debug_level(
        CVarUtils::CreateCVar<>( "debug.Iterate", 1, "Print error level")),

    initialization_debug_level(
        CVarUtils::CreateCVar<>( "debug.Initialization", 1, "Print error level")),

    relocalizer_debug_level(
      CVarUtils::CreateCVar<>(
          "debug.Relocalization", 1, "Print error level") ),

    server_debug_level(
        CVarUtils::CreateCVar<>(
            "debug.FrontendServer", 1, "Print error level") )
  {}

  bool&                 collect_pose_analytics;
  bool&                 use_imu_for_gn;
  bool&                 do_adaptive_window;
  bool&                 do_dense_init;
  bool&                 do_bundle_adjustment;
  bool&                 do_async_bundle_adjustment;
  bool&                 use_inverse_depth_parameterization;
  bool&                 do_keyframing;
  bool&                 do_relocalization;
  unsigned int&         relocalizer_match_threshold;
  unsigned int&         relocalizer_min_keyframe_separation;
  unsigned int&         min_keyframes_for_initializing;
  unsigned int&         keyframe_search_depth;
  unsigned int&         async_ba_window_size;
  unsigned int&         ba_window_size;
  int&                  ba_num_iter;
  int&                  ba_num_iter_adaptive;
  int&                  use_only_camera_id;
  double&               keyframe_threshold;
  double&               keyframe_max_distance;
  double&               keyframe_max_angle;

  unsigned int&         init_min_keyframes;
  double&               init_min_disparity;
  double&               init_max_distortion;
  double&               init_min_pctg_init_landmarks;

  double&               inlier_threshold;
  unsigned int&         server_upload_map_size;
  unsigned int&         server_download_map_size;

  std::string&	        ground_truth_file;
  int&                  timer_window_size;
  double&               server_query_spread;

  bool&                 debug_map;
  bool&                 debug_feature_detection;
  int&         ceres_debug_level;
  int&         iterate_debug_level;
  int&         initialization_debug_level;
  int&         relocalizer_debug_level;
  int&         server_debug_level;
};

extern FrontEndCVars g_frontend_cvars;

#endif
