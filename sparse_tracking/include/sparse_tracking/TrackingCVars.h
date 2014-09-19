// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <Utils/CVarHelpers.h>
#include <CVars/CVar.h>

struct TrackingCVars
{
  TrackingCVars() :
      do_ransac(CVarUtils::CreateCVar<>(
          "frontend.DoRANSAC", true,
          "Do RANSAC at each step.")),

      do_rethreading(CVarUtils::CreateCVar<>(
          "frontend.Rethreading", false,
          "Try to find missed matches after pose estimation ")),

      init_mono_landmarks(
          CVarUtils::CreateCVar<>(
              "frontend.InitMonoLandmarks", true,
              "Initialize landmarks from monocular features." ) ),

      ref_cam_id(
          CVarUtils::CreateCVar<>(
              "frontend.ReferenceCameraId", 0u,
              "The camera from which new landmarks are intitialized." ) ),

      min_tracked_features(CVarUtils::CreateCVar<>(
          "tracker.MinTrackedFeatures", 15,
          "Minimum number of features to track in a frame")),

      gn_max_num_iter(CVarUtils::CreateCVar<>(
          "frontend.GaussNewtonMaxNumIter", 10u,
          "Max number of Gauss-Newton iterations after RANSAC.")),

    matchintime_window_size(
        CVarUtils::CreateCVar<>(
            "tracker.MatchInTimeWindowSize",5u,
            "Number of frames to search back in time for landmarks." ) ),

    startnewlandmarks_debug_level(CVarUtils::CreateCVar<>(
          "debug.StartNewLandmarks", 1u, "Print error level")),

      matchintime_debug_level(CVarUtils::CreateCVar<>(
          "debug.MatchInTime", 1u, "Print error level")),

    flagoutliers_debug_level(CVarUtils::CreateCVar<>(
        "debug.FlagOutliers", 1u, "Print error level")),

    estimate_debug_level(CVarUtils::CreateCVar<>(
        "debug.EstimateRelativePose", 1u, "Print error level"))
  {}
  virtual ~TrackingCVars() {}

  bool&                 do_ransac;
  bool&                 do_rethreading;
  bool&                 init_mono_landmarks;

  unsigned int&         ref_cam_id;
  int&                  min_tracked_features;
  unsigned int&         gn_max_num_iter;
  unsigned int&         matchintime_window_size;

  unsigned int&         startnewlandmarks_debug_level;
  unsigned int&         matchintime_debug_level;
  unsigned int&         flagoutliers_debug_level;
  unsigned int&         estimate_debug_level;
};

extern TrackingCVars g_tracking_cvars;
