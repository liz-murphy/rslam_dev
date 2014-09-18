#pragma once

#include <stdint.h>
#include <random>

namespace rslam {
namespace common {
enum eTrackingState {
  eTrackingGood = 1,
  eTrackingPoor = 2,
  eTrackingBad  = 4,
  eTrackingFail = 8
};

struct SystemStatus {
  SystemStatus() : is_initialized(false),
                   drop_new_frame(false),
                   frame_number(0),
                   keyframe_number(0),
                   tracking_status(eTrackingFail),
                   time(0),
                   inlier_noise_error(0),
                   rmsre(0),
                   distance_traveled(0),
                   mean_track_length(0),
                   num_mit_matches(0),
                   num_tracked_landmarks(0),
                   num_new_landmarks(0),
                   is_imu_converged(false) {}

  /// Have the first set of frames being processed.
  bool is_initialized;

  /// Create a new reference frame in the map.
  bool drop_new_frame;

  /// Current frame number.
  uint32_t frame_number;

  /// Current keyframe number. Only incremented when a new
  /// keyframe is added.
  uint32_t keyframe_number;

  /// Result of the frame-to-frame pose estimation (good, poor, bad, fail).
  eTrackingState tracking_status;

  /// Timestamp of the current frame.
  double time;

  /// Number of inliers
  uint32_t num_inliers;

  /// Learnt measurement error (it is updated at each iteration).
  double inlier_noise_error;

  /// Root mean squared error (current reprojection error).
  double rmsre;

  /// Estimated distance traveled.
  double distance_traveled;

  /// Mean landmark number of observations.
  double mean_track_length;

  /// Number of match-in-time tentative matches.
  int num_mit_matches;

  /// Number of good landmark observations.
  unsigned int num_tracked_landmarks;

  /// Number of added landmarks.
  unsigned int num_new_landmarks;

  /// RNG for Ransac, etc.
  std::mt19937 rng;

  /// Heuristic: Have the IMU related terms converged yet or do we
  /// need to continue batch operations?
  bool is_imu_converged;

  void Clear() {
    rmsre             = 0.0;
    distance_traveled = 0.0;
    frame_number   = 0;
    tracking_status   = common::eTrackingGood;
    is_initialized    = false;
  }
};
}
}
