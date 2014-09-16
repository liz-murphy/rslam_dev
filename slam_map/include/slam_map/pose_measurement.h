// Copyright (c) John Morrison, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <Common/scalar.h>
#include <Utils/MathTypes.h>

namespace rslam {
namespace map {

/** Absolute vehicle pose measurements */
struct PoseMeasurement {
  /** Measured pose estimate. (world to vehicle) */
  Sophus::SE3t t_wv;

  /**
   * Measurement covariance (x, y, z  roll, pitch, yaw).
   *
   * @note This is in the SE3 tangent space.
   */
  Eigen::Matrix6t cov = Eigen::Matrix6t::Zero();

  Scalar timestamp = 0.0;
};
}  // namespace rslam
}  // namespace map
