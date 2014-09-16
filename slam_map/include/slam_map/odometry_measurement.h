// Copyright (c) John Morrison, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <Common/scalar.h>
#include <Utils/MathTypes.h>

namespace rslam {
namespace map {

/** A relative pose measurement between two times. */
struct OdometryMeasurement {
  /** Measured relative pose estimate (start to end) */
  Sophus::SE3t t_se;

  /**
   * Measurement covariance (x, y, z  roll, pitch, yaw)
   *
   * @note This is in the SE3 tangent space.
   */
  Eigen::Matrix6t cov = Eigen::Matrix6t::Zero();

  Scalar start_timestamp = 0.0;
  Scalar end_timestamp = 0.0;
};
}  // namespace rslam
}  // namespace map
