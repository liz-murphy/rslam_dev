#pragma once

#include <vector>
#include <slam_map/PatchHomography.h>
#include <slam_map/Landmark.h>
#include <slam_map/Measurement.h>
#include <slam_map/SlamMapFwd.h>
#include <utils/MathTypes.h>

struct LandmarkContainer {
  LandmarkContainer();
  LandmarkContainer(const Landmark& lmk, const SlamMap* slam_map);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LandmarkState state;
  LandmarkId id;
  /// \brief  3D position in the lifted local frame
  Eigen::Vector4t x_r;

  /// \brief 3D position in the lifted local frame
  Eigen::Vector4t x_w;

  /// \brief id used in bundle adjustment
  unsigned int ba_id;

  /// \brief The ration of the depth of the landmark before and
  /// after the BA operation
  double depth_ratio;

  std::vector<std::shared_ptr<MeasurementContainer> > measurements;
  unsigned int base_camera_id;
  unsigned int num_frames_visible;
  Sophus::SO3Group<Scalar> orientation_3d;

  // hmm should these working variable be elsewhere?
  // NO.  TODO we move this to a TrackingWorkingSet
  std::vector<PatchHomography<CANONICAL_PATCH_SIZE> > tracking_homographies;
};
