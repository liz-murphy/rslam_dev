// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//   \file Measurement.h   This class represents multi-view image
//                         measurements for synced rigs.

#pragma once

#include <string>
#include <vector>
#include <slam_map/PatchHomography.h>
#include <Eigen/Core>
#include <slam_map/MatchFlags.h>
#include <slam_map/MeasurementId.h>
#include <common/config.h>
#include <common/scalar.h>
///
/// A measurement in a single camera
struct Measurement {
  enum {
    kPatchSize = CANONICAL_PATCH_SIZE * CANONICAL_PATCH_SIZE
  };

  Measurement() : flag(UnInitialized),
                  pixel(Eigen::Matrix<Scalar,2,1>(-1.0, -1.0)),
                  matching_error(-1.0),
                  reprojection_error(-1.0),
                  scale(0.0),
                  orientation(0.0) {}
  MatchFlag flag;
  Eigen::Matrix<Scalar, 2, 1> pixel;
  Scalar matching_error;
  Scalar reprojection_error;
  float scale;
  float orientation;
  PatchHomography<CANONICAL_PATCH_SIZE> tracking_homography;
  Eigen::Matrix<uint8_t, kPatchSize, 1> patch;
};

///
/// Represents a point viewed (possibly) from multiple cameras
class MultiViewMeasurement {
 public:
  MultiViewMeasurement() : rig_size_(0) {}

  explicit MultiViewMeasurement(int rig_size)
      : rig_size_(rig_size), meas_(rig_size) {}

  // Query
  bool HasGoodMeasurementInCam(int cam) const {
    return (cam < rig_size_ &&
            (IsFlagged(cam, NewFeature) ||
             IsFlagged(cam, GoodMatch) ||
             IsFlagged(cam, Rethreaded)));
  }

  bool HasGoodMeasurement() const {
    for (int cam = 0; cam < rig_size_; ++cam) {
      const MatchFlag& flag = meas_[cam].flag;
      if (!(flag == NewFeature || flag == GoodMatch || flag == Rethreaded)) {
        return false;
      }
    }
    return true;
  }

  unsigned int NumCameras() const {
    return rig_size_;
  }

  MeasurementId id() const {
    return id_;
  }

  const unsigned char* patch(int cam) const {
    return &(meas_[cam].patch[0]);
  }

  float Scale(int cam) const {
    return meas_[cam].scale;
  }

  float Orientation(int cam) const {
    return meas_[cam].orientation;
  }

  Scalar MatchingError(int cam) const {
    return meas_[cam].matching_error;
  }

  Scalar ReprojectionError(int cam) const {
    return meas_[cam].reprojection_error;
  }

  MatchFlag Flag(int cam) const {
    return meas_[cam].flag;
  }

  static constexpr size_t patch_size() {
    return Measurement::kPatchSize;
  }

  decltype(Measurement::patch)& PatchVector(int cam) {
    return meas_[cam].patch;
  }

  const decltype(Measurement::patch)& PatchVector(int cam) const {
    return meas_[cam].patch;
  }

  const Eigen::Matrix<Scalar, 2, 1>& Pixel(int cam) const {
    return meas_[cam].pixel;
  }

  const PatchHomography<CANONICAL_PATCH_SIZE>&
  GetPatchHomography(int cam) const {
    return meas_[cam].tracking_homography;
  }

  bool IsFlagged(int cam, MatchFlag Option) const {
    return (meas_[cam].flag == Option);
  }

  void set_id(const MeasurementId& id) {
    id_ = id;
  }

  // Setters
  void SetFlag(int cam, MatchFlag Option) {
    meas_[cam].flag = Option;
  }

  void SetScale(int cam, const float fScale) {
    meas_[cam].scale = fScale;
  }

  void SetOrientation(int cam, const float fOrientation) {
    meas_[cam].orientation = fOrientation;
  }

  void SetPixelInCam(int cam, Scalar U, Scalar V) {
    meas_[cam].pixel << U, V;
  }

  void SetMatchingError(int cam, const Scalar dError) {
    meas_[cam].matching_error = dError;
  }

  void SetReprojectionError(int cam, const Scalar dError) {
    meas_[cam].reprojection_error = dError;
  }

  void SetPatchHomography(int cam,
                          const PatchHomography<CANONICAL_PATCH_SIZE>& H) {
    meas_[cam].tracking_homography = H;
  }

  std::string InfoStr() {
    std::stringstream ss;
    ss << "Measurement 'f" << id_ << "':";
    for (int n = 0; n < rig_size_; n++){
      ss << "  cam[" << n << "]=[" << meas_[n].pixel.transpose() << "] ";
      ss << MatchStr(meas_[n].flag);
    }

    return ss.str();
  }

 private:
  int rig_size_;
  MeasurementId id_;
  std::vector<Measurement> meas_;
};
