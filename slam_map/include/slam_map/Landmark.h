// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <vector>
//#include <Common/config.h>
//#include <Utils/MathTypes.h>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/LandmarkId.h>
#include <slam_map/MeasurementId.h>

enum LandmarkState {
  eLmkActive      = 1,
  eLmkMono        = 2,
  eLmkNotReliable = 3,
  eLmkAtInfinity  = 4
};

inline std::string LandmarkStateToString(LandmarkState e) {
  switch(e){
    case eLmkActive       : return "eLmkActive";
    case eLmkMono         : return "eLmkMono";
    case eLmkNotReliable  : return "eLmkNotReliable";
    case eLmkAtInfinity   : return "eLmkAtInfinity";
  }
  return "ERROR";
}

class Landmark {
 public:
  enum {
    kPatchNumPixels = CANONICAL_PATCH_SIZE * CANONICAL_PATCH_SIZE
  };
  typedef Eigen::Matrix<uint8_t, kPatchNumPixels, 1> PatchVectorT;

  Landmark(): is_active_(true),
              num_failed_track_attempts_(0),
              base_camera_(0),
              state_(eLmkActive),
              extent_(0),
              cam_plane_extent_(0) {
  }

  Landmark(const Landmark& lmk) : Landmark() {
    _Copy(lmk);
  }

  void Init(const MeasurementId&    zid,
            const bool              active,
            const LandmarkId&       id,
            const unsigned int      base_camera,
            const Eigen::Vector4t&  relative_pos,
            const Sophus::SO3t&     orientation_3d,
            const Scalar            extent,
            const unsigned char*    descriptor,
            const unsigned int      desc_size,
            const PatchVectorT&     patch);

  Landmark& operator=(const Landmark& RHS) {
    _Copy(RHS);
    return *this;
  }

  unsigned int base_camera() const {
    return base_camera_;
  }

  Eigen::Vector4t xrp() const {
    return xrp_;
  }

  LandmarkId id() const {
    return id_;
  }

  // Not including OutsideFOV or other valid reasons not to track.
  unsigned int num_failed_track_attempts() const {
    return num_failed_track_attempts_;
  }

  bool is_active() const {
    return is_active_;
  }

  LandmarkState state() const {
    return state_;
  }

  const unsigned char* patch() const {
    return &patch_[0];
  }

  const unsigned char* descriptor()  const {
    return &descriptor_[0];
  }

  /** Get length of edge of patch */
  static constexpr size_t patch_size()  {
    return CANONICAL_PATCH_SIZE;
  }

  size_t desc_size() const {
    return descriptor_.size();
  }

  const Sophus::SO3t& orientation() const {
    return orientation3d_;
  }

  double extent() const {
    return extent_;
  }

  double cam_plane_extent() const {
    return cam_plane_extent_;
  }

  PatchVectorT patch_vector() const {
    return patch_;
  }

  void set_id(const LandmarkId& id) {
    id_ = id;
  }

  void set_xrp(const Eigen::Vector4t& Xrp) {
    xrp_ = Xrp;
  }

  void set_state(LandmarkState state){
    state_ = state;
  }

  void set_active(bool active) {
    is_active_ = active;
  }

  void set_extent(double extent) {
    extent_ = extent;
  }

  void set_camera_plane_extent(double extent) {
    cam_plane_extent_ = extent;
  }

  void set_track2d_id(int id) {
    id_.track2d_id = id;
  }

  void set_orientation(const Sophus::SO3t& t) {
    orientation3d_ = t;
  }

  void AddToFeatureTrack(const MeasurementId& zId,
                         const bool bSuccessTracking = true);
  void RemoveFromFeatureTrack(const MeasurementId& zId,
                              const bool bSuccessTracking);
  const std::vector<MeasurementId> &GetFeatureTrackRef() const {
    return feature_track_;
  }

  size_t FeatureTrackLength() const {
    return feature_track_.size();
  }

  ///
  /// Return 3D patch corners in landmark base coordinate frame
  void Get3DPatchCorners(Eigen::Vector3t& tl3d,
                         Eigen::Vector3t& tr3d,
                         Eigen::Vector3t& bl3d,
                         Eigen::Vector3t& br3d) const {
    tl3d = Eigen::Vector3t(0,  extent_, -extent_);
    tr3d = Eigen::Vector3t(0, -extent_, -extent_);
    bl3d = Eigen::Vector3t(0,  extent_, extent_);
    br3d = Eigen::Vector3t(0, -extent_, extent_);

    tl3d = orientation3d_*tl3d + xrp_.head<3>();
    tr3d = orientation3d_*tr3d + xrp_.head<3>();
    bl3d = orientation3d_*bl3d + xrp_.head<3>();
    br3d = orientation3d_*br3d + xrp_.head<3>();
  }

  /// Comparison for use in stl::map
  bool operator<(const Landmark& RHS) const {
    return id_ < RHS.id_;
  }

 protected:
  void _Copy(const Landmark& lmk);

 private:
  bool is_active_;
  unsigned int num_failed_track_attempts_;
  unsigned int base_camera_;
  LandmarkId id_;
  LandmarkState state_;

  /** feature info */
  PatchVectorT patch_;
  std::vector<uint8_t> descriptor_;

  /** Landmark 3D surface orientation */
  Sophus::SO3t orientation3d_;

  /** Landmark "size" */
  double extent_;

  // Landmark size on the z=1 plane
  double cam_plane_extent_;

  /** landmark position, state estimate */
  Eigen::Vector4t xrp_;

  std::vector<MeasurementId> feature_track_;
};
