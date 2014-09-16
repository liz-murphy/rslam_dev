// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//   \file Landmark.h

#include <slam_map/Landmark.h>

void Landmark::Init(const MeasurementId&    r_zid,
                    const bool              is_active,
                    const LandmarkId&       id,
                    const unsigned int      base_camera,
                    const Eigen::Vector4t&  relative_pos,
                    const Sophus::SO3t&     orientation3d,
                    const Scalar            extent,
                    const unsigned char*    descriptor,
                    const unsigned int      desc_size,
                    const PatchVectorT&     pPatch) {
  feature_track_.clear();
  is_active_     = is_active;
  id_            = id;
  base_camera_   = base_camera;
  xrp_           = relative_pos;
  extent_        = extent;
  orientation3d_ = orientation3d;

  patch_ = pPatch;
  if (desc_size > 0 && descriptor) {
    descriptor_.assign(descriptor, descriptor + desc_size);
  }

  // start the feature track
  feature_track_.push_back(r_zid);
}

void Landmark::AddToFeatureTrack(const MeasurementId &zId,
                                 const bool bSuccessTracking) {
  if(!bSuccessTracking) {
    ++num_failed_track_attempts_;
    if(num_failed_track_attempts_ > 5) { /// @todo holy hard coded disaster
      is_active_ = false;
    }
  }

  feature_track_.push_back(zId);
}

void Landmark::RemoveFromFeatureTrack(const MeasurementId &zId,
                                      const bool bSuccessTracking) {
  auto it = std::find(feature_track_.begin(), feature_track_.end(), zId);
  if(it != feature_track_.end()) {
    feature_track_.erase(it);
  }

  if(!bSuccessTracking) {
    --num_failed_track_attempts_;
    if(num_failed_track_attempts_ <= 5) {
      is_active_ = true;
    }
  }
}

void Landmark::_Copy(const Landmark& lmk) {
  is_active_             = lmk.is_active_;
  base_camera_           = lmk.base_camera_; // this should be encoded in the Id
  xrp_                   = lmk.xrp_;
  feature_track_         = lmk.feature_track_;
  id_                    = lmk.id_;
  num_failed_track_attempts_ = lmk.num_failed_track_attempts_;
  orientation3d_         = lmk.orientation3d_;
  extent_                = lmk.extent_;
  state_                 = lmk.state_;
  patch_                 = lmk.patch_;
  descriptor_            = lmk.descriptor_;
  feature_track_         = lmk.feature_track_;
  cam_plane_extent_      = lmk.cam_plane_extent_;
}
