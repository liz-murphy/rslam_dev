// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//   \file MeasurementId.h
//    Using a measurement ID we can look up the actual measurement, which is
//    stored in a vehicle frame somewhere.

#pragma once

#include <slam_map/LandmarkId.h>
#include <slam_map/ReferenceFrameId.h>

struct MeasurementId {
  MeasurementId() = default;
  MeasurementId(const MeasurementId& other) = default;
  MeasurementId& operator=(const MeasurementId& other) = default;
  MeasurementId(const ReferenceFrameId& ref_frame_id, const LandmarkId& lm_id)
      : frame_id(ref_frame_id), landmark_id(lm_id) {}
  ~MeasurementId() = default;

  bool operator<(const MeasurementId& RHS) const {
    if (frame_id == RHS.frame_id) {
      return landmark_id < RHS.landmark_id;
    }
    return frame_id < RHS.frame_id;
  }

  bool operator==(const MeasurementId &RHS) const {
    return (landmark_id == RHS.landmark_id && frame_id == RHS.frame_id);
  }
  ReferenceFrameId frame_id;
  LandmarkId landmark_id;
};

inline std::ostream& operator<<(std::ostream& os, const MeasurementId& id) {
  return os << "(MeasurementId=" << id.frame_id << ", "
            << id.landmark_id << ")";
}

namespace std {
template <> struct hash<MeasurementId> {
  size_t operator()(const MeasurementId& x) const {
    return (frame_hash(x.frame_id) ^ landmark_hash(x.landmark_id));
  }

  hash<ReferenceFrameId> frame_hash;
  hash<LandmarkId> landmark_hash;
};
}
