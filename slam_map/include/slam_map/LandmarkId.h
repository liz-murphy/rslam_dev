// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

///   \file LandmarkId.h
///   LandmarkId's specify the reference frame in which the landmark
///   is stored, as well as the index within that frame.  This allows
///   landmark lookup.

#pragma once

#include <slam_map/ReferenceFrameId.h>

/** Unique identifier for landmarks */
struct LandmarkId {
  LandmarkId() : landmark_index(-1), track2d_id(-1) {}
  LandmarkId(const LandmarkId& other) = default;
  LandmarkId& operator=(const LandmarkId& other) = default;
  LandmarkId(const ReferenceFrameId& ref_id, uint32_t lm_index)
      : ref_frame_id(ref_id), landmark_index(lm_index), track2d_id(-1) {}
  ~LandmarkId() = default;

  // so we can use LandmarkId's in stl::map
  bool operator<(const LandmarkId& RHS) const {
    if (ref_frame_id == RHS.ref_frame_id) {
      return landmark_index < RHS.landmark_index;
    }

    return ref_frame_id < RHS.ref_frame_id;
  }

  bool operator==(const LandmarkId &RHS) const {
    return ((landmark_index == RHS.landmark_index) &&
            (ref_frame_id == RHS.ref_frame_id));
  }

  /** which frame this landmark is stored in. */
  ReferenceFrameId ref_frame_id;

  /** landmark index within the reference frame. */
  unsigned int landmark_index;

  /** Use only for Track2d */
  int track2d_id;
};

inline std::ostream& operator<<(std::ostream& os, const LandmarkId& id) {
  return os << "(LandmarkId = " << id.landmark_index << ", "
            << "track2d_id = " << id.track2d_id << ", "
            << id.ref_frame_id  << ")";
}

namespace std {
template <> struct hash<LandmarkId> {
  size_t operator()(const LandmarkId& x) const {
    return (frame_hash(x.ref_frame_id) ^ index_hash(x.landmark_index));
  }

  hash<ReferenceFrameId> frame_hash;
  hash<decltype(LandmarkId::landmark_index)> index_hash;
  hash<decltype(LandmarkId::track2d_id)> track2d_hash;
};
}
