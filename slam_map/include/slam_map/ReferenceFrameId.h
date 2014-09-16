// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

///   \file ReferenceFrameId.h
///
///   ReferenceFrameId's specify the map which the reference frame
///   originated in and the frame's own ID within that map.
#include <limits>
#include <ostream>
#include <slam_map/session_id.h>

struct ReferenceFrameId {
  explicit ReferenceFrameId(uint32_t i, const SessionId& m) :
      id(i), session_id(m) {}

  ReferenceFrameId() = default;
  ReferenceFrameId(const ReferenceFrameId& other) = default;
  ReferenceFrameId& operator=(const ReferenceFrameId& other) = default;
  ~ReferenceFrameId() = default;

  bool valid() const {
    return id != kNoFrame;
  }

  bool operator<(const ReferenceFrameId& RHS) const {
    if (rslam::uuid::uuid_equal(session_id.uuid, RHS.session_id.uuid)) {
      return id < RHS.id;
    }
    return rslam::uuid::uuid_less_than(session_id.uuid, RHS.session_id.uuid);
  }

  bool operator==(const ReferenceFrameId &RHS) const {
    return id == RHS.id && session_id == RHS.session_id;
  }

  bool operator!=(const ReferenceFrameId &RHS) const {
    return id != RHS.id || session_id != RHS.session_id;
  }

  uint32_t id = kNoFrame;
  SessionId session_id;

 private:
  // id value for an unassigned edge id
  static const uint32_t kNoFrame = std::numeric_limits<uint32_t>::max();
};

namespace std {
template <> struct hash<ReferenceFrameId> {
  size_t operator()(const ReferenceFrameId& x) const {
    return id_hash(x.id) ^ map_hash(x.session_id);
  }

  hash<decltype(ReferenceFrameId::session_id)> map_hash;
  hash<decltype(ReferenceFrameId::id)> id_hash;
};
}

inline std::ostream& operator<<(std::ostream& os, const ReferenceFrameId& id) {
  return os << "(ReferenceFrameId=" << id.id << ", " << id.session_id << ")";
}
