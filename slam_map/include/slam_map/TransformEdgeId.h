// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

///   \file TransformEdgeId.h
///
///   TransformEdgeId's specify the map which the reference frame
///   originated in and the frame's own ID within that map.
#include <limits>
#include <ostream>
#include <slam_map/ReferenceFrameId.h>
#include <slam_map/session_id.h>

struct TransformEdgeId {
  explicit TransformEdgeId(uint32_t i, const SessionId& m)
      : id(i), session_id(m) {}
  explicit TransformEdgeId(uint32_t i,
                           const ReferenceFrameId& s,
                           const ReferenceFrameId& e,
                           const SessionId& m) :
      id(i), session_id(m), start(s), end(e) {}

  TransformEdgeId() = default;
  TransformEdgeId(const TransformEdgeId& other) = default;
  TransformEdgeId& operator=(const TransformEdgeId& other) = default;
  ~TransformEdgeId() = default;

  bool valid() const {
    return id != kNoEdge;
  }

  /**
   * Get the other end of an edge. Returns start node if frame is not
   * part of this edge.
   */
  ReferenceFrameId OtherEnd(const ReferenceFrameId& frame) const {
    return (frame == start) ? end : start;
  }

  bool operator<(const TransformEdgeId& RHS) const {
    if (rslam::uuid::uuid_equal(session_id.uuid, RHS.session_id.uuid)) {
      return id < RHS.id;
    }
    return rslam::uuid::uuid_less_than(session_id.uuid, RHS.session_id.uuid);
  }

  bool operator==(const TransformEdgeId &RHS) const {
    return id == RHS.id && session_id == RHS.session_id;
  }

  bool operator!=(const TransformEdgeId &RHS) const {
    return id != RHS.id || session_id != RHS.session_id;
  }

  int64_t operator-(const TransformEdgeId &RHS) const {
    return static_cast<int64_t>(id) - static_cast<int64_t>(RHS.id);
  }

  uint32_t id = kNoEdge;
  SessionId session_id;
  ReferenceFrameId start, end;

 private:
  // id value for an unassigned edge id
  static const uint32_t kNoEdge = std::numeric_limits<uint32_t>::max();
};

namespace std {
template <> struct hash<TransformEdgeId> {
  size_t operator()(const TransformEdgeId& x) const {
    return id_hash(x.id) ^ map_hash(x.session_id);
  }

  hash<decltype(TransformEdgeId::id)> id_hash;
  hash<decltype(TransformEdgeId::session_id)> map_hash;
};
}

inline std::ostream& operator<<(std::ostream& os, const TransformEdgeId& id) {
  return os << "(TransformEdgeId=" << id.id
            << ", " << id.session_id
            << ", " << id.start
            << ", " << id.end
            << ")";
}
