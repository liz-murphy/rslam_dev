// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

///   \file session_id.h
///
///   SessionId's identify a particular map
#include <algorithm>
#include <ostream>
#include <iostream>
#include <type_traits>
#include <slam_map/uuid.h>

struct SessionId {
  SessionId() = default;
  SessionId(const SessionId& other) = default;
  SessionId& operator=(const SessionId& other) = default;

  ~SessionId() = default;

  static inline SessionId generate() {
    SessionId id;
    rslam::uuid::uuid_generate(id.uuid);

    return id;
  }

  bool operator<(const SessionId& RHS) const {
    return rslam::uuid::uuid_less_than(uuid, RHS.uuid);
  }

  bool operator==(const SessionId &RHS) const {
    return rslam::uuid::uuid_equal(uuid, RHS.uuid);
  }

  bool operator!=(const SessionId &RHS) const {
    return !rslam::uuid::uuid_equal(uuid, RHS.uuid);
  }

  rslam::uuid::uuid_t uuid;
};

namespace std {
template <> struct hash<SessionId> {
  size_t operator()(const SessionId& x) const {
    return uuid_hash(x.uuid);
  }

  std::hash<rslam::uuid::uuid_t> uuid_hash;
};
}

inline std::ostream& operator<<(std::ostream& os, const SessionId& id) {
  rslam::uuid::uuid_string_t str;
  rslam::uuid::uuid_unparse(id.uuid, str);
  return os << "SessionId=" << str;
}
