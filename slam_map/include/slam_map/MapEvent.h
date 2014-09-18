// Copyright (c) Jack Morrison, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <slam_map/SlamMapFwd.h>
#include <slam_map/ReferenceFrameId.h>
#include <slam_map/TransformEdgeId.h>

namespace rslam {
namespace map {

enum MapEvent {
  kFinalMapEvent,  // MapEvent for shutdown

  // Frame events: update payload is frame id
  kAddFrameMapEvent,
  kUpdateFrameMapEvent,

  // Edge events: update payload is edge id
  kAddEdgeMapEvent,
  kUpdateEdgeMapEvent,

  kMapEventMax  // Not an actual event
};

struct MapEventUpdate {
  MapEvent event;

  // The payload. Type contained depends on MapEvent
  union payload_t {
    ReferenceFrameId frame;
    TransformEdgeId edge;
    payload_t() {}
  } payload;

  MapEventUpdate();
  MapEventUpdate(const MapEventUpdate& u);
  MapEventUpdate& operator=(const MapEventUpdate& u);
 protected:
  inline void CopyFrom(const MapEventUpdate& u);
};

typedef std::function<void(const MapEventUpdate&)> NotificationCallback;
}  // namespace rslam
}  // namespace map
