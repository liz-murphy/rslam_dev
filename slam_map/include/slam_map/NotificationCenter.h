// Copyright (c) Jack Morrison, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <vector>

#include <slam_map/SlamMapFwd.h>
#include <slam_map/MapEvent.h>
#include <mutex>
namespace rslam {
namespace map {
class NotificationCenter {
 public:
  NotificationCenter() {}
  virtual ~NotificationCenter() {}

  void Subscribe(const std::vector<MapEvent>& event_subscriptions,
                 const NotificationCallback& listener);
  /** Notify listeners of an event using the MapEventUpdate structure */
  void Notify(const MapEventUpdate& update) const;
  void Notify(const MapEvent& event, const ReferenceFrameId& id) const;
  void Notify(const MapEvent& event, const TransformEdgeId& id) const;

 private:
  mutable std::mutex callback_mutex_;
  std::vector<std::tuple<std::vector<rslam::map::MapEvent>,
                         rslam::map::NotificationCallback> > callbacks_;
};
}
}
