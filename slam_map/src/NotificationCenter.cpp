#include <slam_map/NotificationCenter.h>

namespace rslam {
namespace map {

static const auto event_equals = [](const MapEvent& update_event,
                                    const MapEvent& desired) {
  return update_event == desired || update_event == kFinalMapEvent;
};

void NotificationCenter::Subscribe(
    const std::vector<MapEvent>& event_subscriptions,
    const NotificationCallback& callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callbacks_.emplace_back(event_subscriptions, callback);
}

void NotificationCenter::Notify(const MapEvent& event,
                                const ReferenceFrameId& id) const {
  MapEventUpdate update;
  update.event = event;
  update.payload.frame = id;
  Notify(update);
}

void NotificationCenter::Notify(const MapEvent& event,
                                const TransformEdgeId& id) const {
  MapEventUpdate update;
  update.event = event;
  update.payload.edge = id;
  Notify(update);
}

void NotificationCenter::Notify(const MapEventUpdate& update) const {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  for (const auto& f : callbacks_) {
    const std::vector<MapEvent>& events = std::get<0>(f);
    if (events.empty() ||
        std::any_of(events.begin(), events.end(),
                    std::bind(event_equals, update.event,
                              std::placeholders::_1))) {
      std::get<1>(f)(update);
    }
  }
}
}  // namespace rslam
}  // namespace map
