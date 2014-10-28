#include <slam_map/MapEvent.h>

namespace rslam {
namespace map {
//MapEventUpdate::MapEventUpdate() : event(kMapEventMax) {}
MapEventUpdate::MapEventUpdate(const MapEventUpdate& u) {
  CopyFrom(u);
}

MapEventUpdate& MapEventUpdate::operator=(const MapEventUpdate& u) {
  if (this != &u) {
    CopyFrom(u);
  }
  return *this;
}

inline void MapEventUpdate::CopyFrom(const MapEventUpdate& u) {
  event = u.event;
  if (event == kAddFrameMapEvent ||
      event == kUpdateFrameMapEvent) {
    payload.frame = u.payload.frame;
  } else if (event == kAddEdgeMapEvent ||
             event == kUpdateEdgeMapEvent) {
    payload.edge = u.payload.edge;
  }
}
}  // namespace rslam
}  // namespace map
