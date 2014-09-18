// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <vector>
#include <slam_map/PointerSlamMapProxy.h>
#include <slam_map/SlamMap.h>

PointerSlamMapProxy::PointerSlamMapProxy(const std::shared_ptr<SlamMap>& map)
    : map_(map) {}

PointerSlamMapProxy::~PointerSlamMapProxy() {}

bool PointerSlamMapProxy::HasFrame(const ReferenceFrameId& id) const {
  return map_->HasFrame(id);
}

bool PointerSlamMapProxy::HasEdge(const ReferenceFrameId& start_id,
                                  const ReferenceFrameId& end_id) const {
  return map_->HasEdge(start_id, end_id);
}

bool PointerSlamMapProxy::IsFrameLoaded(const ReferenceFrameId& id) const {
  return map_->IsFrameLoaded(id);
}

bool PointerSlamMapProxy::IsEdgeLoaded(const TransformEdgeId& id) const {
  return map_->IsEdgeLoaded(id);
}

SlamFramePtr
PointerSlamMapProxy::GetFramePtr(const ReferenceFrameId& id) const {
  return map_->GetFramePtr(id);
}

SlamEdgePtr PointerSlamMapProxy::GetEdgePtr(const TransformEdgeId& id) const {
  return map_->GetEdgePtr(id);
}

SlamEdgePtr
PointerSlamMapProxy::GetEdgePtr(const ReferenceFrameId& start_id,
                                const ReferenceFrameId& end_id) const {
  return map_->GetEdgePtr(start_id, end_id);
}

CameraRigPtr PointerSlamMapProxy::GetCamera(const SessionId& session_id) const {
  return map_->GetCamera(session_id);
}

bool PointerSlamMapProxy::GetMeasurement(const MeasurementId& nId,
                                         MultiViewMeasurement& Msr) const {
  return map_->GetMeasurement(nId, Msr);
}

bool PointerSlamMapProxy::GetMeasurement(const ReferenceFrameId& frame_id,
                                         unsigned int uMsrLocalIndex,
                                         MultiViewMeasurement& Msr) const {
  return map_->GetMeasurement(frame_id, uMsrLocalIndex, Msr);
}

bool PointerSlamMapProxy::GetLandmark(const LandmarkId& Id,
                                      Landmark* lm) const {
  return map_->GetLandmark(Id, lm);
}

unsigned int PointerSlamMapProxy::NumEdges() const {
  return map_->NumEdges();
}

unsigned int PointerSlamMapProxy::NumFrames() const {
  return map_->NumFrames();
}

unsigned int PointerSlamMapProxy::NumCameras() const {
  return map_->NumCameras();
}

unsigned int PointerSlamMapProxy::NumSessions() const {
  return map_->NumSessions();
}

void PointerSlamMapProxy::sessions(std::vector<SessionId>* sessions) const {
  map_->sessions(sessions);
}

ReferenceFrameId PointerSlamMapProxy::SessionStart(
    const SessionId& track) const {
  return map_->SessionStart(track);
}

void PointerSlamMapProxy::Save() const {
  map_->Save();
}

uint32_t PointerSlamMapProxy::GetEdgeAttribute(
    const TransformEdgeId &id) const {
  return map_->GetEdgeAttribute(id);
}

void PointerSlamMapProxy::BFS(MapVisitor* visitor) const {
  map_->BFS(visitor);
}

void PointerSlamMapProxy::InMemoryBFS(MapVisitor* visitor) const {
  map_->InMemoryBFS(visitor);
}

void PointerSlamMapProxy::Subscribe(
    const std::vector<rslam::map::MapEvent>& event_subscriptions,
    const rslam::map::NotificationCallback& listener) {
  map_->Subscribe(event_subscriptions, listener);
}
