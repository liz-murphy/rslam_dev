// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <vector>
#include <slam_map/PointerSlamMapProxy.h>
#include <slam_map/slam_map.h>

Pointerslam_mapProxy::Pointerslam_mapProxy(const std::shared_ptr<slam_map>& map)
    : map_(map) {}

Pointerslam_mapProxy::~Pointerslam_mapProxy() {}

bool Pointerslam_mapProxy::HasFrame(const ReferenceFrameId& id) const {
  return map_->HasFrame(id);
}

bool Pointerslam_mapProxy::HasEdge(const ReferenceFrameId& start_id,
                                  const ReferenceFrameId& end_id) const {
  return map_->HasEdge(start_id, end_id);
}

bool Pointerslam_mapProxy::IsFrameLoaded(const ReferenceFrameId& id) const {
  return map_->IsFrameLoaded(id);
}

bool Pointerslam_mapProxy::IsEdgeLoaded(const TransformEdgeId& id) const {
  return map_->IsEdgeLoaded(id);
}

SlamFramePtr
Pointerslam_mapProxy::GetFramePtr(const ReferenceFrameId& id) const {
  return map_->GetFramePtr(id);
}

SlamEdgePtr Pointerslam_mapProxy::GetEdgePtr(const TransformEdgeId& id) const {
  return map_->GetEdgePtr(id);
}

SlamEdgePtr
Pointerslam_mapProxy::GetEdgePtr(const ReferenceFrameId& start_id,
                                const ReferenceFrameId& end_id) const {
  return map_->GetEdgePtr(start_id, end_id);
}

CameraRigPtr Pointerslam_mapProxy::GetCamera(const SessionId& session_id) const {
  return map_->GetCamera(session_id);
}

bool Pointerslam_mapProxy::GetMeasurement(const MeasurementId& nId,
                                         MultiViewMeasurement& Msr) const {
  return map_->GetMeasurement(nId, Msr);
}

bool Pointerslam_mapProxy::GetMeasurement(const ReferenceFrameId& frame_id,
                                         unsigned int uMsrLocalIndex,
                                         MultiViewMeasurement& Msr) const {
  return map_->GetMeasurement(frame_id, uMsrLocalIndex, Msr);
}

bool Pointerslam_mapProxy::GetLandmark(const LandmarkId& Id,
                                      Landmark* lm) const {
  return map_->GetLandmark(Id, lm);
}

unsigned int Pointerslam_mapProxy::NumEdges() const {
  return map_->NumEdges();
}

unsigned int Pointerslam_mapProxy::NumFrames() const {
  return map_->NumFrames();
}

unsigned int Pointerslam_mapProxy::NumCameras() const {
  return map_->NumCameras();
}

unsigned int Pointerslam_mapProxy::NumSessions() const {
  return map_->NumSessions();
}

void Pointerslam_mapProxy::sessions(std::vector<SessionId>* sessions) const {
  map_->sessions(sessions);
}

ReferenceFrameId Pointerslam_mapProxy::SessionStart(
    const SessionId& track) const {
  return map_->SessionStart(track);
}

void Pointerslam_mapProxy::Save() const {
  map_->Save();
}

uint32_t Pointerslam_mapProxy::GetEdgeAttribute(
    const TransformEdgeId &id) const {
  return map_->GetEdgeAttribute(id);
}

void Pointerslam_mapProxy::BFS(MapVisitor* visitor) const {
  map_->BFS(visitor);
}

void Pointerslam_mapProxy::InMemoryBFS(MapVisitor* visitor) const {
  map_->InMemoryBFS(visitor);
}

void Pointerslam_mapProxy::Subscribe(
    const std::vector<rslam::map::MapEvent>& event_subscriptions,
    const rslam::map::NotificationCallback& listener) {
  map_->Subscribe(event_subscriptions, listener);
}
