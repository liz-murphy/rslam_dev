// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <slam_map/SlamMapProxy.h>

/**
 * SlamMapProxy that operates through a pointer to a SlamMap.
 */
class PointerSlamMapProxy : public SlamMapProxy {
 public:
  explicit PointerSlamMapProxy(const std::shared_ptr<SlamMap>& map);
  virtual ~PointerSlamMapProxy();

  void Save() const override;

  bool HasFrame(const ReferenceFrameId& id) const override;
  bool HasEdge(const ReferenceFrameId& start_id,
               const ReferenceFrameId& end_id) const override;

  bool IsFrameLoaded(const ReferenceFrameId& id) const override;
  bool IsEdgeLoaded(const TransformEdgeId& id) const override;

  SlamFramePtr GetFramePtr(const ReferenceFrameId& id) const override;
  SlamEdgePtr GetEdgePtr(const TransformEdgeId& id) const override;
  SlamEdgePtr GetEdgePtr(const ReferenceFrameId& start_id,
                         const ReferenceFrameId& end_id) const override;
  CameraRigPtr GetCamera(const SessionId& session_id) const override;

  bool GetMeasurement(const MeasurementId& nId,
                      MultiViewMeasurement& z) const override;
  bool GetMeasurement(const ReferenceFrameId& frame_id,
                      unsigned int z_local_index,
                      MultiViewMeasurement& z) const override;
  bool GetLandmark(const LandmarkId& id, Landmark* lm) const override;

  unsigned int NumEdges() const override;
  unsigned int NumFrames() const override;
  unsigned int NumCameras() const override;
  unsigned int NumSessions() const override;

  void sessions(std::vector<SessionId>* sessions) const override;
  ReferenceFrameId SessionStart(const SessionId& session) const override;

  uint32_t GetEdgeAttribute(const TransformEdgeId& id) const override;

  void BFS(MapVisitor* visitor) const override;
  void InMemoryBFS(MapVisitor* visitor) const override;

  void Subscribe(
      const std::vector<rslam::map::MapEvent>& event_subscriptions,
      const rslam::map::NotificationCallback& listener) override;
 private:
  const std::shared_ptr<SlamMap> map_;
};
