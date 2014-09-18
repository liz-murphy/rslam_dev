// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <vector>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/SlamMap.h>
#include <utils/MathTypes.h>

/**
 * Interface for a remotely accessible SlamMap.
 *
 * This means the SlamMap is located elsewhere, but you can still read
 * from it. Operations may go over the network or simply be local.
 */
class SlamMapProxy {
 public:
  SlamMapProxy() {}
  virtual ~SlamMapProxy() {}

  virtual void Save() const = 0;

  virtual bool HasFrame(const ReferenceFrameId& id) const = 0;
  virtual bool HasEdge(const ReferenceFrameId& start_id,
                       const ReferenceFrameId& end_id) const = 0;

  virtual bool IsFrameLoaded(const ReferenceFrameId& id) const = 0;
  virtual bool IsEdgeLoaded(const TransformEdgeId& id) const = 0;
  virtual SlamFramePtr GetFramePtr(const ReferenceFrameId& id) const = 0;
  virtual SlamEdgePtr GetEdgePtr(const TransformEdgeId& id) const = 0;
  virtual SlamEdgePtr GetEdgePtr(const ReferenceFrameId& start_id,
                                  const ReferenceFrameId& end_id) const = 0;
  virtual CameraRigPtr GetCamera(const SessionId& session_id) const = 0;

  virtual bool GetMeasurement(const MeasurementId& nId,
                              MultiViewMeasurement& Msr) const = 0;
  virtual bool GetMeasurement(const ReferenceFrameId& frame_id,
                              unsigned int uMsrLocalIndex,
                              MultiViewMeasurement& Msr) const = 0;
  virtual bool GetLandmark(const LandmarkId& Id, Landmark* lm) const = 0;

  virtual unsigned int NumEdges() const = 0;
  virtual unsigned int NumFrames() const = 0;
  virtual unsigned int NumCameras() const = 0;
  virtual unsigned int NumSessions() const = 0;

  virtual void sessions(std::vector<SessionId>* sessions) const = 0;
  virtual ReferenceFrameId SessionStart(const SessionId& session) const = 0;

  virtual uint32_t GetEdgeAttribute(const TransformEdgeId& id) const = 0;

  virtual void BFS(MapVisitor* visitor) const = 0;
  virtual void InMemoryBFS(MapVisitor* visitor) const = 0;

  virtual void Subscribe(
      const std::vector<rslam::map::MapEvent>& event_subscriptions,
      const rslam::map::NotificationCallback& listener) = 0;
};
