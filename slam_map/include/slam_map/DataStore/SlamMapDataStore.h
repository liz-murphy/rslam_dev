#pragma once

#include <memory>
#include <set>
#include <vector>
#include <slam_map/SlamMapFwd.h>

class SlamMapDataStore {
 public:
  SlamMapDataStore() {}
  virtual ~SlamMapDataStore() {}

  /** Save the map */
  virtual void Save() const = 0;
  virtual bool CanSave() const = 0;

  /** Get the list of sesions in this store */
  virtual void sessions(std::vector<SessionId>* sessions) const = 0;

  /** How many sesions' of frames make up this map? */
  virtual unsigned int NumSessions() const = 0;

  ///////////////////
  // Frame Methods //
  ///////////////////
  virtual void AddFrame(const SlamFramePtr& frame) = 0;
  virtual SlamFramePtr _GetFramePtr(const ReferenceFrameId& frame_id) = 0;
  virtual SlamFramePtr GetFramePtr(const ReferenceFrameId& frame_id) const = 0;
  virtual void SetFramePtr(const ReferenceFrameId& frame_id,
                           const SlamFramePtr& pFrame) = 0;
  virtual unsigned int NumFrames() = 0;
  virtual void ClearFrames() = 0;
  virtual bool IsFrameLoaded(const ReferenceFrameId& id) const = 0;

  //////////////////
  // Edge Methods //
  //////////////////
  virtual void AddEdge(const SlamEdgePtr& frame) = 0;
  virtual SlamEdgePtr _GetEdgePtr(const TransformEdgeId& uEdgeId) = 0;
  virtual SlamEdgePtr GetEdgePtr(const TransformEdgeId& uEdgeId) const = 0;
  virtual unsigned int NumEdges() = 0;
  virtual void ClearEdges() = 0;
  virtual void SetEdgePtr(const TransformEdgeId& uEdgeId,
                          const SlamEdgePtr& pEdge) = 0;
  virtual bool IsEdgeLoaded(const TransformEdgeId& id) const = 0;

  ////////////////////////
  // Camera Information //
  ////////////////////////
  virtual size_t NumCameras() const = 0;
  virtual void AddCamera(const SessionId& session_id,
                         const _CameraRigPtr& cam) = 0;
  virtual void SetCamera(const SessionId& session_id,
                         const _CameraRigPtr& cam) = 0;
  virtual CameraRigPtr GetCamera(const SessionId& session_id) const = 0;

  virtual void set_notification_center(
      const std::shared_ptr<rslam::map::NotificationCenter>& n) = 0;
};
