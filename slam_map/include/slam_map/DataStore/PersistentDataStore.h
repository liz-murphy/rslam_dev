// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <atomic>
#include <set>
#include <string>
#include <thread>
#include <queue>
#include <unordered_map>
#include <vector>
#include <slam_map/slam_mapFwd.h>
#include <slam_map/DataStore/slam_mapDataStore.h>

class PersistentDataStore : public slam_mapDataStore {
 public:
  typedef PersistentBackend<SlamEdgePtrTable> EdgeStoreT;
  typedef PersistentBackend<SlamFramePtrTable> FrameStoreT;
  typedef PersistentBackend<CameraRigPtrTable> CameraRigStoreT;

  PersistentDataStore();
  virtual ~PersistentDataStore();

  /// REQUIRED - Initialize data storage
  void Init(const std::string& filename,
            bool continue_existing);

  void Save() const override;

  bool CanSave() const override {
    return true;
  }

  ///////////////////
  // Frame Methods //
  ///////////////////
  void AddFrame(const SlamFramePtr& pFrame) override;
  void SetFramePtr(const ReferenceFrameId& frame_id,
                   const SlamFramePtr& pFrame) override;
  SlamFramePtr _GetFramePtr(const ReferenceFrameId& frame_id) override;
  SlamFramePtr GetFramePtr(const ReferenceFrameId& frame_id) const override;
  unsigned int NumFrames() override;
  void ClearFrames() override;
  bool IsFrameLoaded(const ReferenceFrameId& id) const override;

  //////////////////
  // Edge Methods //
  //////////////////
  void AddEdge(const SlamEdgePtr& frame) override;
  void SetEdgePtr(const TransformEdgeId& uEdgeId,
                  const SlamEdgePtr& pEdge) override;
  SlamEdgePtr _GetEdgePtr(const TransformEdgeId& uEdgeId) override;
  SlamEdgePtr GetEdgePtr(const TransformEdgeId& uEdgeId) const override;
  unsigned int NumEdges() override;
  void ClearEdges() override;
  bool IsEdgeLoaded(const TransformEdgeId& id) const override;

  size_t NumCameras() const override;
  void AddCamera(const SessionId& session_id,
                 const _CameraRigPtr& cam) override;
  void SetCamera(const SessionId& session_id,
                 const _CameraRigPtr& cam) override;
  CameraRigPtr GetCamera(const SessionId& session_id) const override;

  void sessions(std::vector<SessionId>* sessions) const override;
  unsigned int NumSessions() const override;

  void set_notification_center(
      const std::shared_ptr<rslam::map::NotificationCenter>& n) override;

 protected:
  void Close();

 private:
  std::string filename_;

  std::atomic<bool> should_persistor_end_;
  std::thread persistor_;

  /** @todo Setter and getter and test for frame limit */
  std::shared_ptr<FrameStoreT> frame_store_;
  std::shared_ptr<EdgeStoreT> edge_store_;
  std::shared_ptr<CameraRigStoreT> rig_store_;

  /** @todo Add mutex to protect sessions_ */
  std::set<SessionId> sessions_;

  std::mutex hold_mutex_;
  std::vector<SlamFramePtr> held_frames_;
  std::vector<SlamEdgePtr> held_edges_;

  std::shared_ptr<rslam::map::NotificationCenter> notification_center_;
};
