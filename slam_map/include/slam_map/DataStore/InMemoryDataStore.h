#pragma once

#include <map>
#include <mutex>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/DataStore/SlamMapDataStore.h>

class InMemoryDataStore : public SlamMapDataStore {
 public:
  InMemoryDataStore() {}
  virtual ~InMemoryDataStore() {}

  virtual void Save() const {}

  virtual bool CanSave() const {
    return false;
  }

  ///////////////////
  // Frame Methods //
  ///////////////////
  virtual void AddFrame(const SlamFramePtr& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    ReferenceFrameId id = frame->id();
    frames_.insert({id, frame});
    sessions_.insert(id.session_id);
  }

  virtual SlamFramePtr _GetFramePtr(const ReferenceFrameId& frame_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = frames_.find(frame_id);
    if (it != frames_.end()){
      return it->second;
    }

    return nullptr;
  }

  virtual SlamFramePtr GetFramePtr(const ReferenceFrameId& frame_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = frames_.find(frame_id);
    if (it != frames_.end()){
      return it->second;
    }

    return nullptr;
  }

  virtual void SetFramePtr(const ReferenceFrameId& frame_id,
                           const SlamFramePtr& pFrame) {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_[frame_id] = pFrame;
  }

  virtual unsigned int NumFrames() {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.size();
  }

  virtual void ClearFrames() {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
  }

  //////////////////
  // Edge Methods //
  //////////////////
  virtual void AddEdge(const SlamEdgePtr& edge) {
    std::lock_guard<std::mutex> lock(mutex_);
    edges_.insert({edge->id(), edge});
  }

  virtual SlamEdgePtr _GetEdgePtr(const TransformEdgeId& edge_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = edges_.find(edge_id);
    if (it != edges_.end()){
      return it->second;
    }

    return nullptr;
  }

  virtual SlamEdgePtr GetEdgePtr(const TransformEdgeId& edge_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = edges_.find(edge_id);
    if (it != edges_.end()){
      return it->second;
    }

    return nullptr;
  }

  virtual unsigned int NumEdges() {
    std::lock_guard<std::mutex> lock(mutex_);
    return edges_.size();
  }

  virtual void ClearEdges() {
    std::lock_guard<std::mutex> lock(mutex_);
    edges_.clear();
  }

  virtual void SetEdgePtr(const TransformEdgeId& edge_id,
                          const SlamEdgePtr& edge) {
    std::lock_guard<std::mutex> lock(mutex_);
    edges_[edge_id] = edge;
  }

  virtual bool IsFrameLoaded(const ReferenceFrameId& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.count(id);
  }

  virtual bool IsEdgeLoaded(const TransformEdgeId& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return edges_.count(id);
  }

  virtual size_t NumCameras() const {
    return cameras_.size();
  }

  virtual void AddCamera(const SessionId& session_id,
                         const _CameraRigPtr& cam) {
    cameras_[session_id] = cam;
  }

  virtual void SetCamera(const SessionId& session_id,
                         const _CameraRigPtr& cam) {
    cameras_[session_id] = cam;
  }

  virtual CameraRigPtr GetCamera(const SessionId& session_id) const {
    auto it = cameras_.find(session_id);
    if (it != cameras_.end()) {
      return it->second;
    }
    return nullptr;
  }

  void sessions(std::vector<SessionId>* sessions) const override {
    CHECK_NOTNULL(sessions);
    std::set<SessionId> ids;
    for (const auto& pair : frames_) {
      ids.insert(pair.first.session_id);
    }

    for (const auto& pair : edges_) {
      ids.insert(pair.first.session_id);
    }
    sessions->assign(ids.begin(), ids.end());
  }

  unsigned int NumSessions() const override {
    return sessions_.size();
  }

  void set_notification_center(
      const std::shared_ptr<rslam::map::NotificationCenter>&) override {}

 private:
  std::unordered_map<ReferenceFrameId, SlamFramePtr> frames_;
  std::unordered_map<TransformEdgeId, SlamEdgePtr> edges_;
  std::map<SessionId, CameraRigPtr> cameras_;
  std::set<SessionId> sessions_;
  mutable std::mutex mutex_;
};
