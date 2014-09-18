#pragma once

#include <map>
#include <mutex>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/DataStore/SlamMapDataStore.h>

class InMemorySingleTrackDataStore : public SlamMapDataStore {
 public:
  InMemorySingleTrackDataStore() {}
  virtual ~InMemorySingleTrackDataStore() {}

  void Save() const {}

  bool CanSave() const {
    return false;
  }

  ///////////////////
  // Frame Methods //
  ///////////////////
  void AddFrame(const SlamFramePtr& frame) override {
    std::lock_guard<std::mutex> lock(mutex_);
    DCHECK_EQ(frame->id().id, frames_.size()) << "# frames: " << frames_.size()
                                              << ", id: " << frame->id();
    frames_.push_back(frame);
  }

  SlamFramePtr _GetFramePtr(const ReferenceFrameId& frame_id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    return (frame_id.id < frames_.size()) ? frames_[frame_id.id] : nullptr;
  }

  SlamFramePtr GetFramePtr(const ReferenceFrameId& frame_id) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return (frame_id.id < frames_.size()) ? frames_[frame_id.id] : nullptr;
  }

  void SetFramePtr(const ReferenceFrameId& frame_id,
                   const SlamFramePtr& pFrame) override {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK_LT(frame_id.id, frames_.size());
    frames_[frame_id.id] = pFrame;
  }

  unsigned int NumFrames() override {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_.size();
  }

  void ClearFrames() override {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
  }

  //////////////////
  // Edge Methods //
  //////////////////
  void AddEdge(const SlamEdgePtr& edge) override {
    std::lock_guard<std::mutex> lock(mutex_);
    DCHECK_EQ(edge->id().id, edges_.size()) << "# edges: " << edges_.size()
                                            << ", id: " << edge->id();
    edges_.push_back(edge);
  }

  SlamEdgePtr _GetEdgePtr(const TransformEdgeId& edge_id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    return (edge_id.id < edges_.size()) ? edges_[edge_id.id] : nullptr;
  }

  SlamEdgePtr GetEdgePtr(const TransformEdgeId& edge_id) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return (edge_id.id < edges_.size()) ? edges_[edge_id.id] : nullptr;
  }

  unsigned int NumEdges() override {
    std::lock_guard<std::mutex> lock(mutex_);
    return edges_.size();
  }

  void ClearEdges() override {
    std::lock_guard<std::mutex> lock(mutex_);
    edges_.clear();
  }

  void SetEdgePtr(const TransformEdgeId& edge_id,
                  const SlamEdgePtr& edge) override {
    std::lock_guard<std::mutex> lock(mutex_);
    edges_[edge_id.id] = edge;
  }

  bool IsFrameLoaded(const ReferenceFrameId& id) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return id.id < frames_.size();
  }

  bool IsEdgeLoaded(const TransformEdgeId& id) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return id.id < edges_.size();
  }

  size_t NumCameras() const override {
    return 1;
  }

  void AddCamera(const SessionId& session_id, const _CameraRigPtr& cam) override {
    // This should only ever have a single camera.
    CHECK(!session_);
    session_.reset(new SessionId(session_id));
    camera_ = cam;
  }

  void SetCamera(const SessionId& session_id, const _CameraRigPtr& cam) override {
    if (!session_) {
      session_.reset(new SessionId(session_id));
    } else {
      CHECK_EQ(*session_, session_id);
    }
    camera_ = cam;
  }

  CameraRigPtr GetCamera(const SessionId& session_id) const override {
    CHECK(session_);
    CHECK_EQ(*session_, session_id);
    return camera_;
  }

  void sessions(std::vector<SessionId>* sessions) const override {
    CHECK_NOTNULL(sessions);
    sessions->clear();
    if (session_) {
      sessions->push_back(*session_);
    }
  }

  unsigned int NumSessions() const override {
    return session_ ? 1 : 0;
  }

  void set_notification_center(
      const std::shared_ptr<rslam::map::NotificationCenter>& n) override {}

 private:
  std::vector<SlamFramePtr> frames_;
  std::vector<SlamEdgePtr> edges_;
  CameraRigPtr camera_;
  std::unique_ptr<SessionId> session_;
  mutable std::mutex mutex_;
};
