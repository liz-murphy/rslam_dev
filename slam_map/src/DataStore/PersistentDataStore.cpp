// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#include <set>
#include <string>
#include <vector>

#include <slam_map/DataStore/SlamEdgePtrTable.h>
#include <slam_map/DataStore/SlamFramePtrTable.h>
#include <slam_map/DataStore/CameraRigPtrTable.h>
#include <slam_map/DataStore/PersistentDataStore.h>
#include <slam_map/DataStore/PersistentBackend.h>

static const int kNumDbConnections = 5;

typedef PersistentDataStore::FrameStoreT FrameStoreT;
typedef PersistentDataStore::EdgeStoreT EdgeStoreT;
typedef PersistentDataStore::CameraRigStoreT CameraRigStoreT;

inline void persist_thread(const std::atomic<bool>* should_stop,
                           std::shared_ptr<FrameStoreT> frames,
                           std::shared_ptr<EdgeStoreT> edges) {
  std::chrono::milliseconds wait_time(1000);
  while (!*should_stop) {
    if (!edges->ShouldEvict() && !frames->ShouldEvict()) {
      edges->WaitUntilAdd(wait_time);
      continue;
    }

    frames->Evict();
    edges->Evict();
  }
}

PersistentDataStore::PersistentDataStore() {}

PersistentDataStore::~PersistentDataStore() {
  should_persistor_end_ = true;
  if (persistor_.joinable()) {
    persistor_.join();
  }
}

void PersistentDataStore::Init(const std::string& filename,
                               bool continue_existing) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  filename_ = filename;
  frame_store_.reset(new FrameStoreT(filename_, !continue_existing,
                                     kNumDbConnections));
  edge_store_.reset(new EdgeStoreT(filename_, !continue_existing,
                                   kNumDbConnections));
  rig_store_.reset(new CameraRigStoreT(filename_, !continue_existing,
                                       kNumDbConnections));

  if (!continue_existing) {
    frame_store_->Clear();
    edge_store_->Clear();
    rig_store_->Clear();
  }

  should_persistor_end_ = false;
  persistor_ = std::thread(persist_thread,
                           &should_persistor_end_,
                           frame_store_,
                           edge_store_);
  frame_store_->UniqueIds(SlamFramePtrTable::MapIdColumn, &sessions_);
}

void PersistentDataStore::AddFrame(const SlamFramePtr& frame) {
  ReferenceFrameId id = frame->id();
  frame_store_->Add(id, frame);
  sessions_.insert(id.session_id);
}

SlamFramePtr
PersistentDataStore::_GetFramePtr(const ReferenceFrameId& frame_id) {
  return frame_store_->Get(frame_id);
}

SlamFramePtr
PersistentDataStore::GetFramePtr(const ReferenceFrameId& frame_id) const {
  return frame_store_->Get(frame_id);
}

void PersistentDataStore::SetFramePtr(const ReferenceFrameId& frame_id,
                                      const SlamFramePtr& frame) {
  CHECK_EQ(frame_id, frame->id());
  frame_store_->Set(frame_id, frame);
}

unsigned int PersistentDataStore::NumFrames() {
  return frame_store_->Size();
}

void PersistentDataStore::ClearFrames() {
  frame_store_->Clear();
}

void PersistentDataStore::AddEdge(const SlamEdgePtr& edge) {
  edge_store_->Add(edge->id(), edge);
}

SlamEdgePtr PersistentDataStore::_GetEdgePtr(const TransformEdgeId& edge_id) {
  return edge_store_->Get(edge_id);
}

SlamEdgePtr
PersistentDataStore::GetEdgePtr(const TransformEdgeId& edge_id) const {
  return edge_store_->Get(edge_id);
}

unsigned int PersistentDataStore::NumEdges() {
  return edge_store_->Size();
}

void PersistentDataStore::ClearEdges() {
  edge_store_->Clear();
}

void PersistentDataStore::SetEdgePtr(const TransformEdgeId& edge_id,
                                     const SlamEdgePtr& edge) {
  CHECK_EQ(edge_id, edge->id());
  edge_store_->Set(edge_id, edge);
}

bool PersistentDataStore::IsFrameLoaded(const ReferenceFrameId& id) const {
  return frame_store_->IsInMemory(id);
}

bool PersistentDataStore::IsEdgeLoaded(const TransformEdgeId& id) const {
  return edge_store_->IsInMemory(id);
}

size_t PersistentDataStore::NumCameras() const {
  return rig_store_->Size();
}

void PersistentDataStore::AddCamera(const SessionId& session_id,
                                    const _CameraRigPtr& cam) {
  rig_store_->Add(session_id, cam);
}

void PersistentDataStore::SetCamera(const SessionId& session_id,
                                     const _CameraRigPtr& cam) {
  rig_store_->Set(session_id, cam);
}

CameraRigPtr PersistentDataStore::GetCamera(const SessionId& session_id) const {
  return rig_store_->Get(session_id);
}

void PersistentDataStore::Save() const {
  frame_store_->Flush();
  edge_store_->Flush();
  rig_store_->Flush();
}

void PersistentDataStore::sessions(std::vector<SessionId>* sessions) const {
  CHECK_NOTNULL(sessions);
  sessions->assign(sessions_.begin(), sessions_.end());
}

unsigned int PersistentDataStore::NumSessions() const {
  return sessions_.size();
}

void PersistentDataStore::set_notification_center(
    const std::shared_ptr<rslam::map::NotificationCenter>& n) {
  frame_store_->set_on_load_callback([n](const SlamFramePtr& f) {
      f->set_notification_center(n);
    });
  edge_store_->set_on_load_callback([n](const SlamEdgePtr& e) {
      e->set_notification_center(n);
    });
}
