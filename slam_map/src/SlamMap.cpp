// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#include <algorithm>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>
#include <slam_map/SlamMap.h>
#include <slam_map/DataStore/InMemoryDataStore.h>
#include <slam_map/DataStore/InMemorySingleTrackDataStore.h>
#include <slam_map/DataStore/PersistentDataStore.h>
#include <slam_map/FrameObject.h>
#include <slam_map/NotificationCenter.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/MapVisitor/NearbyNodesMapVisitor.h>
#include <slam_map/SlamMapParamsConfig.h>

//static int& g_hold_debug_level =
 //   CVarUtils::CreateCVar<>("map.hold_frames_debug", 1,
  //                          "Debug level for SlamMap frame holding.");

// This will disappear when we properly namespace this class
using namespace rslam::map;

SlamMap::SlamMap()
    : store_(new InMemoryDataStore()),
      token_(0),
      hold_token_(0),
      last_frame_id_(-1),
      id_(SessionId::generate()),
      notifier_(std::make_shared<NotificationCenter>()) {
}

SlamMap::~SlamMap() {
  notifier_->Notify(kFinalMapEvent, ReferenceFrameId());
}

/// Allocate a new frame, but do not link it into the graph.
SlamFramePtr SlamMap::AddFrame(const double time) {
  SlamFramePtr frame(new ReferenceFrame);
  ReferenceFrameId new_frame_id(last_frame_id_ + 1, id_);
  frame->set_id(new_frame_id);
  frame->set_time(time);
  frame->set_notification_center(notifier_);

  if (last_frame_id_ >= 0) {
    SlamFramePtr last_frame = store_->GetFramePtr(
        ReferenceFrameId(last_frame_id_, id_));
    if (last_frame) {
      frame->set_g_r(last_frame->g_r());
      frame->set_v_r(last_frame->v_r());
      frame->set_b(last_frame->b());
      frame->set_t_vs(last_frame->t_vs());
      frame->set_cam_params(last_frame->cam_params());
    }
  }

  store_->AddFrame(frame);
  ++last_frame_id_;
  notifier_->Notify(kAddFrameMapEvent, new_frame_id);
  return frame;
}

void SlamMap::AddFrame(const SlamFramePtr& new_node) {
  CHECK(new_node);
  if (SlamFramePtr existing = GetFramePtr(new_node->id())) {
    existing->Merge(*new_node);
  } else {
    store_->AddFrame(new_node);
    new_node->set_notification_center(notifier_);
    notifier_->Notify(kAddFrameMapEvent, new_node->id());
  }
}

void SlamMap::AddEdge(const SlamEdgePtr& new_edge) {
  CHECK(new_edge);
  TransformEdgeId edge_id = new_edge->id();
  bool added_new = false;
  if (SlamEdgePtr existing = GetEdgePtr(edge_id)) {
    existing->Merge(*new_edge);
  } else {
    store_->AddEdge(new_edge);
    new_edge->set_notification_center(notifier_);
    added_new = true;
  }

  if (SlamFramePtr start = GetFramePtr(edge_id.start)) {
    if (!start->HasNeighbor(edge_id)) {
      start->AddNeighbor(edge_id);
      start->set_linked();
    }
  }
  if (SlamFramePtr end = GetFramePtr(edge_id.end)) {
    if (!end->HasNeighbor(edge_id)) {
      end->AddNeighbor(edge_id);
      end->set_linked();
    }
  }

  Sophus::SE3t t_ab;
  new_edge->transform(edge_id.start, edge_id.end, t_ab);

  if (added_new) {
    notifier_->Notify(kAddEdgeMapEvent, new_edge->id());
  }
}

void SlamMap::Clear() {
  store_->ClearFrames();
  store_->ClearEdges();
}

void SlamMap::Save() const {
  store_->Save();
}

bool SlamMap::CanSave() const {
  return store_->CanSave();
}

unsigned int SlamMap::NumFrames() const {
  return store_->NumFrames();
}

unsigned int SlamMap::NumSessions() const {
  return store_->NumSessions();
}

unsigned int SlamMap::NumEdges() const {
  return store_->NumEdges();
}

// lookup the id
/// Output: global Id of edge
bool SlamMap::_FindEdgeId(const ReferenceFrameId& start_id,
                          const ReferenceFrameId& end_id,
                          TransformEdgeId& edge_id) const {
  SlamFramePtr pf = GetFramePtr(start_id);
  if (!pf) {
    return false;
  }

  const std::vector<TransformEdgeId>& edge_ids = pf->Neighbors();
  for (const TransformEdgeId& neighbor_id : edge_ids) {
    if (SlamEdgePtr edge = store_->GetEdgePtr(neighbor_id)) {
      if ((edge->end_id() == end_id && edge->start_id() == start_id) ||
          (edge->end_id() == start_id && edge->start_id() == end_id)) {
        edge_id = neighbor_id;
        return true;
      }
    }
  }
  return false;
}

SlamEdgePtr SlamMap::AddEdge(const SlamFramePtr& a,
                             const SlamFramePtr& b,
                             const Sophus::SE3t &t_ab,
                             bool is_loop_closure) {
  if (HasEdge(a->id(), b->id())) {
    std::cerr << "WARNING: Edge between frames " << a->id()
              << " and " << b->id() << " already exist [Add Edge]" << std::endl;
    return GetEdgePtr(a->id(), b->id());
  }

  TransformEdgeId edge_id(store_->NumEdges(), a->id(), b->id(), id_);
  SlamEdgePtr edge(new TransformEdge(edge_id, t_ab));


  // Edge must be added before neighbors are set, otherwise there's a
  // race condition where the edge could be accessed through the
  // neighbors.
  store_->AddEdge(edge);
  edge->set_notification_center(notifier_);

  if (SlamFramePtr _a = GetFramePtr(a->id())) {
    _a->AddNeighbor(edge_id);
    _a->set_linked();
  }
  if (SlamFramePtr _b = GetFramePtr(b->id())) {
    if (_b->is_isolated()) {
      _b->set_parent_edge_id(edge_id);
    }

    _b->set_linked();
    _b->AddNeighbor(edge_id);
  }

  edge->set_is_loop_closure(is_loop_closure);

  notifier_->Notify(kAddEdgeMapEvent, edge_id);
  return edge;
}

bool SlamMap::HasFrame(const ReferenceFrameId& id) const {
  return store_->GetFramePtr(id).get();
}

bool SlamMap::HasEdge(const ReferenceFrameId& start_id,
                      const ReferenceFrameId& end_id) const {
  TransformEdgeId tmp;
  return _FindEdgeId(start_id, end_id, tmp);
}

void SlamMap::SwapEdgeStartFrame(const TransformEdgeId& old_edge_id,
                                 const ReferenceFrameId& old_start_id,
                                 const ReferenceFrameId& new_start_id ,
                                 const Sophus::SE3t new_t_ab) {
  CHECK_EQ(old_edge_id.start, old_start_id);
  SlamEdgePtr edge = GetEdgePtr(old_edge_id);
  SlamFramePtr pOldStartFrame = GetFramePtr(old_start_id);
  SlamFramePtr pNewStartFrame = GetFramePtr(new_start_id);
  SlamFramePtr pEndFrame = GetFramePtr(old_edge_id.end);

  pEndFrame->RemoveNeighbor(old_edge_id);
  pOldStartFrame->RemoveNeighbor(old_edge_id);
  edge->set_start_id(new_start_id);
  edge->set_transform(new_t_ab);

  TransformEdgeId new_edge_id = edge->id();
  pNewStartFrame->AddNeighbor(new_edge_id);
  pEndFrame->AddNeighbor(new_edge_id);
}

SlamEdgePtr SlamMap::GetEdgePtr(const ReferenceFrameId& start_id,
                                const ReferenceFrameId& end_id) const {
  if (HasFrame(start_id) && HasFrame(end_id)) {
    TransformEdgeId edge_id;
    if (_FindEdgeId(start_id, end_id, edge_id)) {
      return store_->GetEdgePtr(edge_id);
    }
  }

  return nullptr;
}

SlamEdgePtr
SlamMap::GetEdgePtr(const TransformEdgeId& edge_id) const {
  return store_->GetEdgePtr(edge_id);
}

// return reference to frame
SlamFramePtr
SlamMap::GetFramePtr(const MeasurementId& id) const {
  return GetFramePtr(id.frame_id);
}

SlamFramePtr
SlamMap::GetFramePtr(const ReferenceFrameId& frame_id) const {
  return store_->GetFramePtr(frame_id);
}

SlamFramePtr
SlamMap::GetFramePtrLoaded(const ReferenceFrameId& frame_id) const {
  return IsFrameLoaded(frame_id) ? GetFramePtr(frame_id) : nullptr;
}

SlamEdgePtr
SlamMap::GetEdgePtrLoaded(const TransformEdgeId& edge_id) const {
  return IsEdgeLoaded(edge_id) ? GetEdgePtr(edge_id) : nullptr;
}

// update the transform between two frames:
void SlamMap::UpdateEdgeImuPoints(
    const TransformEdgeId& edge_id,
    const std::vector<ba::ImuPoseT<Scalar>>& vMeas,
    const Eigen::Vector3t& g) {
  SlamEdgePtr edge = store_->GetEdgePtr(edge_id);
  if (!edge) {
    std::cerr << "WARNING: Edge " << edge_id <<
        " is NULL [UpdateImuPoints] " << std::endl;
  }

  Eigen::Vector3tArray poses;
  for (const auto& pose : vMeas) {
    poses.push_back(pose.t_wp.translation());
  }
  edge->SetImuPoses(std::move(poses));
  edge->set_g(g);
}

bool SlamMap::GetMeasurement(const MeasurementId& id,
                             MultiViewMeasurement &Msr) const {
  SlamFramePtr ptr = store_->GetFramePtr(id.frame_id);
  return ptr && ptr->GetMeasurement(id, &Msr);
}

// Return reference in Msr to measurement identified by a Frame Id and
// a local measurement index. True if found, false otherwise.
bool SlamMap::GetMeasurement(const ReferenceFrameId& frame_id,
                             const unsigned int uMsrLocalIndex,
                             MultiViewMeasurement &Msr) const {
  SlamFramePtr ptr = store_->GetFramePtr(frame_id);
  return ptr && ptr->GetMeasurement(uMsrLocalIndex, &Msr);
}

bool SlamMap::GetLandmark(const LandmarkId& Id, Landmark* lm) const {
  SlamFramePtr ptr = store_->GetFramePtr(Id.ref_frame_id);
  return ptr && ptr->GetLandmark(Id.landmark_index, lm);
}

void SlamMap::AddNewMeasurementsToFrame(
    const ReferenceFrameId& frame_id,
    std::vector<MultiViewMeasurement> &measurements) {
  SlamFramePtr z_frame = GetFramePtr(frame_id);
  if (!z_frame) return;

  Landmark lm;
  for (MultiViewMeasurement& z : measurements) {
    SlamFramePtr frame = store_->GetFramePtr(z.id().landmark_id.ref_frame_id);
    if (frame &&
        frame->GetLandmark(z.id().landmark_id.landmark_index, &lm) &&
        lm.is_active()) {
      z_frame->AddMeasurement(z);
      frame->AddMultiViewMeasurementoLandmark(z);
    }
  }
}

void SlamMap::RemoveMeasurementsFromFrame(const ReferenceFrameId& frame_id) {
  // first eliminate measurements from landmark sessions
  SlamFramePtr frame = store_->GetFramePtr(frame_id);

  MeasurementId zid;

  size_t num_meas = frame->NumMeasurements();
  Landmark lm;
  for (size_t i = 0; i < num_meas; ++i) {
    if (!frame->GetMeasurementId(i, &zid)) continue;

    auto frame2 = store_->GetFramePtr(zid.landmark_id.ref_frame_id);
    frame2->GetLandmark(zid.landmark_id.landmark_index, &lm);

    bool success_sessioning = frame->HasGoodMeasurement(i);
    lm.RemoveFromFeatureTrack(zid, success_sessioning);
    frame2->SetLandmark(zid.landmark_id.landmark_index, lm);
  }

  // now delete measurement vector
  store_->GetFramePtr(frame_id)->RemoveMeasurements();
}

uint64_t SlamMap::GetHoldToken() const {
  ++hold_token_;
  std::lock_guard<std::mutex> lock(hold_mutex_);
  hold_updating_[hold_token_] = false;
  return hold_token_;
}

void SlamMap::RemoveFrameHold(uint64_t token) {
  std::lock_guard<std::mutex> lock(hold_mutex_);
  holds_.erase(token);
}

void SlamMap::SetHoldFrames(uint64_t token,
                            const ReferenceFrameId& root_id,
                            unsigned int depth,
                            bool with_covisible) {
  LOG(hold_debug_level_) << "Holding frames: root : " << root_id
                          << " to depth " << depth
                          << (with_covisible ? "" : " not")
                          << " including covisible" << std::endl;
  {
    std::lock_guard<std::mutex> lock(hold_mutex_);
    auto found = holds_.find(token);
    if (found != holds_.end()) {
      HoldRequest& hold = found->second;
      if (hold.root_id == root_id &&
          hold.depth == depth &&
          hold.with_covisible == with_covisible) {
        return;
      }

      hold.root_id = root_id;
      hold.depth = depth;
      hold.with_covisible = with_covisible;

    } else {
      holds_[token] = {root_id, depth, with_covisible};
    }
    if (hold_updating_[token]) return;
  }
  std::thread update_thread(std::bind(&SlamMap::UpdateHeldFrames, this, token));
  update_thread.detach();
}

void SlamMap::SetHoldFramesSync(uint64_t token,
                                const ReferenceFrameId& root_id,
                                unsigned int depth,
                                bool with_covisible) {
  {
    std::lock_guard<std::mutex> lock(hold_mutex_);
    holds_[token] = {root_id, depth, with_covisible};
  }

  UpdateHeldFrames(token);
}

void SlamMap::ClearEdgeAttributes(uint32_t attribute) {
  std::lock_guard<std::mutex> lock(edge_attrib_mutex_);
  auto itr = edge_attributes_.begin();
  while (itr != edge_attributes_.end()) {
    if (itr->second == attribute || itr->second == 0) {
      edge_attributes_.erase(itr++);
    } else {
      if (itr->second & attribute) {
        itr->second &= ~attribute;
      }
      itr++;
    }
  }
}

void SlamMap::SetEdgeAttribute(const TransformEdgeId &id, uint32_t attribute) {
  std::lock_guard<std::mutex> lock(edge_attrib_mutex_);
  edge_attributes_[id] |= attribute;
}

uint32_t SlamMap::GetEdgeAttribute(const TransformEdgeId &id) const {
  std::lock_guard<std::mutex> lock(edge_attrib_mutex_);
  auto it = edge_attributes_.find(id);
  if (it != edge_attributes_.end()) {
    return it->second;
  } else {
    return 0;
  }
}


void SlamMap::UpdateHeldFrames(uint64_t token) {
  HoldRequest hold;
  {
    std::lock_guard<std::mutex> lock(hold_mutex_);
    if (hold_updating_[token]) return;  // Only update this token w/ one thread
    hold_updating_[token] = true;
    hold = holds_[token];
  }

  std::set<ReferenceFrameId> new_holds;
  if (hold.depth > 0) {
    NearbyNodesMapVisitor visitor(this, &new_holds, hold.with_covisible);
    visitor.set_root_id(hold.root_id);
    visitor.set_depth(hold.depth);
    BFS(&visitor);
    LOG(hold_debug_level_) << "Gathered " << new_holds.size()
                            << " frames to be held from "
                            << " root id " << hold.root_id
                            << " at depth " << hold.depth
                            << (hold.with_covisible ? "" : " not")
                            << " including covisible" << std::endl;
  }

  LOG(hold_debug_level_) << "Setting " << new_holds.size()
                          << " frames to be held" << std::endl;

  {
    std::lock_guard<std::mutex> lock(hold_update_mutex_);

    held_frames_[token].clear();
    held_edges_[token].clear();
    for (const ReferenceFrameId& frame_id : new_holds) {
      if (SlamFramePtr frame = GetFramePtr(frame_id)) {
        held_frames_[token].push_back(frame);

        const std::vector<TransformEdgeId>& neighbors = frame->Neighbors();
        for (const TransformEdgeId& eid : neighbors) {
          if (SlamEdgePtr edge = GetEdgePtr(eid)) {
            held_edges_[token].push_back(edge);
          }
        }
      }
    }
  }
  std::lock_guard<std::mutex> lock(hold_mutex_);
  hold_updating_[token] = false;
}

void SlamMap::InitWithPersistence(const std::string& filename,
                                  bool continue_existing) {
  auto store = new PersistentDataStore();
  store->Init(filename, continue_existing);
  store_.reset(store);
  store_->set_notification_center(notifier_);
}

void SlamMap::InitInMemory(const SessionStorage& type) {
  if (type == kSingleSessionStorage) {
    store_.reset(new InMemorySingleTrackDataStore());
  } else {
    store_.reset(new InMemoryDataStore());
  }
  store_->set_notification_center(notifier_);
}

bool SlamMap::IsFrameLoaded(const ReferenceFrameId& id) const {
  return store_->IsFrameLoaded(id);
}

bool SlamMap::IsEdgeLoaded(const TransformEdgeId& id) const {
  return store_->IsEdgeLoaded(id);
}

void SlamMap::BFS(MapVisitor* visitor) const {
  InternalBFS(visitor, false);
}

void SlamMap::InMemoryBFS(MapVisitor* visitor) const {
  InternalBFS(visitor, true);
}

void SlamMap::InternalBFS(MapVisitor* visitor, bool only_loaded) const {
  const unsigned int max_depth = visitor->depth();
  ReferenceFrameId root_id = visitor->root_id();
  const bool ignore_broken = visitor->should_ignore_broken();
  const bool depth_equals_count = visitor->depth_equals_count();

  if (!root_id.valid()) {
    LOG(FATAL) << "BFS called with invalid frame id: " << root_id;
    visitor->Finished();
    return;
  } else if (max_depth == 0) {
    LOG(WARNING) << "Empty BFS called. Did you forget to set a depth?";
    visitor->Finished();
    return;
  }

  bool has_visit = visitor->has_visit();
  bool has_explore_edge = visitor->has_explore_edge();
  bool has_explore_node = visitor->has_explore_node();

#define GET_FRAME(id)                                           \
  (only_loaded ? GetFramePtrLoaded(id) : GetFramePtr(id))

#define GET_EDGE(id)                                            \
  (only_loaded ? GetEdgePtrLoaded(id) : GetEdgePtr(id))

  // get the first node (which will be the origin of the coordinate frame)
  SlamFramePtr root_frame = GET_FRAME(root_id);
  if (!root_frame) {
    visitor->Finished();
    return;
  }

  // now search for close nodes
  typedef std::tuple<ReferenceFrameId, uint32_t> FrameDepth;
  std::queue<FrameDepth> q;
  std::unordered_set<ReferenceFrameId> seen;
  std::set<TransformEdgeId> seen_edges;
  seen.insert(root_id);
  q.emplace(root_id, 0);

  size_t count = 0;
  while (!q.empty() && !visitor->IsDone()) {
    FrameDepth fd = q.front();
    q.pop();

    const ReferenceFrameId& cur_node_id = std::get<0>(fd);
    const uint32_t& cur_depth = std::get<1>(fd);

    SlamFramePtr cur_node = GET_FRAME(cur_node_id);

    if (!cur_node || (has_visit && !visitor->Visit(cur_node))) continue;

    // Explore neighbours
    unsigned int num_neighbors = cur_node->NumNeighbors();
    for (unsigned int ii = 0; ii < num_neighbors; ++ii) {
      TransformEdgeId neighbor_edge_id = cur_node->GetNeighborEdgeId(ii);
      ReferenceFrameId neighbor_node_id =
          neighbor_edge_id.OtherEnd(cur_node_id);

      if (cur_depth >= max_depth) continue;

      // We don't explore an edge twice, though we will continue to a
      // node across a new edge if we haven't seen the edge before
      if (seen_edges.count(neighbor_edge_id)) continue;
      seen_edges.insert(neighbor_edge_id);

      SlamFramePtr neighbor_node;
      SlamEdgePtr neighbor_edge;
      if (has_explore_edge || has_explore_node) {
        neighbor_node = GET_FRAME(neighbor_node_id);
        neighbor_edge = GET_EDGE(neighbor_edge_id);
        if (!neighbor_node ||
            !neighbor_edge ||
            (!ignore_broken && neighbor_edge->is_broken())) {
          continue;
        }
      }

      if (has_explore_edge) {
        visitor->ExploreEdge(cur_node, neighbor_edge, neighbor_node);
      }

      if (seen.count(neighbor_node_id) ||
          (has_explore_node &&
           !visitor->ExploreNode(cur_node, neighbor_edge, neighbor_node))) {
        continue;
      }

      seen.insert(neighbor_node_id);
      q.emplace(neighbor_node_id, cur_depth + 1);
    }

    // if we require a strict number of nodes returned, then exit once
    // we have reached the limit
    if (depth_equals_count && ++count > max_depth) break;
  }
  visitor->Finished();
}

void SlamMap::ParentTraverse(MapVisitor* visitor) const {
  const unsigned int max_depth = visitor->depth();
  ReferenceFrameId root_id = visitor->root_id();
  const bool ignore_broken = visitor->should_ignore_broken();

  if (!root_id.valid()) {
    LOG(WARNING) << "ParentTraverse called with invalid frame id.";
    visitor->Finished();
    return;
  } else if (max_depth == 0) {
    LOG(WARNING) << "Empty ParentTraverse called. "
                 << "Did you forget to set a depth?";
    visitor->Finished();
    return;
  }

  bool has_visit = visitor->has_visit();
  bool has_explore_edge = visitor->has_explore_edge();
  bool has_explore_node = visitor->has_explore_node();

  ReferenceFrameId cur_node_id = root_id;
  size_t count = 0;
  while (!visitor->IsDone()) {
    SlamFramePtr cur_node = GetFramePtr(cur_node_id);
    if (!cur_node || (has_visit && !visitor->Visit(cur_node))) {
      break;
    }

    TransformEdgeId parent_edge_id = cur_node->parent_edge_id();
    if (!parent_edge_id.valid()) {
      break;
    }

    ReferenceFrameId parent_node_id = parent_edge_id.OtherEnd(cur_node_id);
    cur_node_id = parent_node_id;

    // Because this is linear, count == depth
    if (count++ >= max_depth) {
      break;
    }

    SlamFramePtr parent_node;
    SlamEdgePtr parent_edge;
    if (has_explore_edge || has_explore_node) {
      parent_node = GetFramePtr(parent_node_id);
      parent_edge = GetEdgePtr(parent_edge_id);
      if (!parent_node ||
          !parent_edge ||
          (!ignore_broken && parent_edge->is_broken())) {
        break;
      }
    }

    if (has_explore_edge) {
      visitor->ExploreEdge(cur_node, parent_edge, parent_node);
    }

    if ((has_explore_node &&
         !visitor->ExploreNode(cur_node, parent_edge, parent_node))) {
      break;
    }
  }
  visitor->Finished();
}

size_t SlamMap::NumCameras() const {
  return store_->NumCameras();
}

void SlamMap::AddCamera(const SessionId& session_id, const _CameraRigPtr& cam) {
  store_->AddCamera(session_id, cam);
}

void SlamMap::SetCamera(const SessionId& session_id, const _CameraRigPtr& cam) {
  store_->SetCamera(session_id, cam);
}

CameraRigPtr SlamMap::GetCamera(const SessionId& session_id) const {
  return store_->GetCamera(session_id);
}

bool SlamMap::HasCamera(const SessionId& session_id) const {
  return store_->GetCamera(session_id).get();
}

void SlamMap::Subscribe(const std::vector<MapEvent>& event_subscriptions,
                        const NotificationCallback& callback) {
  notifier_->Subscribe(event_subscriptions, callback);
}

void SlamMap::sessions(std::vector<SessionId>* sessions) const {
  store_->sessions(sessions);
}

ReferenceFrameId SlamMap::SessionStart(const SessionId& session) const {
  ReferenceFrameId frame_id;
  frame_id.session_id = session;

  SlamFramePtr frame;
  for (frame_id.id = 0; !frame; ++frame_id.id) {
    frame = GetFramePtr(frame_id);
  }
  return frame->id();
}
