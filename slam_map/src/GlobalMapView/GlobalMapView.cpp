// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#include <vector>
#include <slam_map/GlobalMapView/GlobalMapView.h>
#include <slam_map/GlobalMapView/GlobalMapViewUpdater.h>
#include <slam_map/MapVisitor/MapVisitor.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/SlamMap.h>
#include <slam_map/SlamMapProxy.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/SlamMapParamsConfig.h>

GlobalMapView::GlobalMapView(const std::shared_ptr<SlamMapProxy>& map)
    : map_(map), updater_(new GlobalMapViewUpdater(map, this)),
      has_root_id_(false) {}

GlobalMapView::~GlobalMapView() {
  updater_->Quit();
}

void GlobalMapView::Reset(const std::shared_ptr<SlamMapProxy>& map) {
  LockGuardT lock(mutex_);
  map_ = map;
  frames_.clear();
  edges_.clear();
  tree_.clear();
  maps_.clear();
  updater_->Reset(map);
}

void GlobalMapView::set_root_id(const ReferenceFrameId& id) {
  LockGuardT lock(mutex_);
  set_root_id_inline(id);
}

void GlobalMapView::set_root_id_inline(const ReferenceFrameId& id) {
  root_id_ = id;
  has_root_id_ = true;

  auto t_root_from_world = frames_[root_id_].inverse();
  for (auto& pair : frames_) {
    auto& t_wi = pair.second;
    t_wi = t_root_from_world * t_wi;
  }

  frames_[root_id_] = Sophus::SE3t();
}

inline bool GlobalMapView::GetFramePoseInline(const ReferenceFrameId& uId,
                                              Sophus::SE3t* out) const {
  CHECK_NOTNULL(out);
  auto it = frames_.find(uId);
  if (it == frames_.end()) {
    return false;
  }

  *out = it->second;
  return true;
}

bool GlobalMapView::GetFramePose(const ReferenceFrameId& uId,
                                 Sophus::SE3t* out) const {
  LockGuardT lock(mutex_);
  return GetFramePoseInline(uId, out);
}

void GlobalMapView::AddEdge(const TransformEdgeId& id) {
  LockGuardT lock(mutex_);
  AddEdgeInline(id);
}

void GlobalMapView::AddEdgeInline(const TransformEdgeId& id) {
  UpdateEdgeId(id);
  edges_.insert(id);

  ROS_DEBUG_NAMED("slam_map", "Adding edge");

  if (tree_.empty()) {
    set_root_id_inline(id.start);
    tree_[id.start].insert(id);
    UpdateEdgeFromMap(id.start, id.end);
    ROS_DEBUG_NAMED("slam_map", "Adding initial edge");
  } else {
    // Check if this edge is a leaf, or between two existing nodes
    auto edge_start = frames_.find(id.start);
    auto edge_end = frames_.find(id.end);
    auto frames_end = frames_.end();

    // If it's between two existing nodes (and one isn't the parent
    // of the other), we'll just ignore it for now because it would
    // require recomputing the whole tree. We'll wait for a global
    // map operation to do that for us.
    if (edge_start != frames_end &&
        edge_end != frames_end) {
      auto start_tree = tree_.find(id.start);
      auto end_tree = tree_.find(id.end);
      auto tree_end = tree_.end();
      if (start_tree == tree_end || end_tree == tree_end) {
        ROS_DEBUG_NAMED("slam_map", "Skipping end node's position because neither end exists");
        return;
      } else if (end_tree->second.find(id) == end_tree->second.end() &&
                 start_tree->second.find(id) == start_tree->second.end()) {
        ROS_DEBUG_NAMED("slam_map","Skipping end node's position because both ends exist");
        return;
      }
    }
    // Find which end is parent
    if (edge_start != frames_end) {
      tree_[id.start].insert(id);
      UpdateEdgeFromMap(id.start, id.end);
      ROS_DEBUG_NAMED("slam_map","Update end");
    } else if (edge_end != frames_end) {
      tree_[id.end].insert(id);
      UpdateEdgeFromMap(id.end, id.start);
      ROS_DEBUG_NAMED("slam_map","Update start pose");
    } else {
      // Neither end is in the tree, but the tree is also not
      // empty so there's nothing for us to do since we can't root
      // this node anywhere
      ROS_DEBUG_NAMED("slam_map","Edge disconnected from current map");
      return;
    }
  }
}

void GlobalMapView::UpdateEdge(const TransformEdgeId& id) {
  LockGuardT lock(mutex_);
  auto edge_start = tree_.find(id.start);
  auto edge_end = tree_.find(id.end);

  if (edge_start != tree_.end() && edge_start->second.count(id)) {
    UpdateFromRoot(id.start, id.end);
  } else if (edge_end != tree_.end() && edge_end->second.count(id)) {
    UpdateFromRoot(id.end, id.start);
  } else {
    AddEdgeInline(id);
  }
}

void GlobalMapView::SetFramePose(const ReferenceFrameId& id,
                                 const Sophus::SE3t& pose) {
  LockGuardT lock(mutex_);
  frames_[id] = pose;
}

void GlobalMapView::LinesForGL(std::vector<TransformEdgeId>* edge_ids,
                               Eigen::Vector3tArray* points,
                               std::vector<int>* session_ids,
                               int* num_maps) const {
  LockGuardT lock(mutex_);
  edge_ids->clear();
  edge_ids->reserve(edges_.size());
  points->clear();
  points->reserve(edges_.size());
  session_ids->clear();
  int n_maps = maps_.size();

  // Iterate over edges and add all line points to out vector
  for (const TransformEdgeId& edge_id : edges_) {
    auto start_it = frames_.find(edge_id.start);
    auto end_it = frames_.find(edge_id.end);
    if (start_it == frames_.end() || end_it == frames_.end()) continue;

    const Eigen::Vector3t& start_trans = start_it->second.translation();
    points->push_back(start_trans);

    const Eigen::Vector3t& end_trans = end_it->second.translation();
    points->push_back(end_trans);

    edge_ids->push_back(edge_id);

    auto mit = maps_.find(edge_id.session_id);
    if (mit == maps_.end()) {
      maps_.insert({edge_id.session_id, n_maps});
      session_ids->push_back(n_maps);
      ++n_maps;
    } else {
      session_ids->push_back(mit->second);
    }
  }
  *num_maps = n_maps;
}


bool GlobalMapView::UpdateEdge(const TransformEdgeId& edge,
                               const ReferenceFrameId& a,
                               const ReferenceFrameId& b,
                               const Sophus::SE3t& t_ab) {
  CHECK_EQ(b, edge.OtherEnd(a));
  if (!has_root_id_) {
    std::cerr << "Must set root id of GlobalMapView before UpdateEdge."
              << std::endl;
    return false;
  }

  auto ait = frames_.find(a);
  if (ait == frames_.end()) {
    ROS_ERROR("slam map: Cannot find %d", a.id);
    return false;
  }
  frames_[b] = ait->second * t_ab;
  return true;
}

class LoadWholeMapVisitor : public MapVisitor {
 public:
  LoadWholeMapVisitor(GlobalMapView* v) : view(v) {
    set_has_explore_edge(true);
  }

  void ExploreEdge(const SlamFramePtr& parent,
                   const SlamEdgePtr& edge,
                   const SlamFramePtr& child) override {
    view->AddEdgeInline(edge->id());
  }
  GlobalMapView* view;
};

void GlobalMapView::ResetAndLoadWholeMap() {
  CHECK(has_root_id_) << "Must set_root_id() before loading map";

  Reset(map_);
  LockGuardT lock(mutex_);
  LoadWholeMapVisitor visitor(this);
  visitor.set_root_id(root_id());
  visitor.set_depth(MapVisitor::kMaxDepth);
  visitor.set_should_ignore_broken(true);
  map_->BFS(&visitor);
}

void GlobalMapView::UpdateEdgeId(const TransformEdgeId& id) {
  auto it = edges_.find(id);
  if (it == edges_.end()) return;

  const TransformEdgeId& old_id = *it;

  // If the old start is not the same
  if (old_id.start == id.start) return;
  edges_.erase(it);

  // If the old start is in the tree
  // And has the old end as a child
  auto sit = tree_.find(old_id.start);
  if (sit != tree_.end()) {
    auto eit = sit->second.find(old_id);
    if (eit != sit->second.end()) {
      sit->second.erase(eit);
    }
  } else {
    // If the old end is in the tree
    // And has the old start as a child
    auto eit = tree_.find(old_id.end);
    if (eit != tree_.end()) {
      auto sit = eit->second.find(old_id);
      if (sit != eit->second.end()) {
        eit->second.erase(sit);
      }
    }
  }

  // If new id start is in tree, add edge
  auto new_sit = tree_.find(id.start);
  if (new_sit != tree_.end()) {
    new_sit->second.insert(id);
  } else {
    // Or if new id end is in tree, add to it instead
    auto new_eit = tree_.find(id.end);
    if (new_eit != tree_.end()) {
      new_eit->second.insert(id);
    }
  }
}

void GlobalMapView::UpdateEdgeFromMap(const ReferenceFrameId& a,
                                      const ReferenceFrameId& b) {
  Sophus::SE3t t_parent_child;
  SlamEdgePtr edge = map_->GetEdgePtr(a, b);
  if (edge && edge->transform(a, b, t_parent_child)) {
    UpdateEdge(edge->id(), a, b, t_parent_child);
  }
}

/** Update all children of the given frame */
void GlobalMapView::UpdateFromRoot(const ReferenceFrameId& root,
                    const ReferenceFrameId& dest) {
  std::queue<ReferenceFrameId> to_update;
  std::queue<Sophus::SE3t> pose_updates;
  Sophus::SE3t t_root_dest;
  SlamEdgePtr edge = map_->GetEdgePtr(root, dest);
  if (!edge || !edge->transform(root, dest, t_root_dest)) return;

  Sophus::SE3t t_world_root;
  if (!GetFramePoseInline(root, &t_world_root)) return;

  to_update.push(dest);
  pose_updates.push(t_world_root * t_root_dest);

  Sophus::SE3t t_parent_child;
  for (; !to_update.empty(); to_update.pop(), pose_updates.pop()) {
    const ReferenceFrameId& parent = to_update.front();
    const Sophus::SE3t& t_world_parent_new = pose_updates.front();
    Sophus::SE3t t_world_parent_old;
    GetFramePoseInline(parent, &t_world_parent_old);

    for (const TransformEdgeId& child_edge : tree_[parent]) {
      ReferenceFrameId child = child_edge.OtherEnd(parent);
      Sophus::SE3t t_world_child;
      GetFramePoseInline(child, &t_world_child);

      to_update.push(child);
      pose_updates.push(
          t_world_parent_new * t_world_parent_old.inverse() * t_world_child);
    }
    frames_[parent] = t_world_parent_new;
  }
}
