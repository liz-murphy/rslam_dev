// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <slam_map/TransformEdge.h>
#include <HAL/Utils/TicToc.h>
#include <miniglog/logging.h>
#include <slam_map/NotificationCenter.h>

typedef std::lock_guard<std::mutex> LockGuardT;

TransformEdge::TransformEdge() : is_broken_(false),
                                 g_(Eigen::Vector3t::Zero()),
                                 last_modified_time_(hal::Tic()) {
  id_.store(TransformEdgeId());
  is_loop_closure_ = false;
}

TransformEdge::TransformEdge(const TransformEdge& other)
    : is_broken_(other.is_broken_),
      t_se_(other.t_se_),
      g_(other.g_),
      imu_poses_(other.imu_poses_),
      last_modified_time_(other.last_modified_time_.load()) {
  id_.store(other.id_.load());
  is_loop_closure_.store(other.is_loop_closure_.load());
}

TransformEdge::TransformEdge(const TransformEdgeId& id,
                             const Sophus::SE3t& t_se)
    : is_broken_(false),
      t_se_(t_se),
      g_(Eigen::Vector3t::Zero()),
      last_modified_time_(0.) {
  id_.store(id);
  is_loop_closure_ = false;
}

TransformEdge::~TransformEdge() {}

void TransformEdge::Merge(const TransformEdge& other) {
  CHECK_EQ(other.id_.load(), id_.load());
  std::lock_guard<std::mutex> lock(mutex_);
  is_broken_ = other.is_broken_;
  t_se_ = other.t_se_;
  is_loop_closure_.store(other.is_loop_closure_.load());
  MarkAsModified();
}

void TransformEdge::set_transform(const Sophus::SE3t &t_se) {
  LockGuardT lock(mutex_);
  t_se_ = t_se;
  MarkAsModified();
}

bool TransformEdge::set_transform(const ReferenceFrameId& start_id,
                                  const ReferenceFrameId& end_id,
                                  const Sophus::SE3t &t_se) {
  TransformEdgeId edge_id = id_.load();
  LockGuardT lock(mutex_);
  MarkAsModified();
  if (start_id == edge_id.start && end_id == edge_id.end) {
    t_se_ = t_se;
    return true;
  }
  if (end_id == edge_id.start && start_id == edge_id.end) {
    t_se_ = t_se.inverse();
    return true;
  }
  return false;
}

void TransformEdge::transform(Sophus::SE3t* t_se) const {
  CHECK_NOTNULL(t_se);
  *t_se = t_se_;
}

bool TransformEdge::transform(const ReferenceFrameId& start_id,
                              const ReferenceFrameId& end_id,
                              Sophus::SE3t &t_se) const {
  TransformEdgeId edge_id = id_.load();

  LockGuardT lock(mutex_);
  if (start_id == edge_id.start && end_id == edge_id.end) {
    t_se = t_se_;
    return true;
  }
  if (end_id == edge_id.start && start_id == edge_id.end) {
    t_se = t_se_.inverse();
    return true;
  }

  return false;
}

void TransformEdge::SetImuPoses(
    const Eigen::Vector3tArray& poses) {
  LockGuardT lock(mutex_);
  imu_poses_ = poses;
  MarkAsModified();
}

void TransformEdge::SetImuPoses(
    const Eigen::Vector3tArray&& poses) {
  LockGuardT lock(mutex_);
  imu_poses_ = poses;
  MarkAsModified();
}

void TransformEdge::set_is_broken(const bool bBroken) {
  LockGuardT lock(mutex_);
  is_broken_ = bBroken;
  MarkAsModified();
}

bool TransformEdge::is_broken() const {
  LockGuardT lock(mutex_);
  return is_broken_;
}

void TransformEdge::set_start_id(const ReferenceFrameId& id) {
  TransformEdgeId updated_id = id_;
  updated_id.start = id;
  id_.store(updated_id);
  MarkAsModified();
}

void TransformEdge::set_end_id(const ReferenceFrameId& id) {
  TransformEdgeId updated_id = id_;
  updated_id.end = id;
  id_.store(updated_id);
  MarkAsModified();
}

const Eigen::Vector3tArray& TransformEdge::GetImuPoses() const {
  LockGuardT lock(mutex_);
  return imu_poses_;
}

void TransformEdge::set_id(const TransformEdgeId& id) {
  id_ = id;
  MarkAsModified();
}

void TransformEdge::MarkAsModified() {
  last_modified_time_ = hal::Tic();

  if (notification_center_) {
    notification_center_->Notify(rslam::map::kUpdateEdgeMapEvent, id_);
  }
}

void TransformEdge::set_g(const Eigen::Vector3t& g) {
  std::lock_guard<std::mutex> lock(mutex_);
  g_ = g;
  MarkAsModified();
}
