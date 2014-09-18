// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

///   \file TransformEdge.h defines an edge in a slam_map and the
///         associated Transform.

#pragma once

#include <atomic>
#include <mutex>
#include <sophus/se3.hpp>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/ReferenceFrameId.h>
#include <slam_map/TransformEdgeId.h>
#include <utils/MathTypes.h>

class TransformEdge {
 public:
  TransformEdge();
  TransformEdge(const TransformEdgeId& id, const Sophus::SE3t& t_se);
  TransformEdge(const TransformEdge& other);
  ~TransformEdge();

  void Merge(const TransformEdge& other);

  TransformEdgeId id() const {
    return id_;
  }

  ReferenceFrameId end_id() const {
    return id_.load().end;
  }

  ReferenceFrameId start_id() const {
    return id_.load().start;
  }

  Eigen::Vector3t g() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return g_;
  }

  bool is_broken() const;
  void transform(Sophus::SE3t* t_se) const;
  bool transform(const ReferenceFrameId& start_id,
                 const ReferenceFrameId& end_id,
                 Sophus::SE3t &t_se) const;
  const Eigen::Vector3tArray& GetImuPoses() const;

  double last_modified_time() const {
    return last_modified_time_;
  }

  bool is_loop_closure() const {
    return is_loop_closure_;
  }

  void set_id(const TransformEdgeId& id);
  void set_is_broken(bool broken);
  void set_g(const Eigen::Vector3t& g);
  void set_start_id(const ReferenceFrameId& id);
  void set_end_id(const ReferenceFrameId& id);
  void set_transform(const Sophus::SE3t &t_se);
  bool set_transform(const ReferenceFrameId& start_id,
                     const ReferenceFrameId& end_id,
                     const Sophus::SE3t &t_se);
  void SetImuPoses(const Eigen::Vector3tArray& poses);
  void SetImuPoses(const Eigen::Vector3tArray&& poses);

  void set_last_modified_time(double t) {
    last_modified_time_ = t;
  }

  void set_is_loop_closure(bool t) {
    is_loop_closure_ = t;
  }

  void set_notification_center(
      const std::shared_ptr<rslam::map::NotificationCenter>& notifier) {
    notification_center_ = notifier;
  }

  /** Mark this edge as having been modified at the time this was called. */
  void MarkAsModified();

 private:
  std::atomic<TransformEdgeId> id_;
  bool is_broken_;
  std::atomic<bool> is_loop_closure_;
  Sophus::SE3t t_se_;          // transform
  Eigen::Vector3t g_;
  Eigen::Vector3tArray imu_poses_;

  // Timestamp of last modification to edge
  std::atomic<double> last_modified_time_;

  std::shared_ptr<rslam::map::NotificationCenter> notification_center_;

  mutable std::mutex mutex_;
};
