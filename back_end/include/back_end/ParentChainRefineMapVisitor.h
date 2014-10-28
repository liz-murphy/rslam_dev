// Copyright (c) John Morrison, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <unordered_map>
#include <unordered_set>

#include <ba/InterpolationBuffer.h>
#include <ba/Types.h>
#include <slam_map/MapVisitor/MapVisitor.h>
#include <slam_map/pose_measurement.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>
#include <ros/ros.h>
/** Visitor to gather active frame set for BackEnd::RefineMap */
template <typename BundleAdjuster>
class ParentChainRefineMapVisitor : public MapVisitor {
  typedef ba::InterpolationBufferT<ba::ImuMeasurementT<Scalar>, Scalar>
  InterpolationBuffer;
 public:

  /**
   * Create a visitor.
   *
   * @param ba The BundleAdjuster to add poses & constraints to.
   * @param imu_buffer Optional IMU measurement buffer for adding IMU
   *                   constraints. Pass nullptr to ignore.
   * @param ba_frames Hash for recording frame -> ba_id #'s.
   * @param landmarks Set of landmarks observed by active poses.
   */
  ParentChainRefineMapVisitor(
      BundleAdjuster* ba,
      InterpolationBuffer* imu_buffer,
      std::unordered_map<ReferenceFrameId, uint32_t>* ba_frames,
      std::unordered_map<TransformEdgeId, uint32_t>* edge_ba_ids,
      std::unordered_map<LandmarkId, uint32_t>* landmarks,
      TransformEdgeId* root_edge_id,
      ReferenceFrameId* root_pose_id)
      : ba_(ba), imu_buffer_(imu_buffer), ba_frames_(ba_frames),
        edge_ba_ids_(edge_ba_ids), landmarks_(landmarks),
        root_pose_id_(root_pose_id), root_edge_id_(root_edge_id) {
    set_has_explore_node(true);
  }

  bool ExploreNode(const SlamFramePtr& parent,
                   const SlamEdgePtr& edge,
                   const SlamFramePtr& child) override {
    const ReferenceFrameId parent_id = parent->id();
    const ReferenceFrameId child_id = child->id();

    if (num_poses_seen_ == 0) {
      AddFrame(t_cur_, parent_id, parent, !disable_poses_);
      ++num_poses_seen_;
    }

    Sophus::SE3t t_pc;
    if (!edge->transform(parent_id, child_id, t_pc)) return false;
    t_cur_ *= t_pc;

    bool is_root = false;

    // Child has no parent or we've seen enough, so add a root.
    TransformEdgeId child_precursor = child->parent_edge_id();
    bool no_parent = !child_precursor.valid();
    if (no_parent || num_poses_seen_ == depth()) {
      is_root = true;
    }
    AddFrame(t_cur_, child_id, child, !disable_poses_);

    // Fill in the root pose details.
    if (is_root) {
      ba_->SetRootPoseId(ba_frames_->at(child_id));
      if (imu_buffer_) {
        ba_->SetGravity(t_cur_.so3() * child->g_r());
      }
      if (root_pose_id_) {
        *root_pose_id_ = child_id;
      }
    }

    // Add IMU residuals
    uint32_t edge_ba = UINT_MAX;
    if (!disable_poses_ && imu_buffer_) {
      std::vector<ba::ImuMeasurementT<Scalar> > meas =
          imu_buffer_->GetRange(child->time(), parent->time());
      if (meas.empty()) {
        ROS_ERROR("No measurements in residual from %f to %f, buffer start t: %f, end time: %f, num elements: %d",
            child->time(),
            parent->time(),
            imu_buffer_->start_time,
            imu_buffer_->end_time,
            (int)imu_buffer_->elements.size());
      }
      edge_ba = ba_->AddImuResidual(
          ba_frames_->at(child_id), ba_frames_->at(parent_id), meas,
          (is_root && !no_parent) ? imu_prior_weight_ : imu_weight_);
      if (is_root && root_edge_id_) {
        *root_edge_id_ = edge->id();
      }
    }
    edge_ba_ids_->emplace(edge->id(), edge_ba);

    // Keep our own depth count
    ++num_poses_seen_;
    last_child_ = child;
    return !is_root;
  }

  void Finished() override {
    if (ba_->GetRootPoseId() == 0 && last_child_) {
      ba_->SetRootPoseId(ba_frames_->at(last_child_->id()));
    }
  }

  void set_imu_weight(Scalar weight) {
    imu_weight_ = weight;
  }

  void set_imu_prior_weight(Scalar weight) {
    imu_prior_weight_ = weight;
  }

  /**
   * Disable pose adjustment (set to inactive in BA).
   *
   * Used when initializing landmarks to set all poses to inactive.
   */
  void set_disable_poses(bool should_disable) {
    disable_poses_ = should_disable;
  }

 protected:
  inline void AddFrame(const Sophus::SE3t& t_wp,
                       const ReferenceFrameId& id,
                       const SlamFramePtr& frame,
                       bool is_active) {
    uint32_t ba_id = UINT_MAX;
    if (imu_buffer_) {
      ba_id = ba_->AddPose(t_wp,
                           frame->t_vs(),
                           frame->cam_params(),
                           t_wp.so3() * frame->v_r(),
                           frame->b(),
                           is_active);
    } else {
      ba_id = ba_->AddPose(t_wp, is_active);
    }

    ba_frames_->emplace(id, ba_id);
    AddObservedLandmarks(frame);
    AddPoseMeasurements(ba_id, frame);
  }

  inline void AddObservedLandmarks(const SlamFramePtr& frame) {
    size_t num_meas = frame->NumMeasurements();
    MeasurementId zid;
    for (size_t i = 0; i < num_meas; ++i) {
      if (frame->HasGoodMeasurement(i) && frame->GetMeasurementId(i, &zid)) {
        landmarks_->emplace(zid.landmark_id, UINT_MAX);
      }
    }
  }

  inline void AddPoseMeasurements(uint32_t ba_id, const SlamFramePtr& frame) {
    size_t num_pose_meas = frame->NumPoseMeasurements();
    rslam::map::PoseMeasurement z;
    for (size_t i = 0; i < num_pose_meas; ++i) {
      if (frame->GetPoseMeasurement(i, &z)) {
        ba_->AddUnaryConstraint(ba_id, z.t_wv, z.cov);
      }
    }
  }

 private:
  BundleAdjuster* ba_;
  InterpolationBuffer* imu_buffer_;
  std::unordered_map<ReferenceFrameId, uint32_t>* ba_frames_;
  std::unordered_map<TransformEdgeId, uint32_t>* edge_ba_ids_;
  std::unordered_map<LandmarkId, uint32_t>* landmarks_;
  ReferenceFrameId* root_pose_id_;
  TransformEdgeId* root_edge_id_;
  Sophus::SE3t t_cur_;

  SlamFramePtr last_child_;

  // The number of poses added so far in the search
  size_t num_poses_seen_ = 0;
  Scalar imu_weight_ = 1e3;
  Scalar imu_prior_weight_ = 10.0;
  bool disable_poses_ = false;
};
