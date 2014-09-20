// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <local_map/LocalMap.h>
#include <slam_map/SlamMap.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>

class InsideNodesMapVisitor : public TransformMapVisitor {
 public:
  InsideNodesMapVisitor(const SlamMap* const map,
                        LocalMap* out) : poses_only_(false),
                                         inside_only_(false),
                                         use_static_set_(false),
                                         local_map_(out),
                                         map_(map) {
    set_has_explore_edge(true);
    set_has_visit(true);
  }

  void set_poses_only(bool only) {
    poses_only_ = only;
  }

  void set_inside_only(bool only) {
    inside_only_ = only;
  }

  void set_use_static_set(bool use_static) {
    use_static_set_ = use_static;
  }

  void set_ignore_frame(const ReferenceFrameId& ignore) {
    ignore_frame_ = ignore;
  }

  /** Find the deepest node along the parent chain. */
  ReferenceFrameId last_parent() const {
    return last_parent_;
  }

  void ExploreEdge(const SlamFramePtr& parent,
                   const SlamEdgePtr& edge,
                   const SlamFramePtr& child) override {
    TransformEdgeId edge_id = edge->id();

    if (ignore_frame_.valid() &&
        (ignore_frame_ == edge_id.start ||
         ignore_frame_ == edge_id.end)) {
      return;
    }

    SessionId session_id = map_->id();
    if ((session_id == parent->id().session_id &&
         session_id != child->id().session_id) ||
        (session_id == child->id().session_id &&
         session_id != parent->id().session_id)) {
      local_map_->active_static_edges.push_back(edge_id);
    }

    if (edge_id.session_id == session_id &&
        edge_id.start.session_id == session_id &&
        edge_id.end.session_id == session_id) {
      local_map_->inside_edge_ids[edge_id] = edge_id;
    }
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    TransformMapVisitor::Visit(cur_node);

    ReferenceFrameId id = cur_node->id();
    if (ignore_frame_.valid() &&
        ignore_frame_ == id) {
      return false;
    }

    if (last_parent_.valid()) {
      last_parent_ = next_parent_ = id;
    }

    if (next_parent_ == id) {
      local_map_->parent_chain_ids.insert(id);
      last_parent_ = id;
      next_parent_ = cur_node->parent_edge_id().OtherEnd(id);
    }

    // Transform to apply on left of landmark x_r to put landmark in
    // pose frame. Only really needed for other session's poses.
    Sophus::SE3t pose_from_lmk;
    PoseContainerT pose(CurT(),
                        cur_node->t_vs(),
                        cur_node->cam_params(),
                        cur_node->v_r(),
                        Eigen::Vector3t::Zero(),
                        cur_node->b(),
                        cur_node->g_r(),
                        cur_node->time());

    PoseContainerT* ref;
    if (use_static_set_ && id.session_id != map_->id()) {
      if (!local_map_->static_set_pose) {
        pose.is_static_set_pose = true;
        local_map_->static_set_pose.reset(new PoseContainerT(pose));
        local_map_->static_set_pose_id = id;
      } else {
        pose_from_lmk = local_map_->static_set_pose->t_wp.inverse() * CurT();
        local_map_->static_poses.insert({id, pose_from_lmk});
      }
      ref = local_map_->static_set_pose.get();
    } else {
      local_map_->poses[id] = pose;
      ref = &local_map_->poses[id];
    }
    local_map_->maps.insert(id.session_id);

    if (!poses_only_) {
      AddNonPoses(map_->id(),
                  cur_node,
                  pose_from_lmk,
                  (ref->is_static_set_pose ?
                   &local_map_->static_set_pose_id : nullptr),
                  *ref);
    }
    return true;
  }

  /** @param lmk_transform Transformation to apply to landmarks before
   * adding to container
   *
   * @param lmk_frame_id Optional frame id to set on landmarks
   */
  void AddNonPoses(const SessionId& current_session_id,
                   const SlamFramePtr& cur_node,
                   const Sophus::SE3t& lmk_transform,
                   const ReferenceFrameId* lmk_frame_id,
                   PoseContainerT& container) {
    size_t num_meas = cur_node->NumMeasurements();
    Landmark lmk;
    MeasurementId zid;

    // NB: There is commmented out cerr code in this function, as the
    // implementation is still inefficient and needs to be updated. The cerr
    // code should be used to verify that the correct information is lifted
    // to the map, as a small error might be extremely difficult to catch, and
    // subtly deteriorate the solution.
    for (size_t zi = 0; zi < num_meas; ++zi) {
      if (!cur_node->HasGoodMeasurement(zi) ||
          !cur_node->GetMeasurementId(zi, &zid)) {
        continue;
      }

      // get measurement corresponding landmark
      LandmarkId lmk_id = zid.landmark_id;
      if (lmk_frame_id &&
          lmk_id.ref_frame_id.session_id != current_session_id) {
        lmk_id.ref_frame_id = *lmk_frame_id;
      }

      // if we have indicated we want only inside nodes/landmarks, don't
      // include landmarks that were initialized outside the
      // window we require.
      if (inside_only_ && lmk_id.ref_frame_id != cur_node->id()) {
        continue;
      }

      std::shared_ptr<LandmarkContainer> lmk_ref;

      auto lmk_it = local_map_->landmarks.find(lmk_id);
      if (lmk_it == local_map_->landmarks.end()) {
        if (!map_->GetLandmark(zid.landmark_id, &lmk)) continue;

        lmk_ref = local_map_->landmarks[lmk_id] =
            std::make_shared<LandmarkContainer>(lmk, map_);
        lmk_ref->x_r = Sophus::MultHomogeneous(lmk_transform, lmk_ref->x_r);
        if (lmk_frame_id) {
          lmk_ref->id.ref_frame_id = *lmk_frame_id;
        }

        // save id's of the frames where the landmark was observed
        for (const std::shared_ptr<MeasurementContainer>& zContainer :
                 local_map_->landmarks[lmk_id]->measurements) {

          if (zContainer->id.frame_id.session_id != current_session_id) {
            continue;
          }

          auto it = local_map_->outside_pose_ids.find(zContainer->id.frame_id);
          if (it == local_map_->outside_pose_ids.end()) {
            local_map_->outside_pose_ids.insert(zContainer->id.frame_id);
          }
        }
      } else {
        lmk_ref = lmk_it->second;
      }

      // Now that we have the pointer to the landmark container, see if we need
      // to add this landmark to the pose container, if the reference frame
      // matches that of the pose.
      if (lmk_ref->id.ref_frame_id == cur_node->id()) {
        container.ref_landmarks.push_back(lmk_ref);
      }
    }
  }

 private:
  bool poses_only_, inside_only_, use_static_set_;
  ReferenceFrameId last_parent_, next_parent_, ignore_frame_;
  LocalMap* local_map_;
  const SlamMap* map_;
};
