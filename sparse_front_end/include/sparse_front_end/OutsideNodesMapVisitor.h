// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <local_map/LocalMap.h>
#include <slam_map/SlamMap.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>

class OutsideNodesMapVisitor : public TransformMapVisitor {
 public:
  OutsideNodesMapVisitor(const ReferenceFrameId& last_id,
                         LocalMap* out) : last_frame_id_(last_id),
                                          local_map_(out),
                                          visited(0) {
    set_has_visit(true);
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    TransformMapVisitor::Visit(cur_node);

    ReferenceFrameId cur_node_id = cur_node->id();
    if (cur_node_id == last_frame_id_) {
      return true;
    }

    auto it = local_map_->outside_pose_ids.find(cur_node_id);
    if (it != local_map_->outside_pose_ids.end()) {
      PoseContainerT& pose_container = (local_map_->poses[cur_node_id] =
          PoseContainerT(CurT(),
                         cur_node->t_vs(),
                         cur_node->cam_params(),
                         cur_node->v_r(),
                         Eigen::Matrix<Scalar, 3, 1>::Zero(),
                         cur_node->b(),
                         cur_node->g_r(),
                         cur_node->time()));
      local_map_->maps.insert(cur_node_id.session_id);

      // Get all the ref landmarks and assign them to this pose
      MeasurementId zid;
      size_t num_meas = cur_node->NumMeasurements();
      for (size_t zi = 0; zi < num_meas; ++zi) {
        if (!cur_node->HasGoodMeasurement(zi) ||
            !cur_node->GetMeasurementId(zi, &zid)) {
          continue;
        }

        const auto lmkIt = local_map_->landmarks.find(zid.landmark_id);
        if (lmkIt != local_map_->landmarks.end()) {
          const std::shared_ptr<LandmarkContainer>& lmk_ref = lmkIt->second;
          if (lmk_ref->id.ref_frame_id.session_id == local_map_->map->id() &&
              lmk_ref->id.ref_frame_id == cur_node->id()) {
            pose_container.ref_landmarks.push_back(lmk_ref);
          }
        }
      }
      ++visited;
    }
    return true;
  }

  bool IsDone() {
    return visited >= local_map_->outside_pose_ids.size();
  }

 private:
  ReferenceFrameId last_frame_id_;
  LocalMap* local_map_;
  uint64_t visited;
};
