#include <sparse_front_end/PushLocalMap.h>
#include <local_map/LocalMap.h>
#include <slam_map/SlamMap.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/ReferenceFrame.h>

namespace rslam {
namespace sparse {
void UpdateRelativeNodes(
    const std::map<ReferenceFrameId, PoseContainerT> &absolute_poses,
    SlamMap* map) {
  Sophus::SE3t t_ab;
  bool is_curr_pose_a;
  ReferenceFrameId neighbor_node_id;
  std::unordered_set<ReferenceFrameId> seen_nodes;

  for (auto it_curr = absolute_poses.begin();
       it_curr != absolute_poses.end(); ++it_curr) {
    SlamFramePtr cur_node = map->GetFramePtr(it_curr->first);
    if (!cur_node) continue;

    ReferenceFrameId frame_id = cur_node->id();

    // not that v_w (and g) here should be transformed to v_r via the
    // PushGlobalCoordsToRelative function before calling this function
    if (!it_curr->second.only_pose_optimized) {
      cur_node->set_t_vs(it_curr->second.t_vs);
      cur_node->set_g_r(it_curr->second.g);
      cur_node->set_v_r(it_curr->second.v_w);
      cur_node->set_b(it_curr->second.b);
      cur_node->set_cam_params(it_curr->second.cam_params);
    }
    seen_nodes.insert(frame_id);

    // Explore neighbours
    for (unsigned int ii = 0; ii < cur_node->NumNeighbors(); ++ii) {
      TransformEdgeId neighbor_edge_id = cur_node->GetNeighborEdgeId(ii);
      SlamEdgePtr neighbor_edge  = map->GetEdgePtr(neighbor_edge_id);
      if (!neighbor_edge) continue;

      if (neighbor_edge->start_id() == cur_node->id()) {
        neighbor_node_id = neighbor_edge->end_id();
        is_curr_pose_a = true;
      } else {
        neighbor_node_id = neighbor_edge->start_id();
        is_curr_pose_a = false;
      }

      auto it_neighbour = absolute_poses.find(neighbor_node_id);

      // only update relative edges between optimized poses
      if (it_neighbour == absolute_poses.end()) continue;

      SlamFramePtr neighbor_node = map->GetFramePtr(neighbor_node_id);
      if (neighbor_node && !seen_nodes.count(neighbor_node_id)) {
        // compute edge relative transformation
        if (is_curr_pose_a) {
          t_ab = it_curr->second.t_wp.inverse() * it_neighbour->second.t_wp;
        } else {
          t_ab = it_neighbour->second.t_wp.inverse() * it_curr->second.t_wp;
        }
        neighbor_edge->set_transform(t_ab);
        seen_nodes.insert(neighbor_node_id);
      }
    }
  }
}

void PushLocalMap(SlamMap* map, LocalMap& local_map) {
  // update landmarks relative 3d positions
  for (auto& pair : local_map.landmarks) {
    std::shared_ptr<LandmarkContainer> container = pair.second;
    Landmark lm;
    map->GetLandmark(container->id, &lm);

    lm.set_xrp(container->x_r);
    lm.set_state(container->state);

    const Eigen::Vector4t x_c =
        Sophus::MultHomogeneous(
            map->GetCamera(map->id())->cameras[lm.base_camera()].T_wc.inverse(),
            container->x_r);

    // make sure that if we move a landmark during BA, its extent is
    // adjusted accordingly
    // lm.SetExtent(lm.Extent()*container->depth_ratio);
    lm.set_extent(lm.cam_plane_extent() * x_c.norm());
    // lm.SetOrientation(container->orientation_3d);

    // Now set the landmark on the frame again
    if (SlamFramePtr frame = map->GetFramePtr(container->id.ref_frame_id)) {
      frame->SetLandmark(container->id.landmark_index, lm);
    }
  }

  // update relative vehicle poses
  UpdateRelativeNodes(local_map.poses, map);
}
}  // namespace sparse
}  // namespace rslam
