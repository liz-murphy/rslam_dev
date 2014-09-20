#include <sparse_front_end/LiftLocalMap.h>

#include <sparse_front_end/InsideNodesMapVisitor.h>
#include <sparse_front_end/OutsideNodesMapVisitor.h>
#include <local_map/LocalMap.h>

namespace rslam {
namespace sparse {

inline void GetNodesInsideWindowBFS(
    const SlamMap* map,
    LocalMap&  local_map,
    const bool ignore_broken,
    const bool poses_only,
    const bool inside_only,
    const bool depth_equals_count,
    bool use_static_set,
    const ReferenceFrameId& ignore_frame) {

  InsideNodesMapVisitor visitor(map, &local_map);
  visitor.set_use_static_set(use_static_set);
  visitor.set_poses_only(poses_only);
  visitor.set_inside_only(inside_only);
  visitor.set_root_id(local_map.root_id);
  visitor.set_depth(local_map.depth);
  visitor.set_should_ignore_broken(ignore_broken);
  visitor.set_depth_equals_count(depth_equals_count);
  visitor.set_ignore_frame(ignore_frame);

  map->BFS(&visitor);

  for (auto& id_pose : local_map.poses) {
    local_map.outside_pose_ids.erase(id_pose.first);
  }

  local_map.ba_root_id = visitor.last_parent();

  // get the poses that were outside the window
  if (poses_only == false && inside_only == false) {
    OutsideNodesMapVisitor visitor(local_map.root_id, &local_map);
    visitor.set_root_id(local_map.root_id);
    visitor.set_depth(map->NumFrames());
    map->BFS(&visitor);
  }

  for (const SessionId& session_id : local_map.maps) {
    if (CameraRigPtr rig = map->GetCamera(session_id)) {
      CHECK(rig) << "No rig found for lifted pose";
      local_map.cameras[session_id] = rig;
    }
  }

  // TODO: This is horribly inefficient due to the massive number of tree
  // traverses when calling local_map.poses.find. There needs to be a better
  // way of assigning measurement containers to the poses when doing the BFS.
  // -NK
  if (!poses_only) {
    for (auto pair : local_map.landmarks) {
      // Search for the pose to which this measurement belongs
      for (std::shared_ptr<MeasurementContainer> meas_container :
               pair.second->measurements) {
        auto pose_it = local_map.poses.find(meas_container->id.frame_id);
        if (pose_it != local_map.poses.end()) {
          pose_it->second.measurements.push_back(meas_container);
        }
      }
    }
  }
}

bool LiftAllPoses(const SlamMap* map,
                  const ReferenceFrameId& root_id,
                  LocalMap& local_map_out,
                  bool ignore_broken,
                  bool poses_only,
                  bool inside_only,
                  bool depth_equals_count) {
  return LiftLocalPoses(
      map,
      std::numeric_limits<decltype(local_map_out.depth)>::max(),
      root_id,
      local_map_out,
      ignore_broken,
      poses_only,
      inside_only,
      depth_equals_count);
}

bool LiftLocalPoses(const SlamMap* map,
                    unsigned int depth,
                    const ReferenceFrameId& root_id,
                    LocalMap&  local_map_out,
                    const bool  ignore_broken,
                    const bool  poses_only,
                    const bool  inside_only,
                    const bool  depth_equals_count,
                    bool use_static_set,
                    const ReferenceFrameId& ignore_frame) {
  local_map_out.Clear();
  local_map_out.map = map;
  local_map_out.depth = depth;
  local_map_out.root_id = root_id;

  // poses are Twp
  GetNodesInsideWindowBFS(map, local_map_out, ignore_broken, poses_only,
                          inside_only, depth_equals_count, use_static_set,
                          ignore_frame);

  local_map_out.num_measurements = 0;

  if (!poses_only) {
    for (auto& pair : local_map_out.landmarks) {
      std::shared_ptr<LandmarkContainer> container = pair.second;

      // compute landmark in root reference frame
      const auto itPose = local_map_out.poses.find(container->id.ref_frame_id);
      if (itPose == local_map_out.poses.end()) {
        return false;
      }

      // keep tally of the number of measurements
      local_map_out.num_measurements += container->measurements.size();
    }
  }
  return true;
}

void LiftLocalMap(const SlamMap* map,
                  unsigned int depth,
                  const ReferenceFrameId& root_id,
                  LocalMap&  local_map_out,
                  const bool  ignore_broken,
                  const bool  poses_only,
                  const bool  inside_only,
                  bool use_static_set) {
  LiftLocalPoses(map, depth, root_id, local_map_out, ignore_broken,
                 poses_only, inside_only, use_static_set);
  local_map_out.PushRelativeCoordsToGlobal();
}
}  // namespace sparse
}  // namespace rslam
