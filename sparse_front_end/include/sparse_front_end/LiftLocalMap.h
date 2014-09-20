// Copyright (c) Jack Morrison, Nima Keivan, Gabe Sibley, all rights
// reserved. See the accompanying LICENSE file for more information.

#pragma once

#include <slam_map/SlamMapFwd.h>
#include <slam_map/ReferenceFrameId.h>
#include <local_map/LocalMap.h>

namespace rslam {
namespace sparse {

bool LiftAllPoses(const SlamMap* map,
                  const ReferenceFrameId& root_id,
                  LocalMap&  local_map_out,
                  bool  ignore_broken = false,
                  bool  poses_only = false,
                  bool  inside_only = false,
                  bool  depth_equals_count = false);

bool LiftLocalPoses(const SlamMap* map,
                    unsigned int depth,
                    const ReferenceFrameId& root_id,
                    LocalMap&  local_map_out,
                    const bool  ignore_broken = false,
                    const bool  poses_only = false,
                    const bool  inside_only = false,
                    const bool  depth_equals_count = false,
                    bool use_static_set = false,
                    // A frame through which progression of the BFS
                    // should not continue. This is to allow start a
                    // BFS at a node and NOT traveling a certain
                    // direction, e.g. towards the most recent frame
                    // from the reference frame
                    const ReferenceFrameId& ignore_frame =
                    ReferenceFrameId());

void LiftLocalMap(const SlamMap* map,
                  unsigned int depth,
                  const ReferenceFrameId& root_id,
                  LocalMap&  local_map_out,
                  const bool  ignore_broken = false,
                  const bool  poses_only = false,
                  const bool  inside_only = false,
                  bool use_static_set = false);
}  // namespace sparse
}  // namespace rslam
