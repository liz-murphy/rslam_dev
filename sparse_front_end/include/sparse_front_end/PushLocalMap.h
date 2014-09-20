// Copyright (c) Jack Morrison, Nima Keivan, Gabe Sibley, all rights
// reserved. See the accompanying LICENSE file for more information.

#pragma once

#include <slam_map/SlamMapFwd.h>
#include <local_map/LocalMap.h>

namespace rslam {
namespace sparse {
void PushLocalMap(SlamMap* map, LocalMap& local_map);
}  // namespace sparse
}  // namespace rslam
