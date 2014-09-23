// Copyright (c) John Morrison, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <slam_map/MapVisitor/TransformMapVisitor.h>
#include <sparse_front_end/FrontEndConfig.h>
//#include <sparse_front_end/FrontEndCVars.h>

/**
 * Find the closest keyframe to the root that can be used as a
 * relocalization keyframe.
 */
class ClosestKeyframeMapVisitor : public TransformMapVisitor {
 public:
  ClosestKeyframeMapVisitor() :
      closest_total_(std::numeric_limits<Scalar>::max()) {
    set_has_visit(true);
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    TransformMapVisitor::Visit(cur_node);
    if (cur_node->id() == root_id()) return true;

    const Sophus::SE3t& cur_pose = CurT();
    Scalar dist_traveled = cur_pose.translation().norm();
    Scalar angle_difference = cur_pose.so3().log().maxCoeff() * 180 / M_PI;

    Scalar angle_score = angle_difference / FrontEndConfig::getConfig()->keyframe_max_angle;
    Scalar dist_score = dist_traveled / FrontEndConfig::getConfig()->keyframe_max_distance;

    Scalar total = angle_score + dist_score;
    if (total < closest_total_) {
      closest_total_ = total;
      closest_keyframe_ = cur_node->id();
    }
    return true;
  }

  ReferenceFrameId closest_keyframe() const {
    return closest_keyframe_;
  }

  void relative_pose(Sophus::SE3t* t_ab) const {
    CHECK(t_ab);
    *t_ab = relative_pose_;
  }

 private:
  Sophus::SE3t relative_pose_;
  ReferenceFrameId closest_keyframe_;
  Scalar closest_total_;
};
