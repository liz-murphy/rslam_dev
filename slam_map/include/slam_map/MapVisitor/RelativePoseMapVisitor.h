// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <miniglog/logging.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>

/** Find the transform from the root node to a given node */
class RelativePoseMapVisitor : public TransformMapVisitor {
 public:
  RelativePoseMapVisitor(const ReferenceFrameId& other)
      : relative_pose_found_(false), other_(other) {
    set_has_visit(true);
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    if (relative_pose_found_) return false;

    TransformMapVisitor::Visit(cur_node);
    if (cur_node->id() == other_) {
      relative_pose_found_ = true;
      relative_pose_ = CurT();
    }
    return true;
  }

  bool relative_pose_found() const {
    return relative_pose_found_;
  }

  bool relative_pose(Sophus::SE3t* t_ab) const {
    CHECK(t_ab);
    if (relative_pose_found_) {
      *t_ab = relative_pose_;
    }
    return relative_pose_found_;
  }

  bool IsDone() override {
    return relative_pose_found_;
  }

 private:
  bool relative_pose_found_;
  Sophus::SE3t relative_pose_;
  ReferenceFrameId other_;
};
