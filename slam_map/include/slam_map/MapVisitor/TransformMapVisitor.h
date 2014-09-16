// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <queue>
#include <slam_map/MapVisitor/MapVisitor.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>

class TransformMapVisitor : public MapVisitor {
 public:
  TransformMapVisitor() {
    transform_queue.push(Sophus::SE3t());
    set_has_explore_node(true);
    set_has_visit(true);
  }

  bool ExploreNode(const SlamFramePtr& parent,
                   const SlamEdgePtr& edge,
                   const SlamFramePtr& child) override {
    edge->transform(parent->id(), child->id(), t_parent_from_child);
    transform_queue.push(t_cur * t_parent_from_child);
    return true;
  }

  bool Visit(const SlamFramePtr& /*cur_node*/) override {
    t_cur = transform_queue.front();
    transform_queue.pop();
    return true;
  }

 protected:
  const Sophus::SE3t& CurT() const {
    return t_cur;
  }

 private:
  std::queue<Sophus::SE3t> transform_queue;
  Sophus::SE3t t_cur;
  Sophus::SE3t t_parent_from_child;
};
