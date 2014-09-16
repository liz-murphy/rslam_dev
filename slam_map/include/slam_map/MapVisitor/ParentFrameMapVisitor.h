// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <vector>
#include <miniglog/logging.h>
#include <slam_map/MapVisitor/MapVisitor.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdgeId.h>

/**
 * Visitor for gathering parent frames, starting at the root.
 */
class ParentFrameMapVisitor : public MapVisitor {
 public:
  ParentFrameMapVisitor(std::vector<ReferenceFrameId>* nearby) :
      parents_(nearby) {
    set_has_visit(true);
    parents_->clear();
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    ReferenceFrameId cur_id = cur_node->id();
    if (parents_->empty()) {
      CHECK(cur_id == root_id());
      parents_->push_back(cur_id);
    } else if (parents_->back() != cur_id) {
      return false;
    }

    if (cur_node->is_isolated()) return false;

    TransformEdgeId parent_edge_id = cur_node->parent_edge_id();
    if (!parent_edge_id.valid()) return false;

    parents_->push_back(cur_id == parent_edge_id.start ?
                        parent_edge_id.end : parent_edge_id.start);
    return true;
  }

 private:
  std::vector<ReferenceFrameId>* parents_;
};
