// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <tuple>
#include <Utils/MathTypes.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>

class GatherFrameObjectsMapVisitor : public MapVisitor {
 public:
  typedef std::tuple<ReferenceFrameId, std::shared_ptr<FrameObject> > Result;
  GatherFrameObjectsMapVisitor(std::vector<Result>* out)
      : found_(out) {
    set_has_visit(true);
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    if (cur_node->NumObjects() > 0) {
      std::shared_ptr<FrameObject> obj;
      for (size_t i = 0; i < cur_node->NumObjects(); ++i) {
        cur_node->GetObject(i, &obj);
        found_->emplace_back(cur_node->id(), obj);
      }
    }
    return true;
  }

  virtual ~GatherFrameObjectsMapVisitor() {}

 private:
  std::vector<Result>* found_;
};
