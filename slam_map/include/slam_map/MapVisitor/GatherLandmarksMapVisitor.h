// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <vector>
#include <slam_map/Landmark.h>
#include <slam_map/MapVisitor/MapVisitor.h>
#include <slam_map/ReferenceFrame.h>

class GatherLandmarksMapVisitor : public MapVisitor {
 public:
  GatherLandmarksMapVisitor(std::vector<Landmark>* lms) : landmarks_(lms) {
    set_has_visit(true);
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    size_t cur_size = landmarks_->size();
    size_t nlandmarks = cur_node->NumLandmarks();
    landmarks_->resize(cur_size + nlandmarks);

    for (size_t i = 0; i < nlandmarks; ++i) {
      cur_node->GetLandmark(i, &landmarks_->at(cur_size + i));
    }
    return true;
  }

 private:
  std::vector<Landmark>* landmarks_;
};
