// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <set>
#include <slam_map/MapVisitor/MapVisitor.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/SlamMap.h>

class NearbyNodesMapVisitor : public MapVisitor {
 public:
  NearbyNodesMapVisitor(const SlamMap* const map,
                        std::set<ReferenceFrameId>* nearby,
                        bool with_covisible) :
      map_(map),
      with_covisible_(with_covisible),
      nearby_(nearby) {
    set_has_visit(true);
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    nearby_->insert(cur_node->id());
    if (map_ && with_covisible_) {
      size_t num_meas = cur_node->NumMeasurements();
      for (size_t zi = 0; zi < num_meas; ++zi) {
        if (!cur_node->HasGoodMeasurement(zi) ||
            !cur_node->GetMeasurementId(zi, &zid_)) {
          continue;
        }

        // get measurement corresponding landmark
        if (!map_->GetLandmark(zid_.landmark_id, &lmk_)) continue;
        for (const MeasurementId& zid2 : lmk_.GetFeatureTrackRef()) {
          // Add frame from each zid to outset
          nearby_->insert(zid2.frame_id);
        }
      }
    }
    return true;
  }

 private:
  const SlamMap* const map_;
  bool with_covisible_;
  std::set<ReferenceFrameId>* nearby_;

  MeasurementId zid_;
  Landmark lmk_;
};
