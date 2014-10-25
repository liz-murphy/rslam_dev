// Copyright (c) John Morrison, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <unordered_map>
#include <unordered_set>

//#include <CVars/CVar.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/SlamMap.h>
#include <slam_map/TransformEdge.h>

#include <back_end/BackEndConfig.h>
#include <ros/ros.h>
/** Visitor to gather active frame set for BackEnd::RefineMap */
template <typename BundleAdjuster>
class GatherLandmarkFramesRefineMapVisitor : public TransformMapVisitor {
 public:

  /**
   * Create a visitor.
   *
   * @param ba The BundleAdjuster to add poses & constraints to.
   * @param ba_frames Hash for recording frame -> ba_id #'s.
   * @param landmarks Set of landmarks observed by active poses. uint32_t is to
   *                  be filled in with the BA Id # or UINT_MAX if not added.
   */
  GatherLandmarkFramesRefineMapVisitor(
      bool init_new_landmarks,
      const SlamMap* map,
      BundleAdjuster* ba,
      std::unordered_map<ReferenceFrameId, uint32_t>* ba_frames,
      std::unordered_map<LandmarkId, uint32_t>* landmarks,
      int min_lm_observations)
      : map_(map), ba_(ba), ba_frames_(ba_frames), landmarks_(landmarks) {
    CHECK(map);
    CHECK(ba);
    CHECK(ba_frames);
    CHECK(landmarks);
    set_has_visit(true);

    std::vector<LandmarkId> to_delete;
    for (auto it = landmarks->begin(); it != landmarks->end(); ) {
      const LandmarkId& lm_id = it->first;
      SlamFramePtr frame = map_->GetFramePtr(lm_id.ref_frame_id);
      if (!frame) {
        ++it;
        continue;
      }

      LandmarkState lm_state;
      const std::vector<MeasurementId>* track = nullptr;
      if (!frame->GetLandmarkFeatureTrackRef(lm_id.landmark_index, track) ||
          !track ||
          !frame->GetLandmarkState(lm_id.landmark_index, &lm_state)) {
        ++it;
        ROS_DEBUG_NAMED("back_end","Can't find landmark %s",boost::lexical_cast<std::string>(lm_id).c_str());
        continue;
      }

      /*static int& min_lm_observations =
          CVarUtils::CreateGetCVar<int>("tracker.MinLandmarkObservations", 3);*/
      bool should_delete = false;
      if (lm_state == eLmkAtInfinity) {
        // We require a minimum number of observations before adding it
        if (track->size() < min_lm_observations) {
          should_delete = true;
        }
      } else {
        // If the landmark already has a depth, we require more than one land
        if (track->size() == 1 || init_new_landmarks) {
          should_delete = true;
        }
      }

      if (should_delete) {
        landmarks->erase(it++);
        continue;
      }

      for (const MeasurementId& zid : *track) {
        if (!ba_frames_->count(zid.frame_id)) {
          to_find_.insert(zid.frame_id);
        }
      }
      if (!ba_frames_->count(lm_id.ref_frame_id)) {
        to_find_.insert(lm_id.ref_frame_id);
      }
      ++it;
    }
  }

  bool Visit(const SlamFramePtr& frame) override {
    TransformMapVisitor::Visit(frame);
    const ReferenceFrameId id = frame->id();
    if (to_find_.count(id)) {
      if (root_time_ == 0.0) {
        SlamFramePtr root_frame = map_->GetFramePtr(root_id());
        root_time_ = root_frame->time();
      }

      if (root_time_ < frame->time()) {
        to_find_.erase(id);
        return false;
      }

      AddPose(CurT(), id, frame, false);
      to_find_.erase(id);

      if (!camera_ids_.count(id.session_id)) {
        if (CameraRigPtr rig = map_->GetCamera(id.session_id)) {
          for (size_t i = 0; i < rig->cameras.size(); ++i) {
            camera_ids_[id.session_id].push_back(
                ba_->AddCamera(rig->cameras[i].camera, rig->cameras[i].T_wc));
          }
        }
      }
    }
    return true;
  }

  bool IsDone() override {
    return to_find_.empty();
  }

  void Finished() override {
    // Add root camera
    ReferenceFrameId root = root_id();
    if (!camera_ids_.count(root.session_id)) {
      if (CameraRigPtr rig = map_->GetCamera(root.session_id)) {
        for (size_t i = 0; i < rig->cameras.size(); ++i) {
          camera_ids_[root.session_id].push_back(
              ba_->AddCamera(rig->cameras[i].camera, rig->cameras[i].T_wc));
        }
      }
    }

    SlamFramePtr root_frame = map_->GetFramePtr(root);
    if (!root_frame) return;

    MultiViewMeasurement z;
    SlamFramePtr frame;
    for (auto it = landmarks_->begin(); it != landmarks_->end(); ++it) {
      const LandmarkId& lm_id = it->first;
      uint32_t& lm_ba_id = it->second;

      // Set to unused for now, to simplify exit code.
      /** @todo Get rid of this UINT_MAX b.s. and use a variable. */
      lm_ba_id = UINT_MAX;
      if (to_find_.count(lm_id.ref_frame_id)) {
        continue;
      }

      frame = map_->GetFramePtr(lm_id.ref_frame_id);
      if (!frame) {
        ROS_WARN("Cannot GetFramePtr for %s", boost::lexical_cast<std::string>(lm_id.ref_frame_id).c_str());
        continue;
      }

      auto ba_id_it = ba_frames_->find(lm_id.ref_frame_id);
      if (ba_id_it == ba_frames_->end()) {
        ROS_WARN("Cannot find BA ID for LM ref frame: %s", boost::lexical_cast<std::string>(lm_id.ref_frame_id).c_str());
        continue;
      }

      uint32_t ba_id = ba_id_it->second;
      if (ba_id == UINT_MAX) {
        ROS_WARN("BA ID is UINT_MAX");
        continue;
      }

      auto cam_id_it = camera_ids_.find(lm_id.ref_frame_id.session_id);
      if (cam_id_it == camera_ids_.end()) {
        ROS_WARN("Missing camera for session %s",
                     boost::lexical_cast<std::string>(lm_id.ref_frame_id.session_id).c_str());
        continue;
      }

      uint32_t base_cam = UINT_MAX;
      frame->GetLandmarkBaseCamera(lm_id.landmark_index, &base_cam);
      CHECK_LT(base_cam, cam_id_it->second.size());
      uint32_t cam_id = cam_id_it->second[base_cam];

      Eigen::Vector4t x_r;
      const std::vector<MeasurementId>* track = nullptr;
      if (!frame->GetLandmarkFeatureTrackRef(lm_id.landmark_index, track) ||
          !frame->GetLandmarkXr(lm_id.landmark_index, &x_r) ||
          !track) {
        continue;
      }

      const Sophus::SE3t& t_wr = ba_->GetPose(ba_id).t_wp;
      bool added_lmk = false;

      // Add all projection residuals
      for (const MeasurementId& zid : *track) {
        auto z_frame_ba_id_it = ba_frames_->find(zid.frame_id);
        if (z_frame_ba_id_it == ba_frames_->end()) continue;

        uint32_t ref_frame_ba_id = z_frame_ba_id_it->second;
        SlamFramePtr z_frame = map_->GetFramePtr(zid.frame_id);
        if (!z_frame ||
            !z_frame->HasGoodMeasurement(zid) ||
            !z_frame->GetMeasurement(zid, &z)) {
          continue;
        }

        auto cam_id_it = camera_ids_.find(zid.frame_id.session_id);
        const std::vector<uint32_t>& cam_ids = cam_id_it->second;

        CHECK_EQ(z.NumCameras(), cam_ids.size());
        for (size_t ii = 0 ; ii < z.NumCameras(); ++ii) {
          if (z.HasGoodMeasurementInCam(ii)) {
            if (cam_id_it != camera_ids_.end()) {
              if (!added_lmk) {
                lm_ba_id = ba_->AddLandmark(Sophus::MultHomogeneous(t_wr, x_r),
                                            ba_id, cam_id, landmarks_active_);
                added_lmk = true;
              }

              ba_->AddProjectionResidual(
                  z.Pixel(ii), ref_frame_ba_id, lm_ba_id, cam_ids[ii], 2.0);
            }
          }
        }
      }
    }
  }

  void set_landmarks_active(bool active) {
    landmarks_active_ = active;
  }

  void set_use_imu(bool use_imu) {
    use_imu_ = use_imu;
  }

 protected:
  inline void AddPose(const Sophus::SE3t& t_wp,
                      const ReferenceFrameId& id,
                      const SlamFramePtr& frame,
                      bool is_active) {
    uint32_t ba_id = UINT_MAX;
    if (use_imu_) {
      ba_id = ba_->AddPose(t_wp,
                           frame->t_vs(),
                           frame->cam_params(),
                           t_wp * frame->v_r(),
                           frame->b(),
                           is_active);
    } else {
      ba_id = ba_->AddPose(t_wp, is_active);
    }

    CHECK_NE(ba_id, UINT_MAX);
    ba_frames_->emplace(id, ba_id);
  }

 private:
  const SlamMap* map_;
  BundleAdjuster* ba_;
  std::unordered_map<ReferenceFrameId, uint32_t>* ba_frames_;
  std::unordered_map<LandmarkId, uint32_t>* landmarks_;
  std::unordered_map<SessionId, std::vector<uint32_t> > camera_ids_;
  std::unordered_set<ReferenceFrameId> to_find_;
  bool landmarks_active_ = true;
  bool use_imu_ = false;
  Scalar root_time_ = 0.0;
};
