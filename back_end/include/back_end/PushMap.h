// Copyright (c) Jack Morrison, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <ba/Types.h>
#include <ba/InterpolationBuffer.h>
//#include <CVars/CVar.h>
#include <back_end/BackEndConfig.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/SlamMap.h>
#include <slam_map/TransformEdgeId.h>
#include <slam_map/TransformEdge.h>

namespace rslam {

/**
 * Push the map from the bundle adjuster back into the map
 *
 * @param BundleAdjuster Templated BA type, used to retrieve local info
 * @param edge_ids Mapping from TransformEdgeId -> ba_id for IMU residual
 * @param pose_ids Mapping from ReferenceFrameId -> ba_id (uint32_t)
 * @param landmark_ba_ids Mapping from LandmarkId -> ba_id (uint32_t)
 *
 * @note If IMU is not being used, set all edge_id.second values to UINT_MAX
 * @todo Remove all mention of UINT_MAX
 */
template <class BundleAdjuster, class EdgeBaIds, class PoseBaIds,
          class LandmarkBaIds>
bool PushMap(const BundleAdjuster& ba,
             const EdgeBaIds& edge_ids,
             const PoseBaIds& pose_ids,
             const LandmarkBaIds& landmark_ba_ids,
             const ba::InterpolationBufferT<ba::ImuMeasurementT<Scalar>,
             Scalar>* imu_buffer,
             SlamMap* map) {
  //static double& g_imu_visualization_time_extra =
   //   CVarUtils::CreateGetCVar<>("gui.ImuVisualizationTimeExtra", 0.3, "");

  std::unordered_set<ReferenceFrameId> updated_frames;
  for (const std::pair<TransformEdgeId, uint32_t>& pair : edge_ids) {
    const TransformEdgeId& edge_id = pair.first;
    auto start_it = pose_ids.find(edge_id.start);
    auto end_it = pose_ids.find(edge_id.end);
    CHECK_NE(start_it, pose_ids.end());
    CHECK_NE(end_it, pose_ids.end());

    const ba::PoseT<Scalar>& pose_start = ba.GetPose(start_it->second);
    const ba::PoseT<Scalar>& pose_end = ba.GetPose(end_it->second);
    if (!pose_start.is_active && !pose_end.is_active) continue;

    const Sophus::SE3t& start_tpw = pose_start.t_wp.inverse();
    const Sophus::SE3t& end_tpw = pose_end.t_wp.inverse();
    if (SlamEdgePtr edge = map->GetEdgePtr(edge_id)) {
      edge->set_transform(start_tpw * pose_end.t_wp);
    }

    // Proxy for whether the IMU is being used or not.
    Eigen::Vector3t g_w = ba.GetGravity();
    if (ba.kVelInState) {
      if (pose_start.is_active && !updated_frames.count(edge_id.start)) {
        SlamFramePtr frame = map->GetFramePtr(edge_id.start);
        frame->set_b(pose_start.b);
        frame->set_g_r(start_tpw.so3() * g_w);
        frame->set_v_r(start_tpw.so3() * pose_start.v_w);
        frame->set_t_vs(pose_start.t_vs);
        frame->set_cam_params(pose_start.cam_params);
        updated_frames.insert(edge_id.start);
      }

      if (pose_end.is_active && !updated_frames.count(edge_id.end)) {
        SlamFramePtr frame = map->GetFramePtr(edge_id.end);
        frame->set_b(pose_end.b);
        frame->set_g_r(end_tpw.so3() * g_w);
        frame->set_v_r(end_tpw.so3() * pose_end.v_w);
        frame->set_t_vs(pose_end.t_vs);
        frame->set_cam_params(pose_end.cam_params);
        updated_frames.insert(edge_id.end);
      }
    }

    if (ba.GetNumImuResiduals() > 0) {
      const auto& res = ba.GetImuResidual(pair.second);
      std::vector<ba::ImuPoseT<Scalar> > poses;
      if (res.pose1_id >= ba.GetNumPoses()) {
        LOG(ERROR) << "res.pose1_id is too large";
        continue;
      }

      std::vector<ba::ImuMeasurementT<Scalar> > meas =
          imu_buffer->GetRange(res.measurements.front().time,
                               res.measurements.back().time +
                               BackEndConfig::getConfig()->imu_visualization_time_extra);

      const ba::PoseT<Scalar>& pose = ba.GetPose(res.pose1_id);
      res.IntegrateResidual(pose, meas,
                            pose.b.head<3>(),
                            pose.b.tail<3>(), g_w, poses);

      // Go through the list and make sure every pose is relative
      // to the starting pose.
      Sophus::SE3t t_pw = ba.GetPose(res.pose1_id).t_wp.inverse();
      for (auto& pose : poses) {
        pose.t_wp = t_pw * pose.t_wp;
      }

      //obtain the relative gravity direction
      const Eigen::Vector3t g_p =
          t_pw.so3() * g_w.normalized();
      map->UpdateEdgeImuPoints(edge_id, poses, g_p);
    }
  }

  LandmarkState lm_state;
  for (const std::pair<LandmarkId, uint32_t>& pair : landmark_ba_ids) {
    const LandmarkId& lm_id = pair.first;
    const uint32_t& lm_index = lm_id.landmark_index;
    const uint32_t& ba_id = pair.second;
    if (ba_id == UINT_MAX) continue;
    if (ba_id >= ba.GetNumLandmarks()) {
      LOG(WARNING) << "LM ba_id too high: " << ba_id << " vs. "
                   << ba.GetNumLandmarks();
      continue;
    }

    SlamFramePtr frame = map->GetFramePtr(lm_id.ref_frame_id);
    if (!frame) continue;

    const ba::LandmarkT<Scalar>& ba_lm = ba.GetLandmarkObj(ba_id);
    if (!ba_lm.is_reliable) {
      frame->SetLandmarkState(lm_id.landmark_index, eLmkNotReliable);
      continue;
    }

    Eigen::Vector4t lm_x_w = ba_lm.x_w;
    if (std::abs(lm_x_w[3]) < 1e-6) {
      LOG(ERROR) << "WARNING --> abs(inverse depth) < 1e-6, possible division"
                 << " by zero to follow."
                 << "\n\tReliable: " << ba_lm.is_reliable
                 << "\n\tx_w: " << ba_lm.x_w.transpose()
                 << "\n\t# residuals: " << ba_lm.proj_residuals.size()
                 << "\n\t# outliers: " << ba_lm.num_outlier_residuals;
    }
    lm_x_w /= lm_x_w[3];

    auto pose_it = pose_ids.find(lm_id.ref_frame_id);
    if (pose_it == pose_ids.end()) {
      LOG(ERROR) << "Landmark pose " << lm_id << " not in BA pose_ids";
      continue;
    }

    const ba::PoseT<Scalar>& pose = ba.GetPose(pose_it->second);

    Eigen::Vector4t lm_x_r = Sophus::MultHomogeneous(pose.t_wp.inverse(),
                                                     lm_x_w);
    frame->SetLandmarkXr(lm_id.landmark_index, lm_x_r);

    CameraRigPtr rig = map->GetCamera(lm_id.ref_frame_id.session_id);
    if (!rig) continue;

    uint32_t base_cam = UINT_MAX;
    if (!frame->GetLandmarkBaseCamera(lm_index, &base_cam)) continue;
    CHECK_LT(base_cam, rig->cameras.size());

    const Eigen::Vector4t x_c =
        Sophus::MultHomogeneous(rig->cameras[base_cam].T_wc.inverse(), lm_x_r);
    Scalar plane_extent = 0.0;
    if (!frame->GetLandmarkCamPlaneExtent(lm_index, &plane_extent)) continue;
    frame->SetLandmarkExtent(lm_id.landmark_index,  plane_extent * x_c.norm());

    // If the landmark is judged good by the BA and it was previously
    // at infinity because of a mono camera, we now set it as well estimated.
    if (!frame->GetLandmarkState(lm_index, &lm_state)) continue;
    if (lm_state == eLmkAtInfinity) {
      frame->SetLandmarkState(lm_id.landmark_index, eLmkMono);
    }
  }
  return true;
}
}  // end namespace rslam
