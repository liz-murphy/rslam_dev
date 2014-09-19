#pragma once

#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <slam_map/TransformEdgeId.h>
#include <slam_map/LandmarkId.h>
#include <sparse_tracking/LandmarkContainer.h>
#include <utils/MathTypes.h>

#include <calibu/Calibu.h>
#include <ba/Types.h>

struct MeasurementContainer {
  MeasurementContainer(const MultiViewMeasurement& z,
                       const unsigned int cam_id) :
      z(z.Pixel(cam_id)),
      id(z.id()),
      cam_id(cam_id) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2t z;
  MeasurementId id;
  unsigned int cam_id;
};

struct PoseContainerT : public ba::ImuPoseT<Scalar> {
  PoseContainerT() : ba::ImuPoseT<Scalar>(Sophus::SE3Group<Scalar>(),
                                          Eigen::Vector3t::Zero(),
                                          Eigen::Vector3t::Zero(),
                                          0),
                     ba_id(UINT_MAX),
                     residual_id(UINT_MAX),
                     b(Eigen::Vector6t::Zero()),
                     is_static_set_pose(false) {
  }

  PoseContainerT(const Sophus::SE3Group<Scalar>& twp,
                 const Sophus::SE3Group<Scalar>& tvs,
                 const Eigen::VectorXt camParams,
                 const Eigen::Vector3t& v,
                 const Eigen::Vector3t& w,
                 const Eigen::Vector6t& b,
                 const Eigen::Vector3t& g,
                 const double time) :
      ba::ImuPoseT<Scalar>(twp, v, w, time),
      ba_id(UINT_MAX),
      residual_id(UINT_MAX),
      b(b), g(g), t_vs(tvs), cam_params(camParams),
      is_static_set_pose(false) {
  }

  unsigned int ba_id;
  unsigned int residual_id;
  Eigen::Vector6t b;
  Eigen::Vector3t g;
  Sophus::SE3t t_vs;
  Eigen::VectorXt cam_params;
  std::vector<std::shared_ptr<MeasurementContainer> > measurements;
  std::vector<std::shared_ptr<LandmarkContainer> > ref_landmarks;

  // Should we set all of our landmarks to
  bool is_static_set_pose;

  // By default, poses have all parameters optimized, but sometimes we
  // may not and we want to signal that only the pose should be pushed
  // back to the map.
  bool only_pose_optimized = false;
};

using ReferenceFramePoseMap = std::map<ReferenceFrameId, PoseContainerT>;

struct LocalMap {
  LocalMap() : depth(-1), map(nullptr), num_measurements(-1) {}

  unsigned int                            depth;
  ReferenceFrameId                        root_id;
  std::map<LandmarkId, std::shared_ptr<LandmarkContainer> > landmarks;
  ReferenceFramePoseMap                   poses;
  std::shared_ptr<PoseContainerT>         static_set_pose;
  ReferenceFrameId                        static_set_pose_id;
  std::set<ReferenceFrameId>              outside_pose_ids;
  std::map<TransformEdgeId, TransformEdgeId> inside_edge_ids;
  Eigen::Vector3t                         g_w;

  const SlamMap*                          map;
  unsigned int                            num_measurements;
  std::vector<Sophus::SE3t>               t_sw;
  std::map<SessionId, CameraRigPtr>       cameras;
  std::set<SessionId>                     maps;

  // Root for the BA. The oldest parent-chain node.
  ReferenceFrameId                        ba_root_id;

  // The entire parent chain. Not in order.
  std::unordered_set<ReferenceFrameId>    parent_chain_ids;

  // Frames in the static set, plus the transform
  // special_static_from_this_id
  std::unordered_map<ReferenceFrameId, Sophus::SE3t> static_poses;
  std::vector<TransformEdgeId> active_static_edges;

  void PushRelativeCoordsToGlobal() {
    for (auto& pair: landmarks) {
      const LandmarkId& lm_id = pair.first;

      // landmarks base frame woot
      const auto it = poses.find(lm_id.ref_frame_id);

      // pose of base frame for this lm
      const Sophus::SE3t& t_wr = it->second.t_wp;
      pair.second->x_w =  Sophus::MultHomogeneous(t_wr, pair.second->x_r);
    }

    // transform velocities from global to relative coords
    for (auto& pair: poses) {
      pair.second.v_w = pair.second.t_wp.so3() * pair.second.v_w;

      if (pair.first == root_id) {
        g_w = (pair.second.t_wp.so3() * pair.second.g);
      }
    }
  }

  void PushGlobalCoordsToRelative() {
    for (auto& pair: landmarks) {
      const ReferenceFrameId& frame_id = pair.first.ref_frame_id;
      const SessionId lm_session_id = frame_id.session_id;
      const unsigned int lm_cam_id = pair.second->base_camera_id;
      const Sophus::SE3t t_cw = cameras[lm_session_id]->cameras[lm_cam_id].T_wc.inverse();
      const auto& it = poses.find(frame_id);

      // to calclate the depth along the ray, we must take the landmark
      // into the sensor frame
      const double old_depth =
          Sophus::MultHomogeneous(t_cw, pair.second->x_r).norm();
      pair.second->x_r =
          Sophus::MultHomogeneous(it->second.t_wp.inverse(), pair.second->x_w);

      // now calculate the depth after we update the landmark, and use that to
      // establish the depth ratio which is later used to adjust
      // the extent of the landmark in 3D.
      pair.second->depth_ratio =
          Sophus::MultHomogeneous(t_cw, pair.second->x_r).norm() / old_depth;
    }

    for (auto& pair: poses) {
      pair.second.v_w = pair.second.t_wp.so3().inverse() * pair.second.v_w;
      pair.second.g = (pair.second.t_wp.so3().inverse() * g_w);
    }
  }

  void Clear() {
    landmarks.clear();
    poses.clear();
    outside_pose_ids.clear();
    inside_edge_ids.clear();
    t_sw.clear();
  }
};
